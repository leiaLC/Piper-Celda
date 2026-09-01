#!/usr/bin/env python3
"""
Secuencia de agarre de una laminilla por los CANTOS.

Por que por los cantos: en el escurridor las ranuras estan a 8 mm de paso,
asi que con las vecinas ocupadas no caben dedos entre laminilla y laminilla.
A lo largo de la ranura quedan ~30 mm libres, y ahi si.

Geometria (derivada de los modelos de la celda):
    laminilla 76 x 26 x 1 mm, de canto, centro en z = 0.798
    borde superior en z = 0.836
    punto de agarre 20 mm por debajo -> z = 0.816
    el espesor de 1 mm va en X (direccion de las ranuras)
    los 26 mm de ancho van en Y  ->  la pinza cierra a lo largo de Y
    aproximacion vertical: el eje Z del TCP mira hacia -Z del mundo

La secuencia:
    1. abre la pinza
    2. permite el contacto dedos-laminilla en la matriz de colisiones
       (sin esto no hay plan posible: agarrar es tocar)
    3. va a la pose de preagarre, a 'aproximacion' metros por encima
    4. baja en linea recta (trayectoria cartesiana, no articular)
    5. cierra la pinza
    6. adjunta la laminilla al robot
    7. sube en linea recta

Uso:
    ros2 run piper_celda_moveit agarrar_laminilla.py
    ros2 run piper_celda_moveit agarrar_laminilla.py --solo-planificar
    ros2 run piper_celda_moveit agarrar_laminilla.py --objeto laminilla_02
"""
import argparse
import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetStateValidity,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GRUPO = "brazo"
TCP = "tcp"
MARCO = "world"
PINZA_LINKS = ["Link7", "Link8", "gripper_base", "Link6"]
BRAZO_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def cuaternion_a_matriz(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:          # cuaternion sin inicializar: tratar como identidad
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ]


# Geometria de la laminilla en el mundo: espesor 1 mm en X (direccion de las
# ranuras), ancho 26 mm en Y, alto 76 mm en Z.
ESPESOR = 0.001
ANCHO = 0.026

# En las dos orientaciones el eje Z del TCP mira hacia -Z del mundo
# (aproximacion vertical). Cambia el eje X, que es por donde cierran los dedos.
ORIENTACIONES = {
    # cierra sobre los 26 mm de ancho; la mordaza apoya en 1 mm de canto
    "cantos": dict(quat=(math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0),
                   semi=ANCHO / 2),
    # cierra sobre 1 mm de espesor; la mordaza apoya en 26 mm de cara
    "caras": dict(quat=(1.0, 0.0, 0.0, 0.0),
                  semi=ESPESOR / 2),
}


def quaternion_agarre(modo):
    q = ORIENTACIONES[modo]["quat"]
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class Agarre(Node):
    def __init__(self, a):
        super().__init__("agarrar_laminilla")
        self.a = a
        self.obj = a.objeto
        # Estado al final del tramo anterior. En modo seco el robot no se
        # mueve, asi que cada tramo debe partir de donde acabaria el previo;
        # si no, el segundo tramo cartesiano se calcula desde la pose actual
        # del brazo y devuelve 0 %.
        self.estado = None

        self.cli_get = self.create_client(GetPlanningScene, "get_planning_scene")
        self.cli_set = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.cli_cart = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.cli_val = self.create_client(GetStateValidity, "check_state_validity")
        self.ac_move = ActionClient(self, MoveGroup, "move_action")
        self.ac_exec = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.ac_pinza = ActionClient(
            self, FollowJointTrajectory,
            "/pinza_controller/follow_joint_trajectory")

        for c, n in ((self.cli_get, "get_planning_scene"),
                     (self.cli_set, "apply_planning_scene"),
                     (self.cli_cart, "compute_cartesian_path")):
            if not c.wait_for_service(timeout_sec=20.0):
                self.fatal(f"{n} no responde. Lanza MoveIt primero.")
        for c, n in ((self.ac_move, "move_action"),
                     (self.ac_exec, "execute_trajectory"),
                     (self.ac_pinza, "pinza_controller")):
            if not c.wait_for_server(timeout_sec=20.0):
                self.fatal(f"la accion {n} no responde.")

    def fatal(self, msg):
        self.get_logger().error(msg)
        raise SystemExit(1)

    def paso(self, n, txt):
        self.get_logger().info(f"[{n}] {txt}")

    def fijar_estado(self, traj):
        if not traj.points:
            return
        e = RobotState()
        e.joint_state.name = list(traj.joint_names)
        e.joint_state.position = list(traj.points[-1].positions)
        e.is_diff = True
        self.estado = e

    def inicio(self, req):
        if self.estado is not None:
            req.start_state = self.estado
        else:
            req.start_state.is_diff = True

    def esperar(self, fut, t=60.0):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=t)
        return fut.result()

    # ---------------- escena ----------------
    def pose_laminilla(self):
        """Lee la pose real de la laminilla en la escena, para que el agarre
        siga lo que muevas en Gazebo en vez de una cifra escrita a mano."""
        req = GetPlanningScene.Request()
        req.components.components = req.components.WORLD_OBJECT_GEOMETRY
        r = self.esperar(self.cli_get.call_async(req))
        if r is None:
            self.fatal("no pude leer la escena")
        for co in r.scene.world.collision_objects:
            if co.id != self.obj:
                continue
            poses = list(co.primitive_poses) + list(co.mesh_poses)
            if not poses:
                self.fatal(f"{self.obj} no tiene geometria")
            # MoveIt no devuelve la geometria en coordenadas del mundo: guarda
            # una pose de objeto (co.pose) y las formas RELATIVAS a ella. Hay
            # que componer las dos o el punto sale en el origen del mundo.
            o = co.pose.position
            rel = poses[0].position
            qo = co.pose.orientation
            R = cuaternion_a_matriz(qo.x, qo.y, qo.z, qo.w)
            p = Point(
                x=o.x + R[0][0] * rel.x + R[0][1] * rel.y + R[0][2] * rel.z,
                y=o.y + R[1][0] * rel.x + R[1][1] * rel.y + R[1][2] * rel.z,
                z=o.z + R[2][0] * rel.x + R[2][1] * rel.y + R[2][2] * rel.z,
            )
            alto = self.a.alto
            for prim in co.primitives:
                if prim.type == SolidPrimitive.BOX and len(prim.dimensions) == 3:
                    alto = max(prim.dimensions)
            return (p.x, p.y, p.z), alto
        self.fatal(f"{self.obj} no esta en la escena. "
                   "Corre publicar_escena_celda.py")

    def acm_permitir(self, valor=True):
        """Permite que la laminilla toque los dedos y el escurridor.

        Lo del escurridor no es una concesion: la laminilla ARRANCA metida en
        la ranura. En cuanto se adjunta al robot, esa interpenetracion cuenta
        como colision robot-mundo y ninguna trayectoria de salida es valida.
        """
        req = GetPlanningScene.Request()
        req.components.components = req.components.ALLOWED_COLLISION_MATRIX
        r = self.esperar(self.cli_get.call_async(req))
        acm = r.scene.allowed_collision_matrix
        for link in PINZA_LINKS + list(self.a.permitir_con):
            for n in (self.obj, link):
                if n not in acm.entry_names:
                    acm.entry_names.append(n)
                    for fila in acm.entry_values:
                        fila.enabled.append(False)
                    e = AllowedCollisionEntry()
                    e.enabled = [False] * len(acm.entry_names)
                    acm.entry_values.append(e)
            i = acm.entry_names.index(self.obj)
            j = acm.entry_names.index(link)
            acm.entry_values[i].enabled[j] = valor
            acm.entry_values[j].enabled[i] = valor
        esc = r.scene
        esc.is_diff = True
        esc.robot_state.is_diff = True
        self.esperar(self.cli_set.call_async(ApplyPlanningScene.Request(scene=esc)))

    def adjuntar(self):
        """Adjunta la laminilla al robot.

        MoveIt calcula la posicion del objeto RELATIVA al link usando el
        estado del robot en ese momento. Si se adjunta con el brazo en casa,
        la laminilla queda enganchada con el desfase equivocado y aparece
        dentro de la propia base. Por eso en modo seco hay que declarar
        explicitamente que el robot esta en la configuracion de agarre.
        """
        aco = AttachedCollisionObject()
        aco.link_name = "Link7"
        aco.object.id = self.obj
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = PINZA_LINKS

        esc = PlanningScene()
        esc.is_diff = True
        if self.estado is not None:
            esc.robot_state = self.estado
        esc.robot_state.is_diff = True
        esc.robot_state.attached_collision_objects.append(aco)
        # NO se envia un REMOVE del objeto en el mundo: adjuntar por id ya lo
        # saca de ahi. Si se manda, MoveIt procesa el borrado primero y luego
        # no encuentra nada que adjuntar: queda un cuerpo vacio en la pinza y
        # avisa con "The attached body for link 'Link7' has no geometry".
        self.esperar(self.cli_set.call_async(ApplyPlanningScene.Request(scene=esc)))

    # ---------------- movimiento ----------------
    def restricciones(self, pose):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = MARCO
        pc.link_name = TCP
        pc.target_point_offset = Vector3()
        bv = BoundingVolume()
        esf = SolidPrimitive()
        esf.type = SolidPrimitive.SPHERE
        esf.dimensions = [self.a.tolerancia]
        bv.primitives.append(esf)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = MARCO
        oc.link_name = TCP
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def ir_a(self, pose, etiqueta):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = GRUPO
        # Sin start_state, move_group avisa con "Found empty JointState
        # message" y planifica desde ceros en lugar de desde el brazo.
        self.inicio(r)
        r.num_planning_attempts = 12
        r.allowed_planning_time = 8.0
        r.max_velocity_scaling_factor = self.a.velocidad
        r.max_acceleration_scaling_factor = self.a.velocidad
        r.goal_constraints.append(self.restricciones(pose))
        goal.planning_options.plan_only = self.a.solo_planificar

        h = self.esperar(self.ac_move.send_goal_async(goal))
        if h is None or not h.accepted:
            self.fatal(f"{etiqueta}: goal rechazado")
        res = self.esperar(h.get_result_async(), t=90.0)
        code = res.result.error_code.val
        if code != 1:
            self.fatal(f"{etiqueta}: fallo del planificador, codigo {code}")
        if self.a.solo_planificar:
            self.fijar_estado(res.result.planned_trajectory.joint_trajectory)
        else:
            # Ejecutado de verdad: el siguiente tramo parte del estado
            # MEDIDO, no del planificado, para no arrastrar la desviacion.
            self.estado = None
        self.get_logger().info(f"   {etiqueta}: ok")

    def recta(self, desde, hasta, etiqueta, evitar=True):
        """Trayectoria cartesiana. Para bajar sobre la laminilla hace falta
        linea recta: un plan articular puede describir un arco y barrer las
        laminillas vecinas."""
        req = GetCartesianPath.Request()
        req.header.frame_id = MARCO
        req.group_name = GRUPO
        req.link_name = TCP
        self.inicio(req)
        req.waypoints = [desde, hasta]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = evitar
        req.max_velocity_scaling_factor = self.a.velocidad
        req.max_acceleration_scaling_factor = self.a.velocidad
        r = self.esperar(self.cli_cart.call_async(req), t=60.0)
        if r is None:
            self.fatal(f"{etiqueta}: sin respuesta")
        frac = r.fraction
        self.get_logger().info(f"   {etiqueta}: {100*frac:.0f} % de la recta resuelto")
        if frac < 0.95:
            self.diagnostico(etiqueta)
            self.fatal(f"{etiqueta}: solo {100*frac:.0f} %. Revisa la holgura.")
        if self.a.solo_planificar:
            self.fijar_estado(r.solution.joint_trajectory)
            return
        g = ExecuteTrajectory.Goal()
        g.trajectory = r.solution
        h = self.esperar(self.ac_exec.send_goal_async(g))
        if h is None or not h.accepted:
            self.fatal(f"{etiqueta}: ejecucion rechazada")
        self.esperar(h.get_result_async(), t=90.0)
        self.estado = None

    def diagnostico(self, etiqueta):
        """Si la recta no sale, casi siempre es que el punto de partida ya
        esta en colision. Dice contra que, en vez de dejarte adivinando."""
        if self.estado is None or not self.cli_val.wait_for_service(timeout_sec=3.0):
            return
        req = GetStateValidity.Request()
        req.robot_state = self.estado
        req.group_name = GRUPO
        r = self.esperar(self.cli_val.call_async(req), t=15.0)
        if r is None:
            return
        if r.valid:
            self.get_logger().warn(
                f"   {etiqueta}: el estado inicial es valido; el problema esta "
                "mas adelante en el recorrido (alcance o singularidad)")
            return
        self.get_logger().error(f"   {etiqueta}: el estado inicial YA colisiona:")
        for c in r.contacts:
            self.get_logger().error(f"      {c.contact_body_1}  <->  {c.contact_body_2}")
        if not r.contacts:
            self.get_logger().error("      (sin detalle de contactos)")

    def pinza(self, apertura, etiqueta):
        if self.a.solo_planificar:
            self.get_logger().info(f"   {etiqueta}: omitida (solo planificar)")
            return
        g = FollowJointTrajectory.Goal()
        t = JointTrajectory()
        # Los dos dedos, en espejo. Ya no hay <mimic> que lo haga solo.
        t.joint_names = ["joint7", "joint8"]
        p = JointTrajectoryPoint()
        p.positions = [apertura, -apertura]
        p.time_from_start.sec = 3
        t.points.append(p)
        g.trajectory = t
        h = self.esperar(self.ac_pinza.send_goal_async(g))
        if h is None or not h.accepted:
            self.fatal(f"{etiqueta}: goal rechazado")
        self.esperar(h.get_result_async(), t=30.0)
        # Los dedos siguen moviendose un instante despues de que la accion
        # reporta exito. Sin esta pausa el brazo arranca el descenso con la
        # pinza todavia abriendo, y en Gazebo eso empuja la laminilla.
        time.sleep(1.0)
        self.get_logger().info(f"   {etiqueta}: joint7 = {apertura:.3f} m")

    # ---------------- secuencia ----------------
    def ejecutar(self):
        # Cada dedo se separa 'joint7' del centro, asi que la abertura total
        # es el doble. Depende de por donde se agarre: 26 mm por cantos, 1 mm
        # por caras. Por eso no puede haber un valor unico por defecto.
        semi = ORIENTACIONES[self.a.orientacion]["semi"]
        if self.a.abierta is None:
            self.a.abierta = min(0.035, semi + self.a.holgura)
        if self.a.cerrada is None:
            self.a.cerrada = max(0.0, semi - self.a.apriete)
        self.get_logger().info(
            f"agarre por {self.a.orientacion}: joint7 abierta "
            f"{self.a.abierta*1000:.1f} mm, cerrada {self.a.cerrada*1000:.1f} mm")

        centro, alto = self.pose_laminilla()
        if centro[2] < 0.1:
            self.fatal(
                f"la laminilla aparece en z = {centro[2]:.3f}, bajo la mesa. "
                "La escena no esta bien publicada: corre "
                "publicar_escena_celda.py y vuelve a intentarlo.")
        z_agarre = centro[2] + alto / 2 - self.a.bajo_borde
        q = quaternion_agarre(self.a.orientacion)

        agarre = Pose(position=Point(x=centro[0], y=centro[1], z=z_agarre),
                      orientation=q)
        pre = Pose(position=Point(x=centro[0], y=centro[1],
                                  z=z_agarre + self.a.aproximacion),
                   orientation=q)
        salida = Pose(position=Point(x=centro[0], y=centro[1],
                                     z=z_agarre + self.a.retirada),
                      orientation=q)

        print()
        self.get_logger().info(
            f"laminilla en ({centro[0]:+.3f}, {centro[1]:+.3f}, {centro[2]:.3f}), "
            f"alto {alto*1000:.0f} mm")
        self.get_logger().info(f"agarre a z = {z_agarre:.3f}, "
                               f"preagarre a z = {pre.position.z:.3f}")
        print()

        self.paso(1, "abriendo pinza")
        self.pinza(self.a.abierta, "abrir")

        self.paso(2, "permitiendo contacto dedos-laminilla")
        self.acm_permitir(True)

        self.paso(3, "a la pose de preagarre")
        self.ir_a(pre, "preagarre")

        # El descenso va en dos tramos. Los ultimos milimetros sobre la
        # laminilla son contacto por definicion: exigir ausencia de colision
        # ahi es contradictorio y el planificador se planta justo al llegar
        # al borde superior. Asi que solo el tramo final se libera, y es
        # corto y estrictamente vertical.
        contacto = Pose(
            position=Point(x=centro[0], y=centro[1], z=z_agarre + self.a.contacto),
            orientation=q)

        self.paso(4, f"descenso vertical (libre hasta "
                     f"{self.a.contacto*1000:.0f} mm del agarre)")
        self.recta(pre, contacto, "descenso", evitar=True)
        self.recta(contacto, agarre, "aproximacion final", evitar=False)

        self.paso(5, "cerrando pinza")
        self.pinza(self.a.cerrada, "cerrar")
        time.sleep(0.5)

        self.paso(6, "adjuntando la laminilla al robot")
        self.adjuntar()
        if self.a.solo_planificar:
            self.get_logger().warn(
                "modo seco: la laminilla queda adjunta en la escena. "
                "Para devolverla a su sitio, vuelve a correr "
                "publicar_escena_celda.py")

        self.paso(7, "retirada vertical")
        self.recta(agarre, contacto, "extraccion", evitar=False)
        self.recta(contacto, salida, "retirada", evitar=True)

        print()
        self.get_logger().info("laminilla agarrada y en transito")
        self.get_logger().info(
            "para soltarla: ros2 run piper_celda_moveit "
            "manipular_laminilla.py soltar")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objeto", default="laminilla_01")
    ap.add_argument("--aproximacion", type=float, default=0.10,
                    help="altura del preagarre sobre el punto de agarre, m")
    ap.add_argument("--retirada", type=float, default=0.08,
                    help="altura de salida tras cerrar, m")
    ap.add_argument("--bajo-borde", type=float, default=0.020,
                    help="cuanto por debajo del borde superior se agarra, m")
    ap.add_argument("--alto", type=float, default=0.076,
                    help="alto de la laminilla si la escena no lo dice, m")
    ap.add_argument("--orientacion", choices=["caras", "cantos"], default="caras",
                    help="caras: la mordaza apoya 26 mm. cantos: apoya 1 mm "
                         "pero cabe con las ranuras vecinas ocupadas")
    ap.add_argument("--holgura", type=float, default=0.010,
                    help="separacion extra por lado al abrir, m")
    ap.add_argument("--apriete", type=float, default=0.001,
                    help="interferencia por lado al cerrar, m")
    ap.add_argument("--abierta", type=float, default=None,
                    help="joint7 al abrir; por defecto se deduce")
    ap.add_argument("--cerrada", type=float, default=None,
                    help="joint7 al cerrar; por defecto se deduce")
    ap.add_argument("--permitir-con", nargs="*",
                    default=["escurridor_entrada"],
                    help="objetos de la escena que la laminilla puede tocar")
    ap.add_argument("--contacto", type=float, default=0.035,
                    help="tramo final sin comprobar colisiones, m. Debe cubrir "
                         "lo que la laminilla sobresale por encima del agarre")
    ap.add_argument("--tolerancia", type=float, default=0.004,
                    help="radio de la esfera de destino, m")
    ap.add_argument("--velocidad", type=float, default=0.2)
    ap.add_argument("--solo-planificar", action="store_true",
                    help="planifica y muestra, sin mover el robot")
    a = ap.parse_args([x for x in sys.argv[1:] if not x.startswith("--ros-args")])

    rclpy.init()
    try:
        Agarre(a).ejecutar()
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
