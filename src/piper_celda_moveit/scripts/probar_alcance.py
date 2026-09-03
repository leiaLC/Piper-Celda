#!/usr/bin/env python3
"""
Prueba de alcance sobre las ranuras de los racks. NO mueve el brazo nunca.

Responde dos preguntas distintas, y conviene no confundirlas:

  --barrido   Cinematica inversa sobre las 90 ranuras de un rack, con
              deteccion de colisiones. Rapido (unos 50 ms por ranura) y
              devuelve un mapa de 5 x 18. Dice si EXISTE una configuracion
              del brazo que ponga el TCP en cada ranura.

  (por defecto)
              Plan completo con MoveGroup a UNA ranura, en modo plan_only.
              Mas lento pero dice algo mas fuerte: que existe un camino
              hasta ahi desde donde esta el brazo ahora.

  Una ranura puede tener IK y no tener plan (encerrada), pero si no tiene
  IK no hay nada que buscar. Por eso el barrido va primero.

DE DONDE SALEN LAS COTAS
  Ninguna esta escrita aqui.
    poses de los racks .... <include> de worlds/celda_piper.sdf
    rejilla de ranuras .... <visual name="lam_C_F"> de
                            models/rack_laminillas_lleno/model.sdf
  Si mueves un rack en el mundo o regeneras el rack con otro paso de
  ranura, este script lo sigue solo.

ORIENTACION DE AGARRE
  En los dos modos el eje Z del TCP mira a -Z del mundo (aproximacion
  vertical). Cambia el eje X, que es por donde cierran los dedos.

    caras    cierra sobre las caras de 1 mm. Los dedos entran en el hueco
             ENTRE laminillas vecinas, que en este rack es de 5.8 mm.
    cantos   cierra sobre los cantos de 25.7 mm. Los dedos entran por los
             lados de la columna, donde los tabiques dejan 2 mm.

  OJO: estos nombres estan CAMBIADOS respecto a agarrar_laminilla.py.
  En el escurridor el espesor de la laminilla iba en X del mundo; en este
  rack va en Y. La misma etiqueta corresponde al cuaternion contrario.
  Aqui se declaran por el eje del mundo, no por herencia.

  Con el rack lleno NINGUNO de los dos modos es viable con la pinza de
  serie: el dedo mide 5.11 mm solo en sus primeros 5 mm de punta y 6.23 a
  los 10, contra 5.8 mm de hueco. Este script mide ALCANCE, no agarre.
  Por eso conviene correrlo con --sin-vecinas la primera vez.

    ros2 run piper_celda_moveit probar_alcance.py --barrido
    ros2 run piper_celda_moveit probar_alcance.py --barrido --rack rack_out_3
    ros2 run piper_celda_moveit probar_alcance.py --columna 5 --fila 1
    ros2 run piper_celda_moveit probar_alcance.py --columna 5 --fila 1 --modo cantos

Necesita move_group corriendo y la escena publicada.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

GRUPO = "brazo"
TCP = "tcp"
MARCO = "world"

# Cuaterniones (x, y, z, w). Los dos dejan Z del TCP hacia -Z del mundo.
#   caras   -> X del TCP sobre +Y del mundo (cierra sobre el espesor)
#   cantos  -> X del TCP sobre +X del mundo (cierra sobre el ancho)
R2 = math.sqrt(0.5)
ORIENTACIONES = {
    "caras": (R2, R2, 0.0, 0.0),
    "cantos": (1.0, 0.0, 0.0, 0.0),
}

# Codigos de moveit_msgs/MoveItErrorCodes que salen en la practica.
ERRORES = {
    1: "OK",
    -1: "FAILURE",
    -10: "START_STATE_IN_COLLISION",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -19: "TIMED_OUT",
    -31: "NO_IK_SOLUTION",
}


def texto_error(v):
    return ERRORES.get(v, f"codigo {v}")


# --------------------------------------------------------------- geometria
def leer_pose(el):
    """<pose>x y z r p y</pose> -> lista de 6. Ausente = identidad."""
    if el is None:
        return [0.0] * 6
    p = el.find("pose")
    if p is None or not (p.text or "").strip():
        return [0.0] * 6
    v = [float(x) for x in p.text.split()]
    return (v + [0.0] * 6)[:6]


def racks_del_mundo(ruta):
    """{nombre: (x, y, z, yaw)} de todos los <include> cuyo uri sea un rack."""
    raiz = ET.parse(ruta).getroot().find("world")
    out = {}
    for el in raiz.findall("include"):
        uri = el.find("uri")
        nom = el.find("name")
        if uri is None or nom is None:
            continue
        if "rack_laminillas" not in uri.text:
            continue
        p = leer_pose(el)
        out[nom.text.strip()] = (p[0], p[1], p[2], p[5])
    return out


def rejilla_de_ranuras(ruta):
    """{(col, fila): (x, y, z)} en el marco del rack, del modelo lleno.

    z es el CENTRO de la laminilla. El semialto sale del propio <box>, asi
    que si cambias el largo de la laminilla esto lo sigue."""
    modelo = ET.parse(ruta).getroot().find("model")
    link = modelo.find("link")
    celdas, semialto = {}, None
    for vis in link.findall("visual"):
        nom = vis.get("name", "")
        if not nom.startswith("lam_"):
            continue
        _, c, f = nom.split("_")
        p = leer_pose(vis)
        celdas[(int(c), int(f))] = (p[0], p[1], p[2])
        if semialto is None:
            caja = vis.find("geometry/box/size")
            semialto = float(caja.text.split()[2]) / 2.0
    if not celdas:
        sys.exit(f"ERROR: no hay visuales lam_C_F en {ruta}")
    return celdas, semialto


class Alcance(Node):
    def __init__(self, a):
        super().__init__("probar_alcance")
        self.a = a

        pkg_gz = get_package_share_directory("piper_celda_gazebo")
        mundo = os.path.join(pkg_gz, "worlds", "celda_piper.sdf")
        lleno = os.path.join(pkg_gz, "models", "rack_laminillas_lleno",
                             "model.sdf")
        self.racks = racks_del_mundo(mundo)
        self.celdas, self.semialto = rejilla_de_ranuras(lleno)

        cols = sorted({c for c, _ in self.celdas})
        filas = sorted({f for _, f in self.celdas})
        self.cols, self.filas = cols, filas
        self.get_logger().info(
            f"mundo: {len(self.racks)} racks {sorted(self.racks)}")
        self.get_logger().info(
            f"rejilla: {len(cols)} columnas x {len(filas)} filas, "
            f"laminilla de {self.semialto*2000:.0f} mm")

        if a.rack not in self.racks:
            sys.exit(f"ERROR: '{a.rack}' no esta en el mundo. "
                     f"Hay: {sorted(self.racks)}")

        self.cli_ik = self.create_client(GetPositionIK, "compute_ik")
        if not self.cli_ik.wait_for_service(timeout_sec=15.0):
            sys.exit("compute_ik no responde. Lanza MoveIt primero.")
        self.ac_move = ActionClient(self, MoveGroup, "move_action")

    # ---------------- poses ----------------
    def pose_agarre(self, rack, col, fila, alto_extra=0.0):
        """Pose del TCP para agarrar la laminilla de esa ranura.

        El punto de agarre va --bajo-canto metros por debajo del borde
        superior de la laminilla, no a una z absoluta escrita a mano."""
        rx, ry, rz, yaw = self.racks[rack]
        lx, ly, lz = self.celdas[(col, fila)]
        c, s = math.cos(yaw), math.sin(yaw)
        x = rx + c * lx - s * ly
        y = ry + s * lx + c * ly
        z = rz + lz + self.semialto - self.a.bajo_canto + alto_extra
        q = ORIENTACIONES[self.a.modo]
        return Pose(position=Point(x=x, y=y, z=z),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # ---------------- consultas ----------------
    def hay_ik(self, pose):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GRUPO
        r.ik_link_name = TCP
        r.pose_stamped = PoseStamped(pose=pose)
        r.pose_stamped.header.frame_id = MARCO
        r.avoid_collisions = not self.a.sin_colisiones
        r.robot_state.is_diff = True
        r.timeout = Duration(sec=0, nanosec=int(self.a.timeout_ik * 1e9))
        fut = self.cli_ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None:
            return -1
        return fut.result().error_code.val

    def hay_plan(self, pose, etiqueta):
        if not self.ac_move.wait_for_server(timeout_sec=15.0):
            sys.exit("move_action no responde")
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = GRUPO
        r.start_state.is_diff = True
        r.num_planning_attempts = 12
        r.allowed_planning_time = 8.0
        r.max_velocity_scaling_factor = 0.2
        r.max_acceleration_scaling_factor = 0.2
        r.goal_constraints.append(self.restricciones(pose))
        # plan_only SIEMPRE: esto es una prueba de alcance, no un movimiento.
        goal.planning_options.plan_only = True

        fut = self.ac_move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        h = fut.result()
        if h is None or not h.accepted:
            self.get_logger().error(f"{etiqueta}: goal rechazado")
            return -1
        fut2 = h.get_result_async()
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=90.0)
        res = fut2.result()
        if res is None:
            return -19
        code = res.result.error_code.val
        if code == 1:
            n = len(res.result.planned_trajectory.joint_trajectory.points)
            t = res.result.planned_trajectory.joint_trajectory.points[-1]
            seg = t.time_from_start.sec + t.time_from_start.nanosec * 1e-9
            self.get_logger().info(
                f"   {etiqueta}: plan de {n} puntos, {seg:.1f} s")
        return code

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

    # ---------------- modos ----------------
    def barrido(self):
        rack = self.a.rack
        rx, ry, _, _ = self.racks[rack]
        print(f"\nbarrido de IK sobre {rack}  (modo {self.a.modo}, "
              f"colisiones {'NO' if self.a.sin_colisiones else 'SI'})")
        print(f"agarre a {self.a.bajo_canto*1000:.0f} mm bajo el canto superior\n")

        mapa, fallos = {}, {}
        for f in self.filas:
            for c in self.cols:
                code = self.hay_ik(self.pose_agarre(rack, c, f))
                mapa[(c, f)] = code
                if code != 1:
                    fallos[code] = fallos.get(code, 0) + 1

        print("       " + "".join(f"  col{c} " for c in self.cols))
        for f in self.filas:
            fila = "".join("   ok  " if mapa[(c, f)] == 1 else "   --  "
                           for c in self.cols)
            print(f"fila{f:3d}" + fila)

        ok = sum(1 for v in mapa.values() if v == 1)
        print(f"\nalcanzables: {ok} de {len(mapa)}")
        for code, n in sorted(fallos.items()):
            print(f"   {n:3d} x {texto_error(code)}")

        # Distancia en planta a la base del brazo, para leer el mapa.
        print("\ndistancia horizontal desde x=0,y=0.135 (base del brazo):")
        for c in self.cols:
            ds = []
            for f in (self.filas[0], self.filas[-1]):
                p = self.pose_agarre(rack, c, f).position
                ds.append(math.hypot(p.x - 0.0, p.y - 0.135))
            print(f"   col{c}: de {min(ds)*1000:.0f} a {max(ds)*1000:.0f} mm")

    def puntual(self):
        rack, c, f = self.a.rack, self.a.columna, self.a.fila
        if (c, f) not in self.celdas:
            sys.exit(f"ERROR: ranura ({c},{f}) no existe. "
                     f"columnas {self.cols}, filas {self.filas}")
        p_agarre = self.pose_agarre(rack, c, f)
        p_pre = self.pose_agarre(rack, c, f, alto_extra=self.a.aproximacion)

        print(f"\n{rack}, columna {c}, fila {f}, modo {self.a.modo}")
        for et, p in (("preagarre", p_pre), ("agarre", p_agarre)):
            d = math.hypot(p.position.x, p.position.y - 0.135)
            print(f"   {et:10s} ({p.position.x:+.5f}, {p.position.y:+.5f}, "
                  f"{p.position.z:.5f})   {d*1000:.0f} mm en planta")
        print()

        for et, p in (("preagarre", p_pre), ("agarre", p_agarre)):
            code = self.hay_ik(p)
            print(f"   IK   {et:10s} {texto_error(code)}")
            if code != 1:
                print("        sin IK no tiene sentido pedir plan")
                continue
            code = self.hay_plan(p, et)
            print(f"   plan {et:10s} {texto_error(code)}")
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rack", default="rack_in_1")
    ap.add_argument("--columna", type=int, default=5)
    ap.add_argument("--fila", type=int, default=1)
    ap.add_argument("--modo", choices=sorted(ORIENTACIONES), default="caras")
    ap.add_argument("--barrido", action="store_true",
                    help="IK sobre las 90 ranuras en vez de plan a una")
    ap.add_argument("--bajo-canto", type=float, default=0.020,
                    help="agarre a esta distancia bajo el borde superior (m)")
    ap.add_argument("--aproximacion", type=float, default=0.060,
                    help="altura del preagarre sobre el agarre (m)")
    ap.add_argument("--tolerancia", type=float, default=0.005)
    ap.add_argument("--timeout-ik", type=float, default=0.1)
    ap.add_argument("--sin-colisiones", action="store_true",
                    help="IK sin comprobar colisiones: separa 'no llega el "
                         "brazo' de 'llega pero choca'")
    args = ap.parse_args([x for x in sys.argv[1:] if not x.startswith("--ros-args")])

    rclpy.init()
    try:
        n = Alcance(args)
        n.barrido() if args.barrido else n.puntual()
    except SystemExit as e:
        if e.code:
            print(e, file=sys.stderr)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
