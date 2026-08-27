#!/usr/bin/env python3
"""
Publica la geometria del mundo de Gazebo en la escena de planificacion de
MoveIt, para que el planificador conozca la mesa, los microscopios, los
escurridores y las paredes.

Lee el mismo celda_piper.sdf que carga Gazebo, resuelve los model:// contra
la carpeta models/, y compone las poses (modelo -> link -> colision).
Asi la escena y la simulacion no pueden desincronizarse: son el mismo archivo.

Soporta box, cylinder, sphere y mesh. El lector de STL binario va incluido
para no depender de trimesh ni pyassimp en tiempo de ejecucion.

    ros2 run piper_celda_moveit publicar_escena_celda.py
    ros2 run piper_celda_moveit publicar_escena_celda.py --ros-args -p seguir:=true

Con seguir:=true el nodo no termina: se queda escuchando las poses que
publica Gazebo y reenvia a MoveIt cualquier modelo que muevas en la GUI.
Asi lo que colocas en Gazebo es lo que ve el planificador, sin tocar nada
dos veces. Requiere que el puente de poses este activo (lo lanza
celda_piper.launch.py).
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive


# ----------------------------------------------------------------- geometria
def leer_pose(el):
    """<pose>x y z r p y</pose> -> (xyz, rpy). Ausente = identidad."""
    if el is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    p = el.find("pose")
    if p is None or not (p.text or "").strip():
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    v = [float(x) for x in p.text.split()]
    v += [0.0] * (6 - len(v))
    return tuple(v[:3]), tuple(v[3:6])


def mat_rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def aplica(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def a_matriz(r):
    """Acepta rpy (3 numeros) o una matriz 3x3 ya construida."""
    if len(r) == 3 and isinstance(r[0], (list, tuple)):
        return [list(f) for f in r]
    return mat_rpy(*r)


def componer(pose_a, pose_b):
    """a (+) b: b expresado en el marco de a. Encadenable."""
    (ta, ra), (tb, rb) = pose_a, pose_b
    Ra = a_matriz(ra)
    t = tuple(ta[i] + aplica(Ra, tb)[i] for i in range(3))
    return t, mul(Ra, a_matriz(rb))


def a_cuaternion(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2][1] - R[1][2]) / s
        y, z = (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        w, x = (R[2][1] - R[1][2]) / s, 0.25 * s
        y, z = (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        w, x = (R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s
        y, z = 0.25 * s, (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        w, x = (R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s
        y, z = (R[1][2] + R[2][1]) / s, 0.25 * s
    return x, y, z, w


def cuaternion_a_matriz(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ]


def pose_msg(t, R):
    m = Pose()
    m.position.x, m.position.y, m.position.z = t
    q = a_cuaternion(R)
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
    return m


# ----------------------------------------------------------------- lector stl
def leer_stl(ruta, escala):
    """Devuelve shape_msgs/Mesh. Acepta STL binario y ASCII."""
    with open(ruta, "rb") as f:
        datos = f.read()

    tris = []
    if len(datos) > 84 and not datos[:5].lstrip().startswith(b"solid"):
        n = struct.unpack("<I", datos[80:84])[0]
        if 84 + n * 50 <= len(datos):
            for i in range(n):
                o = 84 + i * 50 + 12
                tris.append([struct.unpack("<3f", datos[o + j * 12:o + 12 + j * 12])
                             for j in range(3)])
    if not tris:  # ASCII
        act = []
        for linea in datos.decode("utf-8", "ignore").splitlines():
            p = linea.split()
            if len(p) == 4 and p[0] == "vertex":
                act.append(tuple(float(x) for x in p[1:4]))
                if len(act) == 3:
                    tris.append(act)
                    act = []

    verts, idx, caras = [], {}, []
    for t in tris:
        tri = []
        for v in t:
            k = (round(v[0], 6), round(v[1], 6), round(v[2], 6))
            if k not in idx:
                idx[k] = len(verts)
                verts.append((v[0] * escala[0], v[1] * escala[1], v[2] * escala[2]))
            tri.append(idx[k])
        if len(set(tri)) == 3:
            caras.append(tri)

    m = Mesh()
    m.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in verts]
    m.triangles = [MeshTriangle(vertex_indices=c) for c in caras]
    return m


# ----------------------------------------------------------------- nodo
class Escena(Node):
    def __init__(self):
        super().__init__("publicar_escena_celda")
        self.declare_parameter("mundo", "celda_piper.sdf")
        self.declare_parameter("marco", "world")
        self.declare_parameter("excluir", ["ground_plane"])
        self.declare_parameter("seguir", False)
        self.declare_parameter("topico_poses", "/poses_gazebo")
        self.declare_parameter("umbral", 0.001)      # m, para no reenviar ruido
        self.declare_parameter("periodo", 0.5)       # s entre actualizaciones

        pkg_gz = get_package_share_directory("piper_celda_gazebo")
        self.raiz_modelos = os.path.join(pkg_gz, "models")
        mundo = os.path.join(pkg_gz, "worlds",
                             self.get_parameter("mundo").value)
        self.marco = self.get_parameter("marco").value
        self.excluir = set(self.get_parameter("excluir").value)

        if not os.path.exists(mundo):
            self.get_logger().error(f"no existe {mundo}")
            raise SystemExit(1)

        self.objetos = self.recorrer(mundo)
        self.get_logger().info(f"{len(self.objetos)} objetos preparados")
        self.enviar(self.objetos)

        if self.get_parameter("seguir").value:
            self.umbral = self.get_parameter("umbral").value
            self.pendientes = {}
            self.create_subscription(
                TFMessage, self.get_parameter("topico_poses").value,
                self.poses_gazebo, 10)
            self.create_timer(self.get_parameter("periodo").value, self.refrescar)
            self.get_logger().info(
                "modo seguimiento activo: mueve modelos en Gazebo y la "
                "escena de MoveIt se actualizara sola")
            self.terminar = False
        else:
            self.terminar = True

    # ---------- lectura del sdf ----------
    def geometrias_de_modelo(self, modelo_el, pose_mundo, carpeta):
        """Devuelve [(primitiva|malla, pose)] de todas las <collision>."""
        salida = []
        for link in modelo_el.findall("link"):
            pose_link = componer(pose_mundo, leer_pose(link))
            for col in link.findall("collision"):
                t_c, r_c = leer_pose(col)
                Rl = a_matriz(pose_link[1])
                t = tuple(pose_link[0][i] + aplica(Rl, t_c)[i] for i in range(3))
                R = mul(Rl, a_matriz(r_c))
                geo = col.find("geometry")
                if geo is None:
                    continue
                g = self.una_geometria(geo, carpeta)
                if g is not None:
                    salida.append((g, (t, R)))
        return salida

    def una_geometria(self, geo, carpeta):
        caja = geo.find("box/size")
        if caja is not None:
            s = [float(x) for x in caja.text.split()]
            p = SolidPrimitive(); p.type = SolidPrimitive.BOX; p.dimensions = s
            return p
        cil = geo.find("cylinder")
        if cil is not None:
            p = SolidPrimitive(); p.type = SolidPrimitive.CYLINDER
            p.dimensions = [float(cil.find("length").text),
                            float(cil.find("radius").text)]
            return p
        esf = geo.find("sphere/radius")
        if esf is not None:
            p = SolidPrimitive(); p.type = SolidPrimitive.SPHERE
            p.dimensions = [float(esf.text)]
            return p
        malla = geo.find("mesh")
        if malla is not None:
            uri = malla.find("uri").text.strip()
            esc = malla.find("scale")
            e = [float(x) for x in esc.text.split()] if esc is not None else [1, 1, 1]
            ruta = self.resolver(uri, carpeta)
            if ruta and os.path.exists(ruta):
                try:
                    return leer_stl(ruta, e)
                except Exception as ex:
                    self.get_logger().warn(f"no pude leer {ruta}: {ex}")
            else:
                self.get_logger().warn(f"malla no encontrada: {uri}")
        return None

    def resolver(self, uri, carpeta):
        if uri.startswith("model://"):
            return os.path.join(self.raiz_modelos, uri[len("model://"):])
        if uri.startswith("file://"):
            return uri[len("file://"):]
        return os.path.join(carpeta, uri)

    def recorrer(self, mundo):
        raiz = ET.parse(mundo).getroot()
        w = raiz.find("world")
        objetos = []
        # geometria expresada en el marco del modelo, y pose del modelo en el
        # mundo. Separarlas permite reubicar el objeto entero cuando Gazebo
        # avisa de que lo has movido, sin volver a leer mallas.
        self.geom_relativa = {}
        self.poses_base = {}
        self.poses_vivas = {}
        ident = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        for el in w:
            if el.tag == "model":
                nombre = el.get("name", "modelo")
                if nombre in self.excluir:
                    continue
                pose = leer_pose(el)
                rel = self.geometrias_de_modelo(el, ident, os.path.dirname(mundo))
                if rel:
                    self.geom_relativa[nombre] = rel
                    self.poses_base[nombre] = pose
                    self.poses_vivas[nombre] = (pose[0], a_matriz(pose[1]))
                    objetos.append((nombre,
                                    [(g, componer(pose, r)) for g, r in rel]))

            elif el.tag == "include":
                uri = el.find("uri")
                if uri is None:
                    continue
                carpeta = self.resolver(uri.text.strip(), os.path.dirname(mundo))
                sdf = os.path.join(carpeta, "model.sdf")
                if not os.path.exists(sdf):
                    self.get_logger().warn(f"sin model.sdf en {carpeta}")
                    continue
                n_el = el.find("name")
                nombre = n_el.text.strip() if n_el is not None else \
                    os.path.basename(carpeta)
                if nombre in self.excluir:
                    continue
                sub = ET.parse(sdf).getroot().find("model")
                if sub is None:
                    continue
                # pose del include (+) pose interna del modelo
                pose = componer(leer_pose(el), leer_pose(sub))
                rel = self.geometrias_de_modelo(sub, ident, carpeta)
                if rel:
                    self.geom_relativa[nombre] = rel
                    self.poses_base[nombre] = pose
                    self.poses_vivas[nombre] = (pose[0], a_matriz(pose[1]))
                    objetos.append((nombre,
                                    [(g, componer(pose, r)) for g, r in rel]))
                    self.get_logger().info(f"   {nombre:24s} {len(rel)} colisiones")
        return objetos

    # ---------- seguimiento de poses ----------
    def poses_gazebo(self, msg):
        """Gazebo publica la pose de cada entidad respecto de su padre.
        Solo interesan los modelos de primer nivel (padre = el mundo)."""
        for t in msg.transforms:
            nombre = t.child_frame_id
            if nombre not in self.poses_base:
                continue
            tr = t.transform.translation
            r = t.transform.rotation
            nueva = ((tr.x, tr.y, tr.z), cuaternion_a_matriz(r.x, r.y, r.z, r.w))
            vieja = self.poses_vivas.get(nombre)
            if vieja is not None and \
               max(abs(nueva[0][i] - vieja[0][i]) for i in range(3)) < self.umbral and \
               max(abs(nueva[1][i][j] - vieja[1][i][j])
                   for i in range(3) for j in range(3)) < 0.002:
                continue
            self.poses_vivas[nombre] = nueva
            self.pendientes[nombre] = nueva

    def refrescar(self):
        if not self.pendientes:
            return
        cambiados = dict(self.pendientes)
        self.pendientes.clear()

        escena = PlanningScene()
        escena.is_diff = True
        for nombre, pose in cambiados.items():
            co = CollisionObject()
            co.header.frame_id = self.marco
            co.id = nombre
            co.operation = CollisionObject.ADD
            for g, rel in self.geom_relativa[nombre]:
                t, R = componer(pose, rel)
                if isinstance(g, SolidPrimitive):
                    co.primitives.append(g)
                    co.primitive_poses.append(pose_msg(t, R))
                else:
                    co.meshes.append(g)
                    co.mesh_poses.append(pose_msg(t, R))
            escena.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request()
        req.scene = escena
        self.cli.call_async(req)
        self.get_logger().info(
            f"actualizados: {', '.join(sorted(cambiados))}")

    # ---------- envio ----------
    def enviar(self, objetos):
        escena = PlanningScene()
        escena.is_diff = True

        for nombre, gs in objetos:
            co = CollisionObject()
            co.header.frame_id = self.marco
            co.id = nombre
            co.operation = CollisionObject.ADD
            for g, (t, R) in gs:
                if isinstance(g, SolidPrimitive):
                    co.primitives.append(g)
                    co.primitive_poses.append(pose_msg(t, R))
                else:
                    co.meshes.append(g)
                    co.mesh_poses.append(pose_msg(t, R))
            escena.world.collision_objects.append(co)

        cli = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.cli = cli
        if not cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                "apply_planning_scene no responde. "
                "Lanza primero moveit.launch.py")
            raise SystemExit(1)

        req = ApplyPlanningScene.Request()
        req.scene = escena
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)

        if fut.result() is not None and fut.result().success:
            n_p = sum(len(c.primitives) for c in escena.world.collision_objects)
            n_m = sum(len(c.meshes) for c in escena.world.collision_objects)
            self.get_logger().info(
                f"escena publicada: {len(escena.world.collision_objects)} objetos, "
                f"{n_p} primitivas, {n_m} mallas")
        else:
            self.get_logger().error("el servicio rechazo la escena")


def main():
    rclpy.init()
    try:
        nodo = Escena()
        if not nodo.terminar:
            rclpy.spin(nodo)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
