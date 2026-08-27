#!/usr/bin/env python3
"""
Maneja la laminilla dentro de la escena de planificacion.

El problema: MoveIt trata la laminilla como obstaculo rigido, asi que
rechaza cualquier trayectoria que la roce. Pero agarrarla ES tocarla.
Hay dos mecanismos para resolverlo, y se usan en momentos distintos:

  permitir  - Marca en la matriz de colisiones que los dedos PUEDEN tocar
              la laminilla. Se hace ANTES de planificar la aproximacion,
              o el planificador no encontrara ninguna solucion de agarre.

  adjuntar  - Mueve la laminilla del mundo a la lista de cuerpos adjuntos
              del robot. A partir de ahi MoveIt la trata como parte del
              brazo: la mueve con el, y comprueba SUS colisiones contra el
              resto de la celda. Se hace en el instante del cierre de pinza.

  soltar    - La devuelve al mundo, en la pose donde este en ese momento.

  prohibir  - Revierte 'permitir'.

Uso:
    ros2 run piper_celda_moveit manipular_laminilla.py permitir
    ros2 run piper_celda_moveit manipular_laminilla.py adjuntar
    ros2 run piper_celda_moveit manipular_laminilla.py soltar
    ros2 run piper_celda_moveit manipular_laminilla.py estado

    ... --objeto laminilla_02 --link Link7
"""
import argparse
import sys

import rclpy
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node

# Links que forman la pinza. Son los que pueden tocar la laminilla sin que
# eso cuente como colision.
PINZA = ["Link7", "Link8", "gripper_base", "Link6"]


class Manipular(Node):
    def __init__(self, args):
        super().__init__("manipular_laminilla")
        self.obj = args.objeto
        self.link = args.link
        self.pinza = args.touch_links or PINZA

        self.cli_get = self.create_client(GetPlanningScene, "get_planning_scene")
        self.cli_set = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        for c, n in ((self.cli_get, "get_planning_scene"),
                     (self.cli_set, "apply_planning_scene")):
            if not c.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"{n} no responde. Lanza MoveIt primero.")
                raise SystemExit(1)

        getattr(self, args.accion)()

    # ---------- utilidades ----------
    def escena_actual(self, componentes):
        req = GetPlanningScene.Request()
        req.components.components = componentes
        fut = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if fut.result() is None:
            self.get_logger().error("no pude leer la escena")
            raise SystemExit(1)
        return fut.result().scene

    def aplicar(self, escena, msg):
        fut = self.cli_set.call_async(ApplyPlanningScene.Request(scene=escena))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info(msg if ok else "el servicio rechazo el cambio")
        if not ok:
            raise SystemExit(1)

    def acm_set(self, acm, a, b, valor):
        """Marca el par (a,b) como permitido o prohibido, creando filas si hace falta."""
        for n in (a, b):
            if n not in acm.entry_names:
                acm.entry_names.append(n)
                for fila in acm.entry_values:
                    fila.enabled.append(False)
                nueva = AllowedCollisionEntry()
                nueva.enabled = [False] * len(acm.entry_names)
                acm.entry_values.append(nueva)
        i, j = acm.entry_names.index(a), acm.entry_names.index(b)
        acm.entry_values[i].enabled[j] = valor
        acm.entry_values[j].enabled[i] = valor

    # ---------- acciones ----------
    def permitir(self, valor=True):
        escena = self.escena_actual(
            GetPlanningScene.Request().components.ALLOWED_COLLISION_MATRIX)
        for link in self.pinza:
            self.acm_set(escena.allowed_collision_matrix, self.obj, link, valor)
        escena.is_diff = True
        escena.robot_state.is_diff = True
        verbo = "permitido" if valor else "prohibido"
        self.aplicar(escena,
                     f"contacto {verbo} entre {self.obj} y {', '.join(self.pinza)}")

    def prohibir(self):
        self.permitir(valor=False)

    def adjuntar(self):
        aco = AttachedCollisionObject()
        aco.link_name = self.link
        aco.object.id = self.obj
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = self.pinza

        escena = PlanningScene()
        escena.is_diff = True
        escena.robot_state.is_diff = True
        escena.robot_state.attached_collision_objects.append(aco)
        # El objeto sale del mundo: si no, quedaria duplicado y en colision
        # consigo mismo.
        quitar = CollisionObject()
        quitar.id = self.obj
        quitar.operation = CollisionObject.REMOVE
        escena.world.collision_objects.append(quitar)

        self.aplicar(escena, f"{self.obj} adjuntada a {self.link}")

    def soltar(self):
        aco = AttachedCollisionObject()
        aco.link_name = self.link
        aco.object.id = self.obj
        aco.object.operation = CollisionObject.REMOVE

        escena = PlanningScene()
        escena.is_diff = True
        escena.robot_state.is_diff = True
        escena.robot_state.attached_collision_objects.append(aco)
        self.aplicar(escena, f"{self.obj} soltada de {self.link}")
        self.get_logger().info(
            "vuelve a publicar la escena si quieres la laminilla "
            "en su pose original del mundo")

    def estado(self):
        c = GetPlanningScene.Request().components
        escena = self.escena_actual(
            c.WORLD_OBJECT_NAMES | c.ROBOT_STATE_ATTACHED_OBJECTS
            | c.ALLOWED_COLLISION_MATRIX)
        mundo = [o.id for o in escena.world.collision_objects]
        adj = [(a.object.id, a.link_name)
               for a in escena.robot_state.attached_collision_objects]
        print("\nobjetos en el mundo:")
        for o in mundo:
            print("   ", o)
        print("\nadjuntos al robot:")
        for o, l in adj or []:
            print(f"    {o}  ->  {l}")
        if not adj:
            print("    (ninguno)")

        acm = escena.allowed_collision_matrix
        if self.obj in acm.entry_names:
            i = acm.entry_names.index(self.obj)
            per = [acm.entry_names[j]
                   for j, v in enumerate(acm.entry_values[i].enabled) if v]
            print(f"\ncontacto permitido con {self.obj}:")
            print("   ", ", ".join(per) if per else "(nada)")
        else:
            print(f"\n{self.obj} no aparece en la matriz de colisiones")
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("accion",
                    choices=["permitir", "prohibir", "adjuntar", "soltar", "estado"])
    ap.add_argument("--objeto", default="laminilla_01")
    ap.add_argument("--link", default="Link7",
                    help="link al que se adjunta la laminilla")
    ap.add_argument("--touch-links", nargs="*", default=None,
                    help="links que pueden tocarla sin contar como colision")
    args = ap.parse_args([a for a in sys.argv[1:] if not a.startswith("--ros-args")])

    rclpy.init()
    try:
        Manipular(args)
    except SystemExit:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
