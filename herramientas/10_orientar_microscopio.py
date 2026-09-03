#!/usr/bin/env python3
"""
Paso 10 - Omica / PiperCelda
Toma el STL reexportado desde FreeCAD y lo deja listo para el model.sdf:

  1. Quita caras exactamente repetidas (el export trae ~1600 de 998000).
  2. Lo endereza: FreeCAD lo saca con el eje vertical en +Y. Se aplica una
     rotacion de +90 grados sobre X para que la vertical sea +Z.
  3. Baja la base al plano z = 0.
  4. Centra el origen en la huella de la placa base, que es lo que espera
     el model.sdf ("centro de la huella, en el plano de apoyo").

    python3 10_orientar_microscopio.py ~/Downloads/model.stl
"""
import shutil
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
DESTINO = (RAIZ /
           "src/piper_celda_gazebo/models/microscopio_msr"
           "/meshes/microscopio_visual.stl")

VIEJO = {"ancho": 384.27, "fondo": 537.96, "alto": 425.99}


def main():
    if len(sys.argv) < 2:
        sys.exit("uso: python3 10_orientar_microscopio.py RUTA_AL_model.stl")
    origen = Path(sys.argv[1]).expanduser()
    if not origen.exists():
        sys.exit(f"ERROR: no existe {origen}")

    import numpy as np
    import trimesh

    m = trimesh.load(origen, file_type="stl", force="mesh")
    n0 = len(m.faces)
    print(f"entrada: {n0} caras, {len(m.vertices)} vertices")
    lo, hi = m.bounds
    print(f"   X {hi[0]-lo[0]:7.2f}   Y {hi[1]-lo[1]:7.2f}   Z {hi[2]-lo[2]:7.2f} mm")

    # 1. caras repetidas
    m.merge_vertices()
    k = np.sort(m.faces, axis=1)
    _, pri = np.unique(k, axis=0, return_index=True)
    rep = len(m.faces) - len(pri)
    m.update_faces(np.isin(np.arange(len(m.faces)), pri))
    m.remove_unreferenced_vertices()
    print(f"\ncaras repetidas eliminadas: {rep}")

    # 2. enderezar: +Y pasa a ser +Z
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))

    # 3. base al plano z=0
    m.apply_translation([0, 0, -m.bounds[0][2]])

    # 4. centrar en la huella de la placa base
    base = m.vertices[m.vertices[:, 2] < 20]
    cx = (base[:, 0].min() + base[:, 0].max()) / 2
    cy = (base[:, 1].min() + base[:, 1].max()) / 2
    m.apply_translation([-cx, -cy, 0])

    lo, hi = m.bounds
    base = m.vertices[m.vertices[:, 2] < 20]
    print("\n" + "=" * 62)
    print("RESULTADO")
    print("=" * 62)
    print(f"   ancho  X : {hi[0]-lo[0]:7.2f} mm   ({lo[0]:+8.2f} a {hi[0]:+8.2f})")
    print(f"   fondo  Y : {hi[1]-lo[1]:7.2f} mm   ({lo[1]:+8.2f} a {hi[1]:+8.2f})")
    print(f"   alto   Z : {hi[2]-lo[2]:7.2f} mm   ({lo[2]:+8.2f} a {hi[2]:+8.2f})")
    print(f"   placa base: {base[:,0].max()-base[:,0].min():.1f} x "
          f"{base[:,1].max()-base[:,1].min():.1f} mm")

    print("\n   contra el modelo anterior:")
    for nom, val in (("ancho", hi[0]-lo[0]), ("fondo", hi[1]-lo[1]), ("alto", hi[2]-lo[2])):
        d = val - VIEJO[nom]
        marca = "  <-- CAMBIA" if abs(d) > 1 else ""
        print(f"      {nom:6s} {VIEJO[nom]:7.2f} -> {val:7.2f}   ({d:+7.2f} mm){marca}")

    if DESTINO.exists():
        bak = DESTINO.with_suffix(".stl.previo_freecad")
        if not bak.exists():
            shutil.copy2(DESTINO, bak)
            print(f"\n   respaldo del anterior: {bak.name}")

    DESTINO.parent.mkdir(parents=True, exist_ok=True)
    m.export(DESTINO)
    print(f"\n   escrito {DESTINO}")
    print(f"   {n0} -> {len(m.faces)} caras")

    print("\n" + "=" * 62)
    print("PENDIENTE")
    print("=" * 62)
    print("Las 16 cajas de colision del model.sdf se calcularon sobre la malla")
    print("vieja, que era mas ancha y tenia geometria duplicada. Ya no")
    print("corresponden. El visual se vera bien pero MoveIt planificara contra")
    print("un obstaculo equivocado. Hay que regenerarlas.")
    print("\nRevisa tambien el <scale> del <visual> en model.sdf: la malla esta")
    print("en milimetros y necesita 0.001 0.001 0.001.")


if __name__ == "__main__":
    main()
