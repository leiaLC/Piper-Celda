#!/usr/bin/env python3
"""
Paso 11 - Omica / PiperCelda
Regenera las cajas de colision de microscopio_msr a partir de la malla
nueva de FreeCAD, y reescribe el model.sdf.

Metodo (el mismo que documentaba el modelo v0.6, sobre geometria sana):
  1. Muestrea la superficie de la malla (mas barato que voxelizar 1M caras).
  2. Rasteriza la ocupacion en una rejilla de 4 mm.
  3. Rebana en franjas horizontales y separa componentes conexas en cada
     franja: base, columna y cabezal salen como solidos distintos.
  4. Fusiona aglomerativamente hasta N cajas, eligiendo cada vez la union
     que menos volumen vacio agrega.

Es conservador: toda celda ocupada cae dentro de alguna caja.

    python3 11_regenerar_colisiones.py
    python3 11_regenerar_colisiones.py --cajas 20 --paso 3
    python3 11_regenerar_colisiones.py --revertir
"""
import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
MODELO = RAIZ / "src/piper_celda_gazebo/models/microscopio_msr"
SDF = MODELO / "model.sdf"
MALLA = MODELO / "meshes/microscopio_visual.stl"
BAK = SDF.with_suffix(".sdf.antescolisiones")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cajas", type=int, default=16)
    ap.add_argument("--paso", type=float, default=4.0, help="rejilla en mm")
    ap.add_argument("--franjas", type=int, default=24)
    ap.add_argument("--margen", type=float, default=3.0, help="holgura en mm")
    ap.add_argument("--revertir", action="store_true")
    a = ap.parse_args()

    if a.revertir:
        if BAK.exists():
            shutil.copy2(BAK, SDF)
            print(f"revertido desde {BAK.name}")
        else:
            print("no hay respaldo")
        return

    for f in (SDF, MALLA):
        if not f.exists():
            sys.exit(f"ERROR: no existe {f}")

    import numpy as np
    import trimesh
    from scipy import ndimage

    m = trimesh.load(MALLA, file_type="stl", force="mesh")
    lo_m, hi_m = m.bounds
    print(f"malla: {len(m.faces)} caras")
    print(f"   X {hi_m[0]-lo_m[0]:7.2f}  Y {hi_m[1]-lo_m[1]:7.2f}  "
          f"Z {hi_m[2]-lo_m[2]:7.2f} mm")

    print("\nmuestreando superficie...")
    pts, _ = trimesh.sample.sample_surface(m, 1_500_000)
    pts = np.vstack([pts, m.vertices])

    P = a.paso
    lo = pts.min(axis=0)
    g = np.floor((pts - lo) / P).astype(np.int32)
    nx, ny, nz = g.max(axis=0) + 1
    print(f"rejilla {nx} x {ny} x {nz}  (paso {P} mm)")

    bordes = np.linspace(0, nz, a.franjas + 1).astype(int)
    cajas = []
    for s in range(a.franjas):
        sel = (g[:, 2] >= bordes[s]) & (g[:, 2] < bordes[s + 1])
        if sel.sum() < 5:
            continue
        sub = g[sel]
        grid = np.zeros((nx, ny), bool)
        grid[sub[:, 0], sub[:, 1]] = True
        grid = ndimage.binary_closing(grid, np.ones((3, 3)))
        lab, n = ndimage.label(grid, structure=np.ones((3, 3)))
        for k in range(1, n + 1):
            xs, ys = np.where(lab == k)
            if len(xs) < 4:
                continue
            cajas.append([xs.min(), ys.min(), bordes[s],
                          xs.max() + 1, ys.max() + 1, bordes[s + 1]])

    c = np.array(cajas, float)
    V = lambda b: float(np.prod(b[3:6] - b[0:3]))
    print(f"\ncajas iniciales: {len(c)}, volumen {sum(V(b) for b in c)*P**3/1e6:.1f} L")

    def union(x, y):
        return np.concatenate([np.minimum(x[:3], y[:3]), np.maximum(x[3:], y[3:])])

    while len(c) > a.cajas:
        mejor = None
        for i in range(len(c)):
            for j in range(i + 1, len(c)):
                f = union(c[i], c[j])
                coste = V(f) - V(c[i]) - V(c[j])
                if mejor is None or coste < mejor[0]:
                    mejor = (coste, i, j, f)
        _, i, j, f = mejor
        c = np.vstack([np.delete(c, [i, j], axis=0), f])

    vol = sum(V(b) for b in c) * P ** 3 / 1e6
    print(f"cajas finales  : {len(c)}, volumen {vol:.1f} L")

    # ---------- reescribir el SDF ----------
    shutil.copy2(SDF, BAK) if not BAK.exists() else None
    print(f"\nrespaldo: {BAK.name}")

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    arbol = ET.parse(SDF, parser=parser)
    raiz = arbol.getroot()
    link = raiz.find(".//link")
    if link is None:
        sys.exit("ERROR: el SDF no tiene <link>")

    viejas = [e for e in link if e.tag == "collision"]
    print(f"colisiones anteriores eliminadas: {len(viejas)}")
    for e in viejas:
        link.remove(e)

    orden = sorted(c, key=lambda b: b[2])
    for k, b in enumerate(orden, 1):
        mn = lo + b[:3] * P - a.margen
        mx = lo + b[3:] * P + a.margen
        ce = (mn + mx) / 2 / 1000.0
        sz = (mx - mn) / 1000.0
        col = ET.SubElement(link, "collision", {"name": f"col_{k:02d}"})
        ET.SubElement(col, "pose").text = \
            f"{ce[0]:.4f} {ce[1]:.4f} {ce[2]:.4f} 0 0 0"
        geo = ET.SubElement(col, "geometry")
        box = ET.SubElement(geo, "box")
        ET.SubElement(box, "size").text = f"{sz[0]:.4f} {sz[1]:.4f} {sz[2]:.4f}"

    def indentar(e, n=0):
        pad = "\n" + "  " * n
        if len(e):
            if not e.text or not e.text.strip():
                e.text = pad + "  "
            for h in e:
                indentar(h, n + 1)
            if not h.tail or not h.tail.strip():
                h.tail = pad
        if n and (not e.tail or not e.tail.strip()):
            e.tail = pad

    indentar(raiz)
    arbol.write(SDF, encoding="utf-8", xml_declaration=True)

    print(f"escrito {SDF}  ({len(orden)} cajas nuevas, +{a.margen} mm de margen)")
    print("\nRecompila y relanza:")
    print("   colcon build --symlink-install   (desde la raiz del proyecto)")
    print("   source install/setup.bash")
    print("   ros2 launch piper_celda_gazebo celda_piper.launch.py")
    print("\nEn Gazebo activa View -> Collisions para verificar que las cajas")
    print("envuelven la malla sin dejar nada fuera.")
    print("\nRevertir:  python3 11_regenerar_colisiones.py --revertir")


if __name__ == "__main__":
    main()
