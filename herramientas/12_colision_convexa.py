#!/usr/bin/env python3
"""
Paso 12 - Omica / PiperCelda
Genera la colision del microscopio por DESCOMPOSICION CONVEXA en vez de
cajas. Parte el solido en piezas convexas que siguen la forma real.

Cajas (paso 11)      : ~25 L de volumen prohibido
Cascos convexos      : ~16 L, con la misma cobertura

Cada pieza se guarda como STL en meshes/collision/ y se referencia desde
model.sdf con <mesh>. Gazebo maneja mallas convexas sin problema; lo que
no soporta bien es geometria concava, y por eso hay que descomponerla.

    python3 12_colision_convexa.py
    python3 12_colision_convexa.py --piezas 64 --umbral 0.03
    python3 12_colision_convexa.py --revertir

Tarda unos 2-4 minutos. Necesita:  pip install coacd fast-simplification
"""
import argparse
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

MODELO = Path.home() / "Omica/PiperCelda/src/piper_celda_gazebo/models/microscopio_msr"
SDF = MODELO / "model.sdf"
MALLA = MODELO / "meshes/microscopio_visual.stl"
DIRCOL = MODELO / "meshes/collision"
BAK = SDF.with_suffix(".sdf.antesconvexa")


def dep(mod, paq=None):
    try:
        __import__(mod)
    except ImportError:
        print(f"   instalando {paq or mod}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-q",
                               "--break-system-packages", paq or mod])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--piezas", type=int, default=48,
                    help="maximo de cascos convexos")
    ap.add_argument("--umbral", type=float, default=0.05,
                    help="concavidad tolerada; mas bajo = mas fiel y mas piezas")
    ap.add_argument("--simplificar", type=int, default=60000,
                    help="caras de entrada al descomponedor")
    ap.add_argument("--revertir", action="store_true")
    a = ap.parse_args()

    if a.revertir:
        if BAK.exists():
            shutil.copy2(BAK, SDF)
            if DIRCOL.exists():
                shutil.rmtree(DIRCOL)
            print(f"revertido desde {BAK.name}, borrado {DIRCOL.name}/")
        else:
            print("no hay respaldo")
        return

    for f in (SDF, MALLA):
        if not f.exists():
            sys.exit(f"ERROR: no existe {f}")

    dep("coacd")
    dep("fast_simplification", "fast-simplification")
    import coacd
    import fast_simplification as fs
    import numpy as np
    import trimesh

    m = trimesh.load(MALLA, file_type="stl", force="mesh")
    print(f"malla: {len(m.faces)} caras")

    if len(m.faces) > a.simplificar:
        v, f = fs.simplify(m.vertices.astype(np.float32),
                           m.faces.astype(np.int32),
                           target_count=a.simplificar)
        d = trimesh.Trimesh(vertices=v, faces=f)
        print(f"   simplificada a {len(d.faces)} caras para descomponer")
    else:
        d = m

    print(f"\ndescomponiendo (max {a.piezas} piezas, umbral {a.umbral})...")
    print("   esto tarda unos minutos")
    t = time.time()
    coacd.set_log_level("error")
    partes = coacd.run_coacd(
        coacd.Mesh(d.vertices, d.faces),
        threshold=a.umbral,
        max_convex_hull=a.piezas,
        preprocess_mode="auto",
        resolution=2000,
        mcts_nodes=20,
        mcts_iterations=100,
        mcts_max_depth=3,
    )
    print(f"   {len(partes)} piezas en {time.time()-t:.0f}s")

    if DIRCOL.exists():
        shutil.rmtree(DIRCOL)
    DIRCOL.mkdir(parents=True)

    vol = 0.0
    caras = 0
    nombres = []
    for i, (vv, ff) in enumerate(partes):
        p = trimesh.Trimesh(vertices=vv, faces=ff).convex_hull
        n = f"col_{i:02d}.stl"
        p.export(DIRCOL / n)
        nombres.append(n)
        vol += p.volume
        caras += len(p.faces)

    print(f"\n   volumen prohibido : {vol/1e6:.1f} L")
    print(f"   triangulos totales: {caras}")

    # ---------- reescribir el SDF ----------
    if not BAK.exists():
        shutil.copy2(SDF, BAK)
        print(f"   respaldo: {BAK.name}")

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    arbol = ET.parse(SDF, parser=parser)
    raiz = arbol.getroot()
    link = raiz.find(".//link")
    if link is None:
        sys.exit("ERROR: el SDF no tiene <link>")

    viejas = [e for e in link if e.tag == "collision"]
    for e in viejas:
        link.remove(e)
    print(f"   colisiones anteriores eliminadas: {len(viejas)}")

    for i, n in enumerate(nombres):
        col = ET.SubElement(link, "collision", {"name": f"conv_{i:02d}"})
        geo = ET.SubElement(col, "geometry")
        me = ET.SubElement(geo, "mesh")
        ET.SubElement(me, "uri").text = f"model://microscopio_msr/meshes/collision/{n}"
        ET.SubElement(me, "scale").text = "0.001 0.001 0.001"
        sur = ET.SubElement(col, "surface")
        con = ET.SubElement(sur, "contact")
        ET.SubElement(con, "collide_bitmask").text = "0x01"

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
    print(f"   escrito {SDF}")

    print("\nRecompila y relanza:")
    print("   cd ~/Omica/PiperCelda && colcon build --symlink-install")
    print("   source install/setup.bash")
    print("   ros2 launch piper_celda_gazebo celda_piper.launch.py")
    print("\nView -> Collisions para comprobar que envuelven la malla.")
    print("\nRevertir:  python3 12_colision_convexa.py --revertir")


if __name__ == "__main__":
    main()
