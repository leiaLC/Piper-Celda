#!/usr/bin/env python3
"""
Paso 14 - Omica / PiperCelda
Puebla un rack del mundo con laminillas DINAMICAS, o lo vacia.

POR QUE HACE FALTA UNA HERRAMIENTA
  rack_laminillas_lleno trae 90 laminillas, pero son <visual> dentro de un
  link estatico: decorado. Un link estatico no puede contener cuerpos
  dinamicos, asi que una laminilla agarrable tiene que ser un modelo
  aparte, incluido en el mundo con su propia pose.

  Escribir 90 <include> a mano es inviable y se desincroniza en cuanto
  mueves un rack. Esta herramienta los genera leyendo:
      poses de los racks .... <include> de worlds/celda_piper.sdf
      rejilla de ranuras .... <visual name="lam_C_F"> de
                              models/rack_laminillas_lleno/model.sdf
      cotas de la laminilla . models/laminilla/model.sdf
  Ninguna cota esta escrita aqui.

IDEMPOTENTE
  El bloque generado va entre dos marcadores. Volver a correrla reemplaza
  el bloque entero; no acumula. --vaciar lo borra.

QUE MAS TOCA
  El rack que se puebla pasa a usar model://rack_laminillas (vacio) en vez
  de rack_laminillas_lleno, o se verian las 90 visuales estaticas
  atravesadas con las 90 dinamicas. Al vaciar se restaura.

ORIENTACION
  El modelo laminilla tiene X = espesor, Y = ancho, Z = largo. En el rack
  el espesor va en Y del mundo, asi que cada include lleva yaw = pi/2.

COSTE
  Cada laminilla es un cuerpo dinamico de 5.15 g y 1.1 mm asentandose en
  una ranura de 2 mm. Noventa de golpe pueden hacer temblar ODE. Empieza
  por un rack, mira el RTF en la GUI de Gazebo, y sube desde ahi.
  --columnas y --filas permiten poblar solo una parte.

    python3 14_poblar_rack.py --rack rack_in_1
    python3 14_poblar_rack.py --rack rack_in_1 --columnas 5 --filas 1 2 3
    python3 14_poblar_rack.py --rack rack_in_1 --vaciar
"""
import argparse
import math
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

INICIO = "    <!-- ##### LAMINILLAS_DINAMICAS_INICIO (generado, no editar) ##### -->"
FIN = "    <!-- ##### LAMINILLAS_DINAMICAS_FIN ##### -->"


def leer_pose(el):
    if el is None:
        return [0.0] * 6
    p = el.find("pose")
    if p is None or not (p.text or "").strip():
        return [0.0] * 6
    return ([float(x) for x in p.text.split()] + [0.0] * 6)[:6]


def racks_del_mundo(ruta):
    raiz = ET.parse(ruta).getroot().find("world")
    out = {}
    for el in raiz.findall("include"):
        uri, nom = el.find("uri"), el.find("name")
        if uri is None or nom is None or "rack_laminillas" not in uri.text:
            continue
        p = leer_pose(el)
        out[nom.text.strip()] = (p[0], p[1], p[2], p[5])
    return out


def rejilla(ruta):
    """{(col,fila): (x,y,z_centro)} en el marco del rack, mas el semialto."""
    link = ET.parse(ruta).getroot().find("model").find("link")
    celdas, semialto = {}, None
    for vis in link.findall("visual"):
        nom = vis.get("name", "")
        if not nom.startswith("lam_"):
            continue
        _, c, f = nom.split("_")
        p = leer_pose(vis)
        celdas[(int(c), int(f))] = (p[0], p[1], p[2])
        if semialto is None:
            semialto = float(vis.find("geometry/box/size").text.split()[2]) / 2
    if not celdas:
        sys.exit(f"ERROR: no hay visuales lam_C_F en {ruta}")
    return celdas, semialto


def cotas_laminilla(ruta):
    link = ET.parse(ruta).getroot().find("model").find("link")
    caja = link.find("collision/geometry/box/size")
    return [float(v) for v in caja.text.split()]


def bloque(rack, pose_rack, celdas, cols, filas, etiqueta):
    rx, ry, rz, yaw = pose_rack
    c, s = math.cos(yaw), math.sin(yaw)
    out = [INICIO,
           f"    <!-- {len(cols)*len(filas)} laminillas dinamicas en {rack}."]
    out.append("         Generado por herramientas/14_poblar_rack.py.")
    out.append("         yaw = pi/2 pone el espesor del modelo (su eje X) sobre Y del")
    out.append("         mundo, que es como van las ranuras de este rack. -->")
    for col in cols:
        for fil in filas:
            lx, ly, lz = celdas[(col, fil)]
            x = rx + c * lx - s * ly
            y = ry + s * lx + c * ly
            z = rz + lz
            out.append("    <include>")
            out.append("      <uri>model://laminilla</uri>")
            out.append(f"      <name>{etiqueta}_c{col}_f{fil}</name>")
            out.append(f"      <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 1.5708</pose>")
            out.append("    </include>")
    out.append(FIN)
    return "\n".join(out)


def sustituir_bloque(txt, nuevo):
    if INICIO in txt and FIN in txt:
        i = txt.index(INICIO)
        j = txt.index(FIN) + len(FIN)
        return txt[:i] + (nuevo if nuevo else "") + txt[j:], True
    if not nuevo:
        return txt, False
    # primera vez: justo antes de cerrar el world
    m = re.search(r"\n(\s*)</world>", txt)
    if not m:
        sys.exit("ERROR: no encuentro </world> en el mundo")
    return txt[:m.start()] + "\n\n" + nuevo + txt[m.start():], True


def cambiar_modelo_rack(txt, rack, uri_nueva):
    """Cambia el <uri> del include cuyo <name> sea 'rack'."""
    patron = re.compile(
        r"(<include>\s*<uri>)model://rack_laminillas(?:_lleno)?(</uri>\s*"
        r"<name>" + re.escape(rack) + r"</name>)")
    nuevo, n = patron.subn(r"\g<1>" + uri_nueva + r"\g<2>", txt)
    return nuevo, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rack", default="rack_in_1")
    ap.add_argument("--mundo", default=None)
    ap.add_argument("--models", default=None)
    ap.add_argument("--columnas", type=int, nargs="*", default=None)
    ap.add_argument("--filas", type=int, nargs="*", default=None)
    ap.add_argument("--etiqueta", default=None,
                    help="prefijo del nombre. Por defecto 'lam_<rack sin rack_>'")
    ap.add_argument("--vaciar", action="store_true")
    a = ap.parse_args()

    raiz = Path(__file__).resolve().parents[1]
    base = raiz / "src/piper_celda_gazebo"
    mundo = Path(a.mundo) if a.mundo else base / "worlds/celda_piper.sdf"
    models = Path(a.models) if a.models else base / "models"
    if not mundo.exists():
        sys.exit(f"ERROR: no existe {mundo}")

    txt = mundo.read_text()

    if a.vaciar:
        txt, habia = sustituir_bloque(txt, "")
        txt, n = cambiar_modelo_rack(txt, a.rack, "model://rack_laminillas_lleno")
        mundo.write_text(txt)
        print("bloque de laminillas dinamicas: " +
              ("eliminado" if habia else "no habia"))
        print(f"{a.rack} vuelve a rack_laminillas_lleno ({n} include tocado)")
        return

    racks = racks_del_mundo(mundo)
    if a.rack not in racks:
        sys.exit(f"ERROR: '{a.rack}' no esta en el mundo. Hay: {sorted(racks)}")

    celdas, semialto = rejilla(models / "rack_laminillas_lleno/model.sdf")
    esp, ancho, largo = cotas_laminilla(models / "laminilla/model.sdf")

    if abs(largo / 2 - semialto) > 1e-6:
        print(f"AVISO: la rejilla se genero con laminillas de "
              f"{semialto*2000:.1f} mm y el modelo mide {largo*1000:.1f} mm. "
              f"Vuelve a correr 13_preparar_rack.py.")

    cols = sorted(a.columnas) if a.columnas else sorted({c for c, _ in celdas})
    filas = sorted(a.filas) if a.filas else sorted({f for _, f in celdas})
    faltan = [(c, f) for c in cols for f in filas if (c, f) not in celdas]
    if faltan:
        sys.exit(f"ERROR: ranuras inexistentes: {faltan[:5]}")

    etiqueta = a.etiqueta or "lam_" + a.rack.replace("rack_", "")
    nuevo = bloque(a.rack, racks[a.rack], celdas, cols, filas, etiqueta)
    txt, _ = sustituir_bloque(txt, nuevo)
    txt, n = cambiar_modelo_rack(txt, a.rack, "model://rack_laminillas")
    mundo.write_text(txt)

    total = len(cols) * len(filas)
    print(f"{total} laminillas dinamicas en {a.rack}")
    print(f"   columnas {cols}")
    print(f"   filas    {filas[0]}..{filas[-1]} ({len(filas)})")
    print(f"   laminilla {esp*1000:.1f} x {ancho*1000:.1f} x {largo*1000:.1f} mm, "
          f"nombres {etiqueta}_cC_fF")
    print(f"   {a.rack} pasa a model://rack_laminillas ({n} include tocado)")
    print(f"   escrito {mundo}")
    print("\nMoveIt las recoge solo: publicar_escena_celda.py lee este mismo")
    print("archivo. Van a ser " + str(total) + " objetos de colision mas.")


if __name__ == "__main__":
    main()
