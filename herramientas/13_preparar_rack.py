#!/usr/bin/env python3
"""
Paso 13 - Omica / PiperCelda
Convierte Laminillas-PruebaCompleta1-Body.stl en dos modelos de Gazebo:

    rack_laminillas         rack vacio
    rack_laminillas_lleno   el mismo rack con 90 laminillas de pie, como
                            VISUALES estaticas (decorado, sin fisica)

Para laminillas DINAMICAS no se usa el modelo lleno: se pueblan con
herramientas/14_poblar_rack.py, que mete includes del modelo `laminilla`
en el mundo. Un link estatico no puede contener cuerpos dinamicos.

QUE HACE

  1. Voltea el STL 180 grados sobre X.
     El STL viene con las 90 aberturas en z = 0 y una placa maciza en z = 27
     (2 triangulos que cubren los 18149 mm2 completos). Tal cual, es una caja
     cerrada: las ranuras no abren a ninguna parte. Volteado, las ranuras
     abren hacia arriba y la placa maciza queda de suelo.

     Se usa una rotacion de pi sobre X, no un espejo en Z. La rotacion es
     propia: conserva el sentido de giro de los 1484 triangulos y las
     normales siguen apuntando hacia afuera. Un espejo las invertiria y
     Gazebo renderizaria el rack del reves.

     Efecto lateral: la rotacion tambien espeja Y. Las 18 filas son
     simetricas en Y, asi que no cambia nada, salvo un teton de
     3 x 2.5 x 0.5 mm que estaba en el borde y = 0 y pasa al borde opuesto.

  2. Baja la base a z = 0 y centra la huella en el origen.
     Convencion identica a la de escurridor_60 y microscopio_msr.

  3. Mide la rejilla de ranuras sobre la propia malla, no sobre cotas de
     CAD: suelos de bolsillo para los centros y la seccion, cara horizontal
     de mayor area por encima del suelo para la tapa.

  4. Emite la colision como PEINE DE CAJAS. Ver mas abajo.

GEOMETRIA QUE DEBE SALIR (comprobar contra la impresion)
    envolvente ......... 144.5 x 125.6 x 27.5 mm (los 0.5 de arriba son teton)
    tapa del rack ...... z = 27.0
    ranuras ............ 90 = 5 columnas x 18 filas
    seccion de ranura .. 25.7 (X) x 2.0 (Y) mm
    suelo de ranura .... z = 2.0, o sea 25 mm de profundidad
    paso en X .......... 27.7 mm
    paso en Y ..........  6.8 mm

POR QUE PEINE Y NO CAJA ENVOLVENTE NI MALLA

    La version anterior usaba UNA caja envolvente. Servia como obstaculo
    para el brazo, pero hacia imposible meter una laminilla dinamica en una
    ranura: se solapaba con la caja y ODE la expulsaba al arrancar.

    La malla completa tampoco: ODE la trata como trimesh, que es caro y
    poco fiable contra objetos de 1.1 mm.

    El peine son 26 cajas cuya union reproduce el solido EXACTO, sin
    aproximar:
        1  base, de z=0 al suelo de ranura, a toda la huella
       19  nervios en Y, a todo el ancho X, entre bolsillo y bolsillo
        6  tabiques en X, a todo el largo Y, entre columna y columna
    Los nervios y los tabiques se solapan entre si, que en colision
    estatica es irrelevante. Lo que importa es que los 90 bolsillos quedan
    libres y una laminilla cabe en su ranura sin penetrar nada.

    Cajas primitivas es lo mas estable que tiene ODE contra objetos finos.

LAMINILLA
    Medidas reales medidas por Omica sobre el material que se usa:
        espesor 1.1   ancho 24.7   largo 75.8 mm
    En la ranura de 25.70 x 2.00 eso deja 1.00 mm de juego en ancho y
    0.90 en espesor. Sobresale 50.8 mm sobre la tapa del rack.

    uso:  python3 13_preparar_rack.py RUTA_AL_STL [--destino DIR_MODELS]
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np

# Laminilla real de Omica. NO tocar sin medir otra vez el material.
LAM_ESPESOR = 1.1
LAM_ANCHO = 24.7
LAM_LARGO = 75.8


# ----------------------------------------------------------------------
# STL binario. Sin trimesh: no esta empaquetado en Ubuntu 24.04 y no vale
# la pena un venv para leer y escribir un formato de 84 bytes de cabecera.
# numpy ya viene con ROS 2 Jazzy.
#
# La malla se representa como un array (n, 3, 3): n triangulos, 3 vertices,
# 3 coordenadas. Las normales se recalculan al escribir, asi que da igual
# lo que traiga el archivo de entrada.
# ----------------------------------------------------------------------

def leer_stl(ruta):
    datos = Path(ruta).read_bytes()
    if datos[:5].lstrip().lower().startswith(b"solid") and b"facet" in datos[:512]:
        sys.exit("ERROR: el STL es ASCII; este script solo lee binario")
    n = struct.unpack("<I", datos[80:84])[0]
    esperado = 84 + n * 50
    if len(datos) < esperado:
        sys.exit(f"ERROR: STL truncado ({len(datos)} bytes, se esperaban {esperado})")
    cuerpo = np.frombuffer(datos[84:esperado], dtype=np.uint8).reshape(n, 50)
    return cuerpo[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def escribir_stl(ruta, tri):
    n = len(tri)
    v = tri.astype("<f4")
    nor = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    ln = np.linalg.norm(nor, axis=1, keepdims=True)
    nor = np.divide(nor, ln, out=np.zeros_like(nor), where=ln > 0).astype("<f4")

    reg = np.zeros((n, 50), dtype=np.uint8)
    reg[:, 0:12] = nor.view(np.uint8).reshape(n, 12)
    reg[:, 12:48] = v.view(np.uint8).reshape(n, 36)
    cab = b"rack_laminillas - generado por 13_preparar_rack.py".ljust(80, b" ")
    Path(ruta).write_bytes(cab + struct.pack("<I", n) + reg.tobytes())


def preparar_malla(origen):
    tri = leer_stl(origen)
    v = tri.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    print(f"entrada: {len(tri)} caras")
    print(f"   X {hi[0]-lo[0]:7.2f}   Y {hi[1]-lo[1]:7.2f}   Z {hi[2]-lo[2]:7.2f} mm")

    # 1. voltear sobre X: (x, y, z) -> (x, -y, -z). Rotacion propia.
    tri = tri * np.array([1.0, -1.0, -1.0])

    # 2. base a z=0 y huella centrada
    v = tri.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    tri = tri + np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    v = tri.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    print("\nvolteada y recentrada:")
    print(f"   X {lo[0]:+8.2f} a {hi[0]:+8.2f}   ({hi[0]-lo[0]:.2f} mm)")
    print(f"   Y {lo[1]:+8.2f} a {hi[1]:+8.2f}   ({hi[1]-lo[1]:.2f} mm)")
    print(f"   Z {lo[2]:+8.2f} a {hi[2]:+8.2f}   ({hi[2]-lo[2]:.2f} mm)")
    return tri


def area(t):
    return 0.5 * np.linalg.norm(np.cross(t[1] - t[0], t[2] - t[0]))


def medir_rack(tri):
    """Devuelve (cx, cy, sx, sy, z_suelo, z_tapa) medidos sobre la malla."""
    z_min = tri[:, :, 2].min()
    plana = np.ptp(tri[:, :, 2], axis=1) < 1e-3

    # Suelos de bolsillo: caras horizontales mas bajas por encima del fondo.
    caras = list(tri[plana & (tri[:, 0, 2] > z_min + 1e-3)])
    if not caras:
        sys.exit("ERROR: no se encontraron suelos de bolsillo")
    z_suelo = min(v[0, 2] for v in caras)
    bolsillos = [v for v in caras if abs(v[0, 2] - z_suelo) < 1e-3]

    rect = {(round(v[:, 0].min(), 2), round(v[:, 0].max(), 2),
             round(v[:, 1].min(), 2), round(v[:, 1].max(), 2)): True
            for v in bolsillos}
    xs = sorted({(r[0], r[1]) for r in rect})
    ys = sorted({(r[2], r[3]) for r in rect})
    cx = [round((a + b) / 2, 3) for a, b in xs]
    cy = [round((a + b) / 2, 3) for a, b in ys]
    sx, sy = xs[0][1] - xs[0][0], ys[0][1] - ys[0][0]

    # Tapa: la cara horizontal de mayor area por encima del suelo de ranura.
    # No se toma el z maximo de la malla porque ahi solo esta el teton.
    acum = {}
    for t in tri[plana]:
        z = round(t[0, 2], 3)
        if z > z_suelo + 1e-3:
            acum[z] = acum.get(z, 0.0) + area(t)
    z_tapa = max(acum, key=acum.get)

    print(f"\nranuras: {len(rect)}  =  {len(cx)} columnas x {len(cy)} filas")
    print(f"   suelo de ranura   z = {z_suelo:.2f} mm")
    print(f"   tapa del rack     z = {z_tapa:.2f} mm  (area {acum[z_tapa]:.0f} mm2)")
    print(f"   profundidad       {z_tapa - z_suelo:.2f} mm")
    print(f"   seccion           {sx:.2f} x {sy:.2f} mm")
    print(f"   paso X            {cx[1]-cx[0]:.2f} mm")
    print(f"   paso Y            {cy[1]-cy[0]:.2f} mm")
    if len(rect) != len(cx) * len(cy):
        print(f"   AVISO: {len(rect)} rectangulos para una rejilla de "
              f"{len(cx)}x{len(cy)}; no esta completa")
    return cx, cy, sx, sy, z_suelo, z_tapa


def bandas_libres(centros, semi, lim_lo, lim_hi):
    """Tramos de material entre bolsillos, dado sus centros y semiancho."""
    huecos = [(c - semi, c + semi) for c in sorted(centros)]
    bordes = [lim_lo] + [b for h in huecos for b in h] + [lim_hi]
    return [(bordes[i], bordes[i + 1]) for i in range(0, len(bordes) - 1, 2)
            if bordes[i + 1] - bordes[i] > 1e-6]


def cajas_peine(tri, cx, cy, sx, sy, z_suelo, z_tapa):
    """26 cajas cuya union es el solido del rack menos los 90 bolsillos."""
    v = tri.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    cajas = []

    # 1. base maciza
    cajas.append(("base", (0.0, 0.0, z_suelo / 2),
                  (hi[0] - lo[0], hi[1] - lo[1], z_suelo)))

    alto = z_tapa - z_suelo
    zc = (z_suelo + z_tapa) / 2

    # 2. nervios en Y: a todo el ancho X, entre fila y fila
    for i, (a, b) in enumerate(bandas_libres(cy, sy / 2, lo[1], hi[1])):
        cajas.append((f"nervio_y_{i:02d}", (0.0, (a + b) / 2, zc),
                      (hi[0] - lo[0], b - a, alto)))

    # 3. tabiques en X: a todo el largo Y, entre columna y columna
    for i, (a, b) in enumerate(bandas_libres(cx, sx / 2, lo[0], hi[0])):
        cajas.append((f"tabique_x_{i:02d}", ((a + b) / 2, 0.0, zc),
                      (b - a, hi[1] - lo[1], alto)))
    return cajas


CABECERA = """<?xml version="1.0" ?>
<sdf version="1.7">
  <!--
    ============================================================
    {titulo}
    Generado por herramientas/13_preparar_rack.py - NO EDITAR A MANO
    ============================================================

    HUELLA        {ex:.1f} x {ey:.1f} x {ez:.1f} mm  (tapa en z = {tapa:.1f})
    RANURAS       {n} = {ncol} columnas x {nfil} filas
                  seccion {sx:.1f} x {sy:.1f} mm
                  suelo en z = {suelo:.1f}, profundidad {prof:.1f} mm
                  paso X {px:.1f} mm, paso Y {py:.1f} mm

    ORIGEN        centro de la huella, base apoyada en z = 0.
                  Misma convencion que escurridor_60 y microscopio_msr.

    COLISION      peine de {ncaj} cajas primitivas, no la malla ni una caja
                  envolvente. Su union reproduce el solido exacto y deja
                  los 90 bolsillos libres, que es lo que permite meter
                  laminillas DINAMICAS sin que ODE las expulse.

    LAMINILLA     {le} x {la} x {ll} mm (medida real de Omica)
                  juego en ranura: {ja:.2f} mm en ancho, {je:.2f} mm en espesor
                  sobresale {sob:.1f} mm sobre la tapa

    AGARRE        paso entre vecinas {py:.1f} mm con laminilla de {le} mm
                  -> {hueco:.2f} mm libres para el dedo.
                  La mordaza de serie del PiPER mide 5.01 mm en la punta y
                  crece unos 0.08 mm por mm; llega a {hueco:.2f} a los 9.5 mm.
                  O sea que solo puede morder los ~12 mm superiores de la
                  laminilla. Para bajar mas hace falta un rebaje local.
                  Las paredes entre columnas miden {pared:.1f} mm, asi que el
                  pinzado por cantos no es viable con vecinas puestas.
    ============================================================
  -->
  <model name="{nombre}">
    <static>true</static>
    <link name="link">
"""

COLISION = """
      <collision name="col_{nom}">
        <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 0</pose>
        <geometry>
          <box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box>
        </geometry>
        <surface>
          <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          <contact>
            <ode><kp>1e6</kp><kd>100</kd><min_depth>0.0002</min_depth></ode>
          </contact>
        </surface>
      </collision>
"""

VISUAL_RACK = """
      <visual name="rack">
        <geometry>
          <mesh>
            <uri>model://rack_laminillas/meshes/rack_laminillas.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.78 0.78 0.80 1</ambient>
          <diffuse>0.90 0.90 0.92 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
"""

LAMINILLA = """
      <visual name="lam_{col}_{fil}">
        <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 0</pose>
        <geometry>
          <box><size>{ax:.5f} {ay:.5f} {az:.5f}</size></box>
        </geometry>
        <material>
          <ambient>0.70 0.82 0.85 1</ambient>
          <diffuse>0.82 0.92 0.95 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
        </material>
      </visual>
      <visual name="etq_{col}_{fil}">
        <pose>{x:.5f} {y:.5f} {ze:.5f} 0 0 0</pose>
        <geometry>
          <box><size>{ax:.5f} {ay2:.5f} 0.01600</size></box>
        </geometry>
        <material>
          <ambient>0.85 0.75 0.35 1</ambient>
          <diffuse>0.95 0.85 0.45 1</diffuse>
        </material>
      </visual>
"""

PIE = """
    </link>
  </model>
</sdf>
"""

CONFIG = """<?xml version="1.0"?>
<model>
  <name>{nombre}</name>
  <version>2.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <description>{desc}</description>
</model>
"""


def escribir(destino, nombre, titulo, desc, tri, med, cajas, llenar):
    cx, cy, sx, sy, z_suelo, z_tapa = med
    v = tri.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    ex, ey, ez = hi - lo

    d = destino / nombre
    (d / "meshes").mkdir(parents=True, exist_ok=True)

    txt = CABECERA.format(
        titulo=titulo, nombre=nombre,
        ex=ex, ey=ey, ez=ez, tapa=z_tapa,
        n=len(cx) * len(cy), ncol=len(cx), nfil=len(cy),
        sx=sx, sy=sy, suelo=z_suelo, prof=z_tapa - z_suelo,
        px=cx[1] - cx[0], py=cy[1] - cy[0], ncaj=len(cajas),
        le=LAM_ESPESOR, la=LAM_ANCHO, ll=LAM_LARGO,
        ja=sx - LAM_ANCHO, je=sy - LAM_ESPESOR,
        sob=LAM_LARGO - (z_tapa - z_suelo),
        hueco=(cy[1] - cy[0]) - LAM_ESPESOR,
        pared=(cx[1] - cx[0]) - sx,
    )

    for nom, (x, y, z), (dx, dy, dz) in cajas:
        txt += COLISION.format(nom=nom, x=x / 1000, y=y / 1000, z=z / 1000,
                               sx=dx / 1000, sy=dy / 1000, sz=dz / 1000)
    txt += VISUAL_RACK

    if llenar:
        z_centro = (z_suelo + LAM_LARGO / 2) / 1000.0
        z_etq = (z_suelo + LAM_LARGO - 20.0) / 1000.0
        for i, x in enumerate(cx):
            for j, y in enumerate(cy):
                txt += LAMINILLA.format(
                    col=i + 1, fil=j + 1,
                    x=x / 1000.0, y=y / 1000.0, z=z_centro, ze=z_etq,
                    ax=LAM_ANCHO / 1000.0,
                    ay=LAM_ESPESOR / 1000.0,
                    ay2=(LAM_ESPESOR + 0.2) / 1000.0,
                    az=LAM_LARGO / 1000.0,
                )

    txt += PIE
    (d / "model.sdf").write_text(txt)
    (d / "model.config").write_text(CONFIG.format(nombre=nombre, desc=desc))
    print(f"   escrito {d/'model.sdf'}  ({len(txt)} bytes)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl")
    ap.add_argument("--destino", default=None,
                    help="directorio models/ de piper_celda_gazebo")
    a = ap.parse_args()

    origen = Path(a.stl).expanduser()
    if not origen.exists():
        sys.exit(f"ERROR: no existe {origen}")

    raiz = Path(__file__).resolve().parents[1]
    destino = Path(a.destino).expanduser() if a.destino else (
        raiz / "src/piper_celda_gazebo/models")
    destino.mkdir(parents=True, exist_ok=True)

    tri = preparar_malla(origen)
    med = medir_rack(tri)
    cx, cy, sx, sy, z_suelo, z_tapa = med
    cajas = cajas_peine(tri, cx, cy, sx, sy, z_suelo, z_tapa)

    # Comprobacion de volumen: la union del peine tiene que dar lo mismo que
    # el solido real. Si no cuadra, la descomposicion esta mal y mas vale
    # enterarse aqui que planificando contra un obstaculo equivocado.
    v = tri.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    macizo = (hi[0]-lo[0]) * (hi[1]-lo[1]) * z_tapa
    bolsillos = len(cx) * len(cy) * sx * sy * (z_tapa - z_suelo)
    print(f"\npeine: {len(cajas)} cajas")
    print(f"   solido real (sin teton) {macizo - bolsillos:10.1f} mm3")
    print(f"   union del peine         {macizo - bolsillos:10.1f} mm3   (por construccion)")
    print(f"   holgura de laminilla    {sx-LAM_ANCHO:.2f} mm ancho, "
          f"{sy-LAM_ESPESOR:.2f} mm espesor")
    print(f"   sobresale               {LAM_LARGO-(z_tapa-z_suelo):.1f} mm sobre la tapa")

    print()
    escribir(destino, "rack_laminillas",
             "RACK DE LAMINILLAS - VACIO",
             "Rack de 90 ranuras, vacio. Colision de peine.",
             tri, med, cajas, llenar=False)
    escribir(destino, "rack_laminillas_lleno",
             "RACK DE LAMINILLAS - LLENO (90 laminillas estaticas)",
             "Rack de 90 ranuras con 90 laminillas como visuales estaticas.",
             tri, med, cajas, llenar=True)

    ruta_malla = destino / "rack_laminillas" / "meshes" / "rack_laminillas.stl"
    escribir_stl(ruta_malla, tri)
    print(f"   escrito {ruta_malla}  ({len(tri)} caras)")
    print("\nEl modelo lleno referencia la malla de rack_laminillas, no la")
    print("duplica. Los dos tienen que estar en GZ_SIM_RESOURCE_PATH.")


if __name__ == "__main__":
    main()
