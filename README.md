# PiperCelda — celda de digitalización Omica

Simulación en Gazebo Harmonic + MoveIt 2 de una celda robótica que toma
laminillas de un escurridor y las coloca en microscopios Zaber Nucleus MSR.
El brazo es un **AgileX PiPER X** de 6 GDL con pinza paralela.

---

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim` 8.x)

```bash
sudo apt install -y \
  ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-moveit ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller ros-jazzy-position-controllers \
  ros-jazzy-xacro ros-jazzy-controller-manager
```

Solo para regenerar geometría (no hace falta para correr la simulación):

```bash
pip install --break-system-packages trimesh shapely rtree mapbox-earcut \
  fast-simplification coacd scipy numpy
```

---

## Arrancar

Dos terminales.

### Terminal 1 — Gazebo

```bash
cd ~/Omica/PiperCelda
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
ros2 launch piper_celda_gazebo celda_piper.launch.py
```

Espera a que los tres controladores estén activos:

```bash
ros2 control list_controllers
# joint_state_broadcaster   ... active
# brazo_controller          ... active
# pinza_controller          ... active
```

### Terminal 2 — MoveIt + RViz

```bash
cd ~/Omica/PiperCelda
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch piper_celda_moveit moveit.launch.py
```

RViz arranca ya configurado y publica la escena de la celda a los 10 s.
Espera a ver `You can start planning now!`.

Para planificar: panel **MotionPlanning**, pestaña *Planning*, grupo `brazo`,
elige un *Goal State* o arrastra el marcador del TCP, **Plan**, **Execute**.

### Argumentos útiles

| Launch | Argumento | Por defecto | Para qué |
|---|---|---|---|
| `celda_piper` | `gui` | `true` | `false` = solo servidor |
| `celda_piper` | `pausado` | `false` | arrancar en pausa |
| `celda_piper` | `x` `y` `z` `yaw` | de `pose_brazo.yaml` | mover la base del brazo |
| `moveit` | `rviz` | `true` | `false` = solo `move_group` |
| `moveit` | `escena` | `true` | publicar la geometría de la celda |
| `moveit` | `seguir` | `true` | seguir en vivo lo que muevas en Gazebo |
| `moveit` | `retardo` | `10.0` | espera antes de publicar la escena |

---

## Estructura

```
~/Omica/PiperCelda/src/
├── piper_celda_description/     brazo: URDF, mallas, controladores
│   ├── config/pose_brazo.yaml   POSE DE LA BASE (fuente única)
│   ├── config/piper_controllers.yaml
│   ├── meshes/dae/              visual (9 mallas)
│   ├── meshes/collision/        cascos convexos
│   ├── meshes/extra/            gripper_base y J8 corregida
│   └── urdf/piper_x.urdf.xacro
├── piper_celda_gazebo/          mundo y modelos de la celda
│   ├── worlds/celda_piper.sdf
│   ├── models/{mesa,microscopio_msr,escurridor_60,laminilla}
│   └── launch/celda_piper.launch.py
└── piper_celda_moveit/          planificación
    ├── config/piper_x.srdf      grupos y matriz de colisiones
    ├── config/moveit.rviz
    ├── launch/moveit.launch.py
    └── scripts/
        ├── publicar_escena_celda.py
        └── manipular_laminilla.py
```

---

## La laminilla

MoveIt trata la laminilla como obstáculo rígido, así que rechaza cualquier
trayectoria que la roce — incluido el propio agarre. Dos mecanismos, en
momentos distintos:

```bash
# ANTES de planificar la aproximación: los dedos pueden tocarla
ros2 run piper_celda_moveit manipular_laminilla.py permitir

# AL CERRAR la pinza: pasa a ser parte del robot y viaja con él
ros2 run piper_celda_moveit manipular_laminilla.py adjuntar

# AL SOLTARLA
ros2 run piper_celda_moveit manipular_laminilla.py soltar

# Ver en qué estado está todo
ros2 run piper_celda_moveit manipular_laminilla.py estado
```

Con `--objeto laminilla_02 --link Link7` para otras laminillas o links.

---

## Cambiar la pose del brazo

**Solo** en `src/piper_celda_description/config/pose_brazo.yaml`. Ese archivo
lo leen los dos launch. Si pones la pose en cualquier otro sitio, Gazebo y
MoveIt acabarán discrepando y el fallo es silencioso: no hay error, solo
planes que no corresponden a la simulación.

---

## Mover objetos de la celda

Arrastra los modelos en la GUI de Gazebo: la escena de MoveIt se actualiza
sola en menos de un segundo (`seguir:=true`). Dos límites:

- Solo sigue **poses**. Si añades o borras un modelo, reinicia el nodo.
- Los cambios **no** se guardan. Cuando te convenza la disposición:
  *File → Save World As* sobre `src/piper_celda_gazebo/worlds/celda_piper.sdf`.

---

## Regenerar geometría

Solo si cambia el CAD de origen.

```bash
# Reexportar el microscopio desde FreeCAD (workbench Mesh Design,
# Meshes -> Create mesh from shape, desviación 0.1 mm), luego:
python3 herramientas/10_orientar_microscopio.py ~/Downloads/model.stl

# Colisiones por descomposición convexa (~16 L, sigue la forma real)
python3 herramientas/12_colision_convexa.py --piezas 48

# Alternativa por cajas (~25 L, más rápida de evaluar)
python3 herramientas/11_regenerar_colisiones.py --cajas 16
```

Ambas tienen `--revertir`.

---

## Problemas conocidos

**Los microscopios parpadean en RViz.** Los 48 cascos convexos se solapan y
compiten por los mismos píxeles. El *Scene Alpha* está a 0.75 para
disimularlo. Es cosmético.

**`No executable found` tras extraer un tar.** Los scripts necesitan permiso
de ejecución en el fuente, porque `--symlink-install` instala enlaces:

```bash
chmod +x src/piper_celda_moveit/scripts/*.py
colcon build --symlink-install --packages-select piper_celda_moveit
```

**`ParseError: not well-formed` al publicar la escena.** Algún `model.sdf`
tiene `<`, `>`, `&` o `--` dentro de un comentario. Gazebo lo tolera (usa
TinyXML2), Python no. Para localizarlo:

```bash
for f in src/piper_celda_gazebo/models/*/model.sdf; do
  python3 -c "import xml.etree.ElementTree as E,sys;E.parse(sys.argv[1])" "$f" \
    2>/dev/null && echo "OK   $f" || echo "ROTO $f"
done
```

En los comentarios de SDF, usa `===` para separar, no guiones.

**Gazebo renderiza en la Intel en vez de la NVIDIA.** Es óptimus sin
offload; de ahí el `failed to create dri2 screen`. Las variables
`__NV_PRIME_RENDER_OFFLOAD` del arranque lo corrigen.

---

## Notas sobre el modelo del brazo

El `piper_x_description` original de AgileX (repo `piper_isaac_sim`) tiene
tres defectos que este paquete corrige:

1. **Falta el `gripper_base`.** El URDF monta los dedos directamente sobre
   `Link6`, dejando 33 mm de aire. La malla se tomó de la descripción
   canónica de AgileX, girada 90° en Z para la convención de la variante X.
2. **`Link7` con `mass=0`** e inercia nula. Gazebo lo rechaza. Se le asignó
   la masa e inercia de `Link8`, que es su espejo.
3. **`J8.dae` con dos triángulos parásitos**, uno a 240 mm del origen.

Además, las colisiones originales usaban las mismas mallas que el visual
(`J2.dae` son 100 980 triángulos). Se sustituyeron por cascos convexos:
240 765 → 7 140 triángulos. Los dedos conservan malla original, porque un
casco convexo taparía la abertura de la pinza.

**Datos verificados por cinemática directa:** alcance horizontal máximo
838 mm, y 833 mm a la altura de la base. Home (todos los joints en 0) deja
el TCP en (0.267, 0, 0.233) m respecto de `base_link` — pero esa pose está
sobre dos topes articulares (`joint2` tiene límite inferior 0, `joint3`
superior 0), así que el SRDF define `plegado` y `listo` para evitarla.
