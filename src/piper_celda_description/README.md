# piper_celda_description

Descripción del brazo **AgileX PiPER X** adaptada a **Gazebo Harmonic + ROS 2 Jazzy**
para la celda de digitalización de Omica.

Origen: `github.com/agilexrobotics/piper_isaac_sim`, paquete `piper_x_description`.

## Cambios respecto al URDF original

| # | Problema en el original | Solución aquí |
|---|---|---|
| 1 | `Link7` (dedo) declarado con `mass="0.0"` y tensor de inercia todo en cero. Gazebo lo rechaza o lo vuelve inestable. | Se le asigna la masa (0.026482 kg) y la inercia de `Link8`, que es su espejo. |
| 2 | Colisiones usando las mismas mallas DAE que el visual: `J2.dae` son 100 980 triángulos. Inviable en simulación. | Cascos convexos en STL para `base_link`–`Link6`. De 240 765 a 7 140 triángulos de colisión. Los dedos `Link7`/`Link8` conservan malla original (1 838 tri) porque un casco convexo taparía la abertura de la pinza. |
| 3 | `joint7` y `joint8` como dos prismáticas independientes. La pinza real tiene un solo actuador. | `joint8` declarado `<mimic>` de `joint7` con multiplicador −1. Solo se comanda `joint7`. |
| 4 | Sin `ros2_control`, sin plugin de simulador, sin propiedades de contacto. | Bloque `ros2_control` con `gz_ros2_control/GazeboSimSystem` + plugin `gz_ros2_control-system`. Fricción alta (μ=1.6) en los dedos. |
| 5 | Sin frame de herramienta. | Link `tcp` a 170 mm de `Link6`, entre los dedos. Es el frame que planificará MoveIt. |
| 6 | Recorrido de pinza ±0.05 m (100 mm de apertura), que no corresponde al hardware. | Limitado a 0.035 m. Ajustar si mides otra cosa en el brazo real. |

## Cinemática verificada

- 6 revolutas (`joint1`–`joint6`) + 1 GDL de pinza (`joint7`, con `joint8` espejo).
- Home (todos los joints en 0): TCP en (0.267, 0, 0.233) m respecto a `base_link`.
- Alcance horizontal máximo: **838 mm**, y **833 mm** a la altura de la base.

## Uso

```bash
# Ver el URDF resultante
xacro urdf/piper_x.urdf.xacro usar_gazebo:=false > /tmp/piper.urdf
check_urdf /tmp/piper.urdf

# Sin anclaje al mundo (brazo libre)
xacro urdf/piper_x.urdf.xacro fijar_a_mundo:=false
```

### Argumentos del xacro

| Argumento | Default | Para qué |
|---|---|---|
| `prefix` | `""` | Prefijo de nombres, por si algún día hay dos brazos. |
| `fijar_a_mundo` | `true` | Añade link `world` + joint fijo a `base_link`. Necesario para que el brazo no se vuelque sobre la mesa. |
| `usar_gazebo` | `true` | Incluye `ros2_control` y el plugin. Ponlo en `false` para RViz puro o para MoveIt Setup Assistant. |
| `controllers_file` | `config/piper_controllers.yaml` | YAML que carga el plugin. |

## Controladores

- `joint_state_broadcaster`
- `brazo_controller` — `JointTrajectoryController` sobre `joint1`–`joint6`
- `pinza_controller` — `JointTrajectoryController` sobre `joint7`

## Pendientes conocidos

- Los cascos convexos de `Link2` y `Link3` engordan ligeramente los links respecto a la
  geometría real. Para el chequeo fino de holguras contra los microscopios conviene
  contrastar con el visual.
- La pinza es la de serie. Según la decisión de rediseño de mordazas (Delrin/PEEK con
  nido en V), habrá que sustituir la geometría de `Link7`/`Link8`.
