# Cómo funciona la celda, por dentro

Documento para entender qué hace cada pieza y poder modificarla. Complementa
al README, que es la guía de uso.

---

## 1. Las tres capas y por qué están separadas

Corren tres cosas distintas que se hablan por ROS 2:

```
   GAZEBO                    ros2_control                MOVEIT
   física, contactos    <->  puente al robot      <->    planificación
   render                    controladores               colisiones
```

**Gazebo** simula física: gravedad, contactos, inercias. No sabe planificar.
Solo obedece posiciones articulares y calcula qué pasa.

**ros2_control** es el puente. Dentro de Gazebo corre el plugin
`gz_ros2_control`, que expone cada articulación como una *interfaz*: una de
comando (dónde quiero que esté) y varias de estado (dónde está, a qué
velocidad, con qué esfuerzo). Encima viven los controladores, que traducen
trayectorias en comandos.

**MoveIt** planifica. Nunca toca Gazebo directamente: calcula una trayectoria
y se la entrega a un controlador. Su modelo del mundo es la *escena de
planificación*, que alguien tiene que rellenar — ese alguien es nuestro script.

La separación importa porque **cada capa puede estar bien y el conjunto mal**.
La mitad de los problemas de esta sesión fueron eso: MoveIt planificando con
el brazo en una pose y Gazebo simulándolo en otra, o dos simulaciones a la vez
y el comando yendo a la que no mirabas.

---

## 2. El recorrido de un movimiento

Cuando pides "lleva el TCP a este punto":

1. **RViz o un script** manda un *goal* a la acción `/move_action`.
2. **`move_group`** recibe la petición. Aplica adaptadores previos (¿el estado
   inicial es válido? ¿está dentro de límites?) y llama a **OMPL**.
3. **OMPL** busca un camino en el espacio de configuraciones muestreando al
   azar. Cada muestra se comprueba contra la escena de planificación con FCL.
   Si no encuentra ninguna configuración válida que alcance el objetivo, sale
   el `Unable to sample any valid states for goal tree`.
4. Encontrado el camino, un adaptador posterior le pone tiempos
   (`AddTimeOptimalParameterization`) respetando `joint_limits.yaml`.
5. `move_group` mira `moveit_controllers.yaml`, ve qué controlador maneja esas
   articulaciones, y le manda la trayectoria por `FollowJointTrajectory`.
6. **`brazo_controller`** interpola la trayectoria y escribe posiciones en las
   interfaces de comando, ciclo a ciclo.
7. **`gz_ros2_control`** aplica esas posiciones a las articulaciones de Gazebo.
8. Gazebo integra la física y publica el estado de vuelta, que sube por la
   misma cadena hasta `/joint_states`.

Si algo falla, el paso donde falla determina el síntoma. Vale la pena
memorizar tres:

- *"Unable to sample any valid states"* → paso 3. El objetivo es inalcanzable
  o está en colisión.
- *"followed 0% of requested trajectory"* → cálculo cartesiano. El punto de
  partida ya está en colisión, o el camino recto es imposible.
- *"Aborted due to path tolerance violation"* → paso 6. El controlador manda
  pero la articulación no llega.

---

## 3. Las tres piezas del brazo

### `piper_x.urdf.xacro` — la geometría y la cinemática

Un xacro es un URDF con plantillas. Describe **links** (piezas rígidas, con
masa, inercia, malla visual y malla de colisión) y **joints** (cómo se conecta
cada link con el anterior y en qué eje gira o desliza).

La cadena es `world → base_link → Link1 … Link6 → gripper_base → Link7/Link8`,
más un link `tcp` de cortesía.

Tres cosas que conviene entender:

**El link `world` y `anclaje_mundo`.** Sin ellos el brazo sería un objeto
libre. La junta fija `world → base_link` es la que lo planta sobre la mesa, y
su origen sale de `pose_brazo.yaml`. Esta junta es la razón de que Gazebo y
MoveIt coincidan: los dos leen el mismo archivo.

**Colisión distinta del visual.** El visual usa las mallas `.dae` originales
(`J2.dae` son 100 980 triángulos). La colisión usa cascos convexos en STL, con
7 140 triángulos en total. Comprobar colisiones contra 240 000 triángulos en
cada muestra de OMPL sería inviable, y la precisión no aporta nada a esta
escala. Los dedos son la excepción: conservan malla original, porque un casco
convexo taparía la abertura de la pinza.

**`gripper_base` no viene del fabricante.** El URDF de AgileX para la variante
X monta los dedos directamente sobre `Link6`, dejando 33 mm de aire. La malla
se tomó de su descripción canónica y se giró 90° en Z.

### `piper_x.gazebo.xacro` — la interfaz de control

Aquí vive el bloque `<ros2_control>`, que declara qué articulaciones se pueden
comandar y con qué interfaces. Y el `<plugin>` que mete `gz_ros2_control`
dentro de Gazebo, apuntándole al YAML de controladores.

Aquí también están las propiedades de contacto: fricción alta (μ=1.6) en los
dedos para que la laminilla no resbale, baja en el resto.

`joint8` se comanda explícitamente, en espejo de `joint7`. La pinza real tiene
un solo actuador y lo natural sería un `<mimic>`, pero en Jazzy eso dio
problemas y la solución fue mandar los dos dedos desde el controlador.

### `piper_controllers.yaml` — los controladores

Tres: `joint_state_broadcaster` (publica `/joint_states`), `brazo_controller`
(joints 1–6) y `pinza_controller` (joints 7 y 8).

Las **tolerancias** son el parámetro con más consecuencias. Son holgadas a
propósito en la pinza: al cerrar sobre la laminilla el dedo *tiene* que
quedarse corto respecto al comando, porque hay vidrio en medio. Con tolerancia
estricta, cada agarre exitoso se reportaría como fallo. El precio es que si la
pinza se atasca de verdad, el controlador no avisa.

---

## 4. Las tres piezas de MoveIt

### `piper_x.srdf` — lo que el URDF no dice

El URDF describe la máquina. El SRDF describe **cómo usarla**:

- **Grupos**: `brazo` es la cadena de `base_link` a `tcp`; `pinza` son los dos
  dedos. Planificas sobre un grupo, no sobre el robot entero.
- **Poses con nombre**: `plegado`, `listo`, `abierta`, `cerrada`. La pose de
  todo-ceros no sirve como *home* porque `joint2` tiene límite inferior 0 y
  `joint3` superior 0: con ceros el brazo queda sobre dos topes y el
  planificador rechaza ese estado inicial.
- **La matriz de colisiones**: 26 pares desactivados. Sin ella, MoveIt
  comprobaría cada par de links en cada muestra, y además rechazaría estados
  válidos — dos links contiguos siempre se tocan en la junta.

Esa matriz no está escrita a ojo. Se calculó muestreando 3 000 configuraciones
aleatorias y midiendo qué pares llegaban a tocarse: 11 adyacentes, 14 que no
se tocaron nunca, y `Link7`/`Link8`, que hay que desactivar o MoveIt se niega
a cerrar la pinza.

### `kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml`

El primero elige el solucionador de cinemática inversa (KDL). El segundo pone
techo a velocidades y aceleraciones — deliberadamente por debajo de lo que
declara el URDF, porque en simulación de layout interesa poder seguir el
movimiento con la vista. El tercero configura OMPL: `RRTConnect` por defecto,
que crece dos árboles desde inicio y meta hasta que se encuentran.

### `moveit_controllers.yaml`

Le dice a MoveIt qué controlador maneja qué articulaciones. Si los nombres no
coinciden con `piper_controllers.yaml`, MoveIt planifica pero *Execute* no
hace nada.

---

## 5. `publicar_escena_celda.py` — meter el mundo en MoveIt

MoveIt no sabe nada de Gazebo. Su mundo es la escena de planificación, y
arranca vacía.

Este nodo lee **el mismo `celda_piper.sdf` que carga Gazebo**, resuelve cada
`model://` contra la misma carpeta `models/`, y publica la geometría como
objetos de colisión. Esa decisión es deliberada: no hay geometría escrita a
mano en ningún sitio, así que la escena y la simulación no pueden divergir.

Lo que hace, en orden:

1. **Parsea el SDF** con `ElementTree`. Aquí apareció un problema curioso:
   Gazebo usa TinyXML2, que tolera `<`, `>` y `--` dentro de comentarios;
   Python sigue el estándar y los rechaza. Mismo archivo, dos lectores con
   criterios distintos.
2. **Compone poses en tres niveles**: la del `<include>` en el mundo, la del
   `<model>` dentro de su archivo, y la de cada `<collision>` dentro del link.
   Se multiplican matrices de rotación y se suman traslaciones.
3. **Lee las mallas**. Lleva un lector de STL binario en Python puro para no
   depender de `trimesh` en tiempo de ejecución.
4. **Publica** por el servicio `apply_planning_scene`.

Con `seguir:=true` no termina: se suscribe a las poses que Gazebo publica
—puenteadas a `/poses_gazebo`— y reenvía cualquier modelo que muevas en la
GUI. Guarda la geometría **en el marco de cada modelo** y solo recompone la
pose cuando cambia; releer las 48 mallas del microscopio a 2 Hz sería
inviable. Refresca cada 0.5 s y solo si el modelo se movió más de 1 mm.

Solo sigue poses. Si añades o borras un modelo en Gazebo, hay que reiniciarlo.

---

## 6. `agarrar_laminilla.py` — la secuencia

### La geometría, primero

La laminilla mide 76 × 26 × 1 mm y está de canto en el escurridor. Su espesor
va en X (dirección de las ranuras), su ancho en Y, su alto en Z. Centro en
z = 0.798, borde superior en 0.836.

El punto de agarre está 20 mm por debajo del borde: **z = 0.816**. El TCP está
en la punta de los dedos, así que poner el TCP ahí hace que la zona de contacto
de las mordazas cubra exactamente esos 20 mm superiores.

**La orientación** determina dónde apoya la mordaza:

| | cierra sobre | apoyo de la mordaza |
|---|---|---|
| `cantos` | 26 mm de ancho | 1 mm de canto |
| `caras` | 1 mm de espesor | 26 mm de cara |

En las dos, el eje Z del TCP mira hacia −Z del mundo. Lo que cambia es el eje
X, que es por donde cierran los dedos. Están precalculadas como cuaterniones
en `ORIENTACIONES`.

La abertura se deduce de ahí: cada dedo se separa `joint7` del centro, así que
la abertura total es el doble. Por eso no puede haber un valor único por
defecto — 26 mm y 1 mm piden aperturas muy distintas.

### Los siete pasos

**1. Abrir la pinza.** Antes de acercarse, no después. Con una espera de un
segundo tras confirmar el movimiento: la acción reporta éxito cuando el
controlador alcanza el objetivo, pero los dedos siguen asentándose, y arrancar
el descenso con la pinza todavía abriendo empuja la laminilla.

**2. Permitir el contacto.** Se marca en la matriz de colisiones que los links
de la pinza pueden tocar la laminilla. Sin esto no hay plan posible: agarrar
*es* tocar. También se permite el contacto laminilla–escurridor, y eso no es
una concesión: la laminilla **arranca metida en la ranura**, así que en cuanto
se adjunta al robot esa interpenetración cuenta como colisión robot–mundo y
ninguna trayectoria de salida sería válida.

**3. Ir al preagarre**, 100 mm por encima. Este tramo sí es un plan articular
normal: no importa por dónde pase mientras evite obstáculos.

**4. Descenso, en dos tramos.** Libre de colisiones hasta 35 mm del agarre; los
últimos 35 mm sin comprobar colisiones. Esos milímetros son contacto por
definición y exigir ausencia de colisión ahí es contradictorio — el
planificador se planta justo al llegar al borde superior de la laminilla. El
tramo liberado es corto, estrictamente vertical, y parte de un punto que sí se
verificó. Lo único que puede tocar es la laminilla que vas a agarrar.

Los dos tramos son **cartesianos**, no articulares. Un plan articular solo
garantiza que inicio y fin están libres; por el camino el TCP puede describir
un arco, y con las ranuras vecinas a 8 mm un arco las barre.

**5. Cerrar la pinza.**

**6. Adjuntar.** La laminilla pasa del mundo a la lista de cuerpos adjuntos del
robot. Desde ahí MoveIt la mueve con el brazo y comprueba **sus** colisiones
contra la celda — importante, porque 76 mm de vidrio sobresaliendo de la pinza
cambian el volumen que hay que hacer pasar entre los microscopios.

Dos trampas aquí, ambas encontradas a golpes:

- MoveIt calcula la posición del objeto *relativa al link* usando el estado del
  robot en ese instante. Si se adjunta con el brazo en casa, la laminilla queda
  enganchada con el desfase equivocado. Por eso el script declara
  explícitamente en qué configuración está el robot.
- No hay que mandar un `REMOVE` del objeto en el mundo. Adjuntar por id ya lo
  saca. Si se manda, MoveIt procesa el borrado primero y luego no encuentra qué
  adjuntar: queda un cuerpo vacío en la pinza.

**7. Retirada**, también en dos tramos, en espejo del descenso.

### Modo seco contra ejecución real

Con `--solo-planificar` el robot no se mueve, así que cada tramo debe partir
del final planificado del anterior — si no, el segundo tramo cartesiano se
calcula desde donde está el brazo de verdad y devuelve 0 %.

En ejecución real ocurre lo contrario: cada tramo parte del estado **medido**.
Si el controlador se queda corto por unos milímetros, el siguiente tramo lo
tiene en cuenta en vez de arrastrar el error.

El modo seco deja la laminilla adjunta en la escena. Para devolverla a su
sitio, vuelve a correr `publicar_escena_celda.py`.

---

## 7. Qué tocar para cambiar cada cosa

| Quiero… | Toco |
|---|---|
| mover la base del brazo | `pose_brazo.yaml`, y **solo** ahí |
| cambiar dónde agarra | `--bajo-borde`, o el cálculo de `z_agarre` |
| agarre por cantos | `--orientacion cantos` |
| más o menos apriete | `--apriete`, `--holgura` |
| que vaya más lento | `--velocidad 0.05` |
| brazo más rápido en general | `joint_limits.yaml` |
| que deje de chocar con algo | la matriz del `piper_x.srdf` |
| añadir un obstáculo | el `celda_piper.sdf`; la escena lo recoge sola |
| otro planificador | `ompl_planning.yaml` |
| colisiones del microscopio | `herramientas/12_colision_convexa.py` |

---

## 8. Lo que esta simulación no te dice

**El agarre físico.** MoveIt lleva la laminilla adjunta cinemáticamente,
pegada a `Link7`, pase lo que pase. Gazebo simula contacto de verdad. Los dos
pueden discrepar, y de hecho discrepan: una laminilla de 5 g y 1 mm sujeta por
una pinza controlada en posición tiende a salir disparada por el error de
penetración. Que RViz muestre el agarre perfecto no significa que el agarre
real vaya a funcionar.

**Las mordazas.** El agarre por caras que usa el script por defecto **no cabe
con el escurridor lleno**: las ranuras están a 8 mm y cada mordaza tendría que
medir menos de 3 mm. Sirve para validar cinemática y secuencia; para operación
real dependen del rediseño de mordazas.

**La vibración.** Nada en esta simulación modela lo que le pasa a la óptica de
un microscopio cuando un brazo se mueve a su lado.
