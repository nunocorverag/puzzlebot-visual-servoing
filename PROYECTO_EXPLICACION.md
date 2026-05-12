# Puzzlebot Visual Servoing: Explicacion del Proyecto

## 1. Objetivo del proyecto

Este proyecto implementa un sistema de visual servoing para un Puzzlebot
diferencial usando ROS2 Humble. El robot detecta un objetivo circular
naranja/terracota con una camara CSI en una Jetson Nano, estima la posicion del
objetivo dentro de la imagen y envia esa informacion a un controlador en laptop.
El controlador decide si debe buscar el objetivo, alinearse con el, avanzar,
detenerse al llegar o evitar un obstaculo frontal.

La idea central es cerrar el lazo de control usando vision: la camara no solo
sirve para observar, sino que produce la variable de error que guia el movimiento
del robot.

## 2. Arquitectura general

El sistema esta dividido en tres partes:

```text
Jetson Nano
  - Lee la camara CSI IMX219.
  - Ejecuta vision_node.
  - Publica /vision_state.
  - Ejecuta micro_ros_agent para comunicar la Hackerboard.

Laptop / Docker
  - Ejecuta mpc_node.
  - Ejecuta la FSM de alto nivel.
  - Publica /cmd_vel.
  - Genera logs CSV de diagnostico.

Hackerboard / micro-ROS
  - Recibe /cmd_vel.
  - Controla motores.
  - Publica encoders y sensores, incluyendo /LaserDistance si esta disponible.
```

Los nodos usan `ROS_DOMAIN_ID=0` y `rmw_fastrtps_cpp`. El contenedor Docker de la
laptop corre con `--network host` para que pueda descubrir los topics de la
Jetson y de micro-ROS.

## 3. Topics principales

```text
/vision_state
  Tipo: puzzlebot_msgs/VisionState
  Publica: vision_node
  Consume: mpc_node
  Campos:
    ex              error horizontal normalizado
    area            area del contorno detectado
    object_detected indica si el objetivo esta visible

/cmd_vel
  Tipo: geometry_msgs/Twist
  Publica: mpc_node
  Consume: micro-ROS / puzzlebot_serial_node
  Uso:
    linear.x controla avance
    angular.z controla giro

/LaserDistance
  Tipo asumido: std_msgs/Float32
  Publica: micro-ROS
  Consume: mpc_node
  Uso:
    distancia frontal para activar AVOID

/fsm_state
  Tipo: std_msgs/String
  Publica: mpc_node
  Uso:
    estado actual de la maquina de estados

/mpc_debug
  Tipo: std_msgs/String con JSON
  Publica: mpc_node
  Uso:
    diagnostico de vision, obstaculo, comandos y decisiones

/vision_obstacle_debug
  Tipo: std_msgs/String con JSON
  Publica: vision_node
  Uso:
    debug visual de obstaculos azules
```

## 4. Vision por computadora

La percepcion se ejecuta en `vision_node.py`. Su tarea principal es detectar el
objeto naranja/terracota que el robot debe seguir. El enfoque usado es vision
clasica, no deep learning, porque el problema es controlado y se necesita baja
latencia.

### 4.1 Captura de imagen

La Jetson abre la camara CSI con un pipeline GStreamer basado en
`nvarguscamerasrc`. Esto permite usar la camara IMX219 de forma eficiente en
Jetson Nano.

El frame se procesa en BGR y luego se convierte a HSV:

```text
BGR -> HSV -> mascara -> morfologia -> contornos -> seleccion del objetivo
```

### 4.2 Por que HSV

HSV separa la informacion de color en:

```text
H: tono
S: saturacion
V: brillo
```

Esto es mas conveniente que RGB para segmentar colores, porque el tono del objeto
se mantiene relativamente estable aunque cambie la iluminacion. Para el objeto
naranja/terracota se usa un rango calibrable:

```yaml
hsv_lower: [0, 80, 61]
hsv_upper: [25, 255, 255]
```

El calibrador HSV permite ajustar estos valores con sliders, texto exacto y
presets.

### 4.3 Mascara y morfologia

Despues de aplicar el umbral HSV, se obtiene una imagen binaria:

```text
pixel blanco = posible objetivo
pixel negro  = fondo
```

La mascara se limpia con operaciones morfologicas:

```text
open  -> elimina ruido pequeno
close -> rellena huecos pequenos
```

Esto reduce falsos positivos y hace que los contornos sean mas estables.

### 4.4 Contornos y metricas de forma

El sistema no acepta cualquier mancha naranja. Calcula metricas geometricas para
favorecer circulos completos o semicirculos:

```text
area:
  cantidad de pixeles del contorno

perimeter:
  longitud del borde

circularity:
  4*pi*area / perimeter^2
  cercano a 1 para un circulo ideal

aspect_ratio:
  ancho / alto del bounding box
  cercano a 1 para objetos redondos

fill_ratio:
  area_contorno / area_circulo_minimo_envolvente
  permite aceptar circulos parcialmente visibles
```

El `fill_ratio` es importante porque el objetivo puede estar parcialmente tapado
o salir cortado en la imagen. Un circulo completo tiene fill alto; un semicirculo
puede tener fill alrededor de 0.35 a 0.65, por eso se permite un rango amplio.

### 4.5 Score de deteccion

Cada candidato se evalua con un score ponderado:

```text
score = area_score
      + circularity_score
      + aspect_score
      + fill_score
      + center_score
```

Los pesos actuales priorizan forma sobre area para rechazar manchas amorfas. Si
el score esta por debajo de `min_detection_score`, el candidato se rechaza y
`object_detected=false`.

### 4.6 Suavizado temporal

La vision puede parpadear por ruido o cambios de luz. Para evitar que el robot
reaccione a detecciones de un solo frame se usan:

```text
confirm_frames:
  numero de frames consecutivos necesarios para confirmar deteccion

lost_frames:
  tolerancia antes de declarar perdida total

ex_smoothing_alpha:
  suavizado del error horizontal

area_smoothing_alpha:
  suavizado del area

ex_deadband:
  zona muerta cerca del centro para evitar oscilacion
```

Esto produce una senal mas estable para el controlador.

### 4.7 Salida de vision

El nodo publica:

```text
ex = (cx - image_center_x) / image_center_x
```

Interpretacion:

```text
ex < 0 -> objetivo a la izquierda
ex > 0 -> objetivo a la derecha
ex = 0 -> objetivo centrado
```

Tambien publica `area`, que se usa como aproximacion de distancia. Si el area es
grande, el objeto esta cerca; si el area es pequena, esta lejos.

## 5. Deteccion visual de obstaculos azules

Ademas del objetivo naranja, el sistema puede marcar obstaculos azules en la
imagen. Esto se usa como debug visual y no reemplaza aun a `/LaserDistance`.

Parametros por defecto:

```yaml
enable_blue_obstacle_detection: true
blue_h_min: 90
blue_h_max: 135
blue_s_min: 40
blue_s_max: 255
blue_v_min: 20
blue_v_max: 255
blue_min_area: 800
blue_close_area: 2500
```

El preview dibuja:

```text
BLUE_OBS       obstaculo azul detectado
BLUE_OBS_CLOSE obstaculo azul suficientemente grande/cercano
```

Y publica `/vision_obstacle_debug` con JSON:

```json
{
  "blue_obstacle_detected": true,
  "blue_obstacle_close": false,
  "blue_obstacle_area": 1200.0,
  "blue_obstacle_ex": 0.15,
  "blue_obstacle_bbox": [x, y, w, h],
  "blue_obstacle_count": 1
}
```

Por seguridad, la FSM sigue usando `/LaserDistance` como disparador real de
evasion. La deteccion azul queda lista para una fusion futura.

## 6. Control del robot

El control se ejecuta en `mpc_node.py`. Este nodo recibe `/vision_state` y
publica `/cmd_vel`.

El robot es diferencial, por lo que se controla con:

```text
v     velocidad lineal hacia adelante
omega velocidad angular
```

En ROS:

```text
cmd.linear.x  = v
cmd.angular.z = omega
```

## 7. Visual servoing

Visual servoing significa controlar el movimiento usando errores medidos en la
imagen. En este proyecto se usan dos errores:

```text
e_x:
  error horizontal del centro del objetivo

e_area:
  error de area respecto al area deseada
```

El error horizontal se calcula en vision:

```text
e_x = (cx_objetivo - cx_imagen) / cx_imagen
```

El error de area se calcula en control:

```text
e_area = (area - area_desired) / area_desired
```

Objetivo del control:

```text
e_x -> 0      centrar el objeto
e_area -> 0   llegar a una distancia deseada
```

## 8. Modelo simplificado para MPC

El controlador usa un modelo lineal aproximado:

```text
e_x[k+1]    = e_x[k]    + dt * omega[k]
e_area[k+1] = e_area[k] + dt * Kv * v[k]
```

Este modelo no pretende describir toda la cinematica del robot. Es una
aproximacion suficiente para decidir comandos pequenos y lentos en una demo de
visual servoing.

## 9. MPC por rollout

MPC significa Model Predictive Control. En cada ciclo:

1. Se toma el estado actual de error visual.
2. Se prueban muchos comandos candidatos `(v, omega)`.
3. Se simula cada candidato durante un horizonte corto.
4. Se calcula un costo.
5. Se elige el comando con menor costo.
6. Se publica solo el primer comando.
7. En el siguiente ciclo se repite con nueva vision.

El costo penaliza:

```text
Qx * e_x^2          error horizontal
Qa * e_area^2       error de distancia visual
Rv * v^2            esfuerzo lineal
Ro * omega^2        esfuerzo angular
Px, Pa              error terminal al final del horizonte
```

El conjunto de candidatos esta limitado para que el robot se mueva lento:

```yaml
v_max: 0.06
omega_max: 0.20
v_candidates: 7
omega_candidates: 11
```

Esto hace que la demo sea mas segura y apreciable.

## 10. Signo angular

Fisicamente puede ocurrir que el signo de giro este invertido por la convencion
del robot, motores o firmware. Para no cambiar la vision ni el modelo, se agrega
un parametro:

```yaml
angular_sign: -1.0
```

El comando final se calcula como:

```text
omega_cmd = angular_sign * omega_controller
```

Si el objetivo esta a la izquierda y el robot gira a la derecha, se debe cambiar
`angular_sign` entre `1.0` y `-1.0`.

## 11. FSM de alto nivel

La FSM decide el comportamiento global. Los estados son:

```text
IDLE
SEARCH
ACQUIRE_TARGET
TRACKING
GOAL_REACHED
AVOID
```

### IDLE

Estado seguro. El controlador esta deshabilitado o no hay comportamiento activo.
Publica:

```text
v = 0
omega = 0
```

### SEARCH

Se activa cuando no hay objetivo visible. El robot gira lento:

```text
v = 0
omega = search_direction * search_omega
```

Esto permite barrer el entorno con la camara.

### ACQUIRE_TARGET

Se activa cuando el objetivo aparece mientras el robot estaba buscando. El robot
se detiene brevemente:

```text
v = 0
omega = 0
```

La razon es evitar que el robot siga girando y pierda el objetivo antes de que el
MPC empiece a seguirlo.

Parametros:

```yaml
enable_acquire_state: true
acquire_hold_sec: 0.30
acquire_timeout_sec: 0.80
```

### TRACKING

Se activa cuando el objetivo esta confirmado. Aqui corre el MPC. El robot centra
el objetivo y avanza lentamente hacia el.

Si el objetivo se pierde por muy poco tiempo, se aplica una gracia:

```yaml
target_lost_grace_sec: 0.40
```

Durante esa gracia el robot se queda quieto para evitar oscilaciones entre
TRACKING y SEARCH.

### GOAL_REACHED

Como no hay medicion directa de distancia al objetivo, se usa el area del
contorno como proxy:

```yaml
target_area_stop: 25000.0
target_area_resume: 18000.0
```

Si:

```text
area >= target_area_stop
```

el robot se detiene. Sale del estado si el objetivo se pierde o si el area baja
por debajo de `target_area_resume`.

### AVOID

Se activa con obstaculo frontal cercano segun `/LaserDistance`. Tiene prioridad
sobre SEARCH, ACQUIRE_TARGET, TRACKING y GOAL_REACHED.

Parametros:

```yaml
obstacle_stop_distance: 0.12
obstacle_avoid_distance: 0.30
obstacle_clear_distance: 0.40
avoid_omega: 0.14
avoid_forward_speed: 0.02
```

Comportamiento:

```text
d_obs < stop_distance:
  v = 0
  gira para esquivar

stop_distance <= d_obs < avoid_distance:
  v = avoid_forward_speed
  gira para rodear lentamente

d_obs >= clear_distance:
  sale de AVOID
```

La histeresis evita parpadeo entre estados.

## 12. Stop seguro

El robot puede quedarse con el ultimo comando si un nodo muere de forma abrupta.
Para reducir ese riesgo se implementa:

```text
publish_zero_cmd_burst(reason)
```

Al cerrar el nodo, publica multiples mensajes:

```text
linear.x = 0
angular.z = 0
```

Parametros:

```yaml
safety_zero_burst_count: 10
safety_zero_burst_dt: 0.05
```

El script `scripts/stop_demo.sh` tambien publica 20 comandos cero antes de matar
procesos o contenedores.

## 13. Logging CSV

Para depuracion fisica se genera un log tipo black box en:

```text
/tmp/puzzlebot_logs/mpc_fsm_log_YYYYMMDD_HHMMSS.csv
```

El CSV contiene una fila por ciclo de control. Columnas importantes:

```text
state
previous_state
transition_reason
object_detected
ex
area
last_target_ex
last_target_age_sec
obstacle_raw
obstacle_distance_m
obstacle_available
obstacle_active
v_controller
omega_controller
angular_sign
v_cmd
omega_cmd
cost
solve_ms
stop_commanded
stop_reason
```

Esto permite reconstruir por que el robot tomo una decision.

Ejemplos de `transition_reason`:

```text
search_target_detected_acquire
acquire_hold_complete_tracking
tracking_target_lost_grace
tracking_target_lost_search
goal_area_reached
obstacle_enter_stop_zone
obstacle_enter_avoid_zone
obstacle_clear_target_visible
safety_stop_shutdown
```

## 14. Parametros principales

Control:

```yaml
angular_sign: -1.0
v_max: 0.06
omega_max: 0.20
search_omega: 0.08
target_area_stop: 25000.0
target_area_resume: 18000.0
```

Obstaculos:

```yaml
enable_obstacle_avoidance: true
obstacle_topic: "/LaserDistance"
obstacle_distance_scale: 1.0
obstacle_stop_distance: 0.12
obstacle_avoid_distance: 0.30
obstacle_clear_distance: 0.40
```

CSV:

```yaml
enable_csv_log: true
csv_log_dir: "/tmp/puzzlebot_logs"
csv_log_prefix: "mpc_fsm_log"
csv_flush_every: 1
```

Vision:

```yaml
hsv_lower: [0, 80, 61]
hsv_upper: [25, 255, 255]
min_detection_score: 0.55
confirm_frames: 3
lost_frames: 4
```

## 15. Como correr

Demo completa:

```bash
cd ~/dev_ws/src/control/puzzlebot-visual-servoing
./scripts/run_demo_tmux.sh
```

Parar seguro:

```bash
cd ~/dev_ws/src/control/puzzlebot-visual-servoing
./scripts/stop_demo.sh
```

Calibrador HSV:

```bash
cd ~/dev_ws/src/control/puzzlebot-visual-servoing
./scripts/run_calibrator.sh
```

Ver estado:

```bash
ros2 topic echo /fsm_state
ros2 topic echo /mpc_debug
ros2 topic echo /vision_state
ros2 topic echo /LaserDistance
```

Ver CSV:

```bash
ls -lh /tmp/puzzlebot_logs
```

## 16. Como probar los comportamientos

### Target no visible

Esperado:

```text
state = SEARCH
v_cmd = 0
omega_cmd pequeno
```

### Target aparece

Esperado:

```text
SEARCH -> ACQUIRE_TARGET -> TRACKING
```

### Target centrado y cerca

Esperado:

```text
TRACKING -> GOAL_REACHED
v_cmd = 0
omega_cmd = 0
```

### Obstaculo frontal

Esperado:

```text
state = AVOID
obstacle_active = true
```

### Stop

Ejecutar:

```bash
./scripts/stop_demo.sh
```

Esperado:

```text
cmd_vel = 0,0 repetido
robot detenido
```

### Obstaculo azul

Poner un objeto azul frente a la camara. Esperado:

```text
preview con BLUE_OBS o BLUE_OBS_CLOSE
/vision_obstacle_debug con blue_obstacle_detected=true
```

## 17. Limitaciones actuales

1. La deteccion azul no activa AVOID por si sola.
2. La distancia al objetivo se estima con area, no con profundidad real.
3. El modelo MPC es aproximado y funciona mejor con velocidades bajas.
4. La deteccion HSV depende de iluminacion y requiere calibracion.
5. `/LaserDistance` debe tener unidades correctas mediante `obstacle_distance_scale`.

## 18. Posibles mejoras futuras

1. Fusionar obstaculos azules con `/LaserDistance`.
2. Agregar un nodo blackbox logger independiente que combine todos los topics.
3. Estimar distancia al objetivo usando calibracion geometrica de camara.
4. Agregar pruebas automatizadas de transiciones FSM.
5. Implementar recuperacion mas inteligente usando memoria del ultimo `ex`.

