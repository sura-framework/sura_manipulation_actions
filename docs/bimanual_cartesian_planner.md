# Planificación bimanual de *facing*

`PlanBimanualCartesianTrajectory` planifica una trayectoria articular conjunta en la que las dos herramientas se encaran: cada una apunta hacia la otra a lo largo de la línea que las une, con un roll relativo y una distancia pedidos. Solo se mueven las articulaciones de los brazos; la base queda fija. No usa planner local. De cada plan salen dos referencias ligadas por el progreso geométrico `s ∈ [0,1]`:

| Prioridad | Referencia | Destino (backend `facing`) |
| --- | --- | --- |
| Primaria | `facing_path`: referencia de facing en función de `s`, sin tiempos | `EndEffectors_Facing` |
| Secundaria | `joint_trajectory` temporizada, con `nominal_progress` por punto | `robot_nominal`, solo brazos |

La acción planifica por defecto sin ejecutar; no activa tareas ni cambia controladores.

## Lanzamiento y lifecycle

Primero debe estar disponible el modelo del robot, su TF, sus estados articulares y la escena con los obstáculos actuales. La lectura requiere estados recientes de todas las articulaciones independientes de una variable del modelo, también las pasivas, incluidas las que no se mueven en este plan, porque también afectan a las colisiones. El estado de la base se mantiene fijo durante el cálculo.

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py \
  execution_backend:=controller use_rviz:=true
ros2 lifecycle set /plan_bimanual_cartesian_trajectory_lifecycle_action_server configure
ros2 lifecycle set /plan_bimanual_cartesian_trajectory_lifecycle_action_server activate
```

El launch no realiza estas transiciones ni activa ningún backend. `configure` crea la acción y los servicios, como en el servidor joint; `activate` habilita los intentos de ejecución; estando inactivo se puede preparar un plan, pero no despacharlo. `deactivate`, `cleanup` y `shutdown` cancelan el trabajo actual y descartan el plan pendiente. Para recargar configuración, usa `cleanup` y vuelve a configurar.

El launch común reúne las acciones articulares y cartesianas, incluyendo la
acción cartesiana de un brazo. El launch `manipulation_cartesian_actions.launch.py`
se conserva como entrada independiente para la acción bimanual; utiliza uno u
otro para evitar servidores duplicados.

La acción por defecto es `/cirtesub/manipulation/plan_bimanual_cartesian_trajectory`, de tipo `sura_manipulation_actions/action/PlanBimanualCartesianTrajectory`.

## Goal y restricciones

| Campo | Significado |
| --- | --- |
| `left_pointing_axis`, `right_pointing_axis` | Eje puro con signo (±x, ±y, ±z) de cada herramienta, en su propio frame, que apunta a la otra. |
| `roll_angle` | Roll relativo alrededor de la línea izquierda → derecha [rad]. `NaN` mantiene el medido. |
| `distance` | Distancia entre herramientas [m], no menor que `planning.facing_min_distance`. |
| `pointing_tolerance` | Ángulo admitido entre cada eje y su referencia [rad], en todo el recorrido. |
| `roll_tolerance`, `distance_tolerance` | Tolerancias de roll [rad] y distancia [m]. Cero es estricto (`1e-6`). |
| `cartesian_step`, `planning_time` | Resolución lineal y presupuesto temporal. Valores no positivos usan el YAML. |
| `velocity_scaling`, `acceleration_scaling` | Factores de límites articulares en `(0, 1]`; cero usa el YAML. |
| `execute` | `false`: publicar y esperar el servicio de ejecución. |
| `execution_backend` | `controller`, `task` o `facing`; vacío usa el parámetro del servidor. |
| `use_stpik` | Solo backend `facing`: simular con STPIK antes de ejecutar. |

El goal no elige brazos: son `pairs.<active_pair>.arms`, en orden `[izquierdo, derecho]`; `active_pair` es por defecto el primero de `pair_names`.

El cero del roll usa un eje de referencia cíclico de cada herramienta, con el signo de su eje de apuntado: ±x → ±y, ±y → ±z, ±z → ±x. El roll es el ángulo con signo, alrededor de la línea, desde la referencia izquierda hasta la derecha, ambas proyectadas en el plano normal. Es la misma geometría que usa `EndEffectorsFacingTask` (`facing_geometry.hpp`).

La referencia empieza en la geometría medida, así que el robot no necesita estar encarado: distancia y roll (arco corto) se interpolan linealmente y el eje de apuntado de cada herramienta recorre el arco corto desde su dirección actual hacia la otra herramienta hasta el eje pedido. Se rechazan el estado inicial con herramientas a menos de la distancia mínima, con roll indefinido (un eje de referencia paralelo a la línea) o con una herramienta apuntando exactamente en sentido contrario, y cualquier recorrido en el que la geometría deje de estar definida. El primer punto articular conserva la configuración medida.

**Primero dirección y roll, después distancia.** Con `a` en `[0, 1)`, tomado del campo `facing_orientation_fraction` del goal o, si es negativo (el valor por defecto del goal), de `planning.facing_orientation_fraction` (0.4 por defecto), el camino tiene dos tramos: hasta `s = a` los ejes de apuntado y el roll llegan al objetivo manteniendo la distancia medida; después solo cambia la distancia. Cada tramo arranca y termina suave y hay un nudo exacto en `s = a`. Si el cambio angular es menor que `planning.angular_step` (0.05 rad) o el de distancia menor que 2 mm, no hay tramos. Con `a = 0` todo cambia a la vez. Se aplica al planner punto a punto y al modo optimizado, y si la búsqueda punto a punto falla, el mensaje indica el tramo (orientación o distancia). `facing_path` contiene los nudos del camino y `nominal_progress` sitúa cada punto de la trayectoria en él.

La velocidad se elige antes de planificar; para cambiarla en un plan pendiente, envía un goal nuevo. `planning_time` limita el cálculo, no la duración del movimiento.

`relative_reference_frame` (en Alpha `cirtesub/base_link`) debe ser un link que no se mueva con las articulaciones comandadas; en él se expresan paths y markers.

**Distancia de la fase de orientación.** El campo `orientation_phase_distance` del goal es la distancia a la que debe llegarse al terminar la fase (`s = a`); desde ella la segunda fase va hasta `distance`. Negativo (por defecto) conserva la medida. Con `a = 0` se ignora.

**Primera fase en espacio articular** (planner punto a punto, `planning.facing_joint_orientation_phase: true`). La primera fase no sigue una referencia impuesta: se busca una postura en `s = a` que cumpla el objetivo de orientación a esa distancia y desde la que la fase de distancia sea completable, y se une con la postura inicial en articulaciones (recta o RRT-Connect). En todo ese tramo se respetan límites articulares, colisiones y: distancia dentro de `facing_phase_distance_margin` de [inicial, fase] y nunca por encima de `facing_phase_max_distance` (0 = sin máximo); cada apuntado y el roll dentro de `facing_phase_pointing_margin` / `facing_phase_roll_margin` de su recorrido de inicio a objetivo. `facing_path` se calcula en ese tramo por cinemática directa sobre la trayectoria temporizada, así que facing, STPIK y la ejecución lo siguen igual que antes. El objeto que une las herramientas no se modela: estos límites son su única protección. El modo optimizado sigue imponiendo la referencia por tramos, con la distancia de fase pedida.

## Optimización de toda la trayectoria (`optimize_trajectory_nominal`)

Con este campo (casilla **Optimize whole trajectory (robot_nominal)** en RQT) el plan no lo calcula el IK punto a punto del servidor, sino el mismo planificador global que usa el modo de un brazo con `robot_nominal`, dentro del controlador (`task_priority/prepare_cartesian_posture`):

1. El servidor construye el `facing_path` desde el estado medido y estima una duración con `facing.max_linear_velocity` (0.3 m/s) y `facing.max_angular_velocity` (1.0 rad/s) multiplicadas por `velocity_scaling`. El progreso sigue un perfil quíntico de reposo a reposo.
2. El controlador siembra la trayectoria proyectando cada nudo sobre el camino de facing, la optimiza entera (SQP) con toda la jerarquía (tareas por encima de facing como restricciones, las intermedias como coste, límites, inclinación, colisiones de la escena MoveIt y velocidades) y, si falla, recurre a la búsqueda por capas con programación dinámica. Valida el resultado con STPIK.
3. La base no se planifica: la máscara de `robot_nominal` anula sus filas y el optimizador congela esas coordenadas. La postura inicial es siempre la medida.
4. Si el plan se rechaza por velocidad, el servidor reintenta con el doble de duración, hasta tres veces.

Requisitos: backend `facing`. Para planificar no hace falta tocar las tareas del controlador: sobre una copia aislada se suponen habilitadas la tarea de facing y `robot_nominal` (colocada justo debajo de facing si estaba por encima) y deshabilitadas las de `pairs.<nombre>.disabled_tasks`; el controlador en marcha no cambia y los brazos no se mueven. La jerarquía del backend `facing` (abajo) sigue siendo obligatoria para ejecutar. Las tolerancias son las de ejecución (`execution.facing_path_tolerance` y `execution.facing_goal_tolerance`); el optimizador mantiene cada fila en un cuarto de la de camino. Las tolerancias del goal no se usan en este modo. La revalidación antes de ejecutar comprueba el estado inicial y repite STPIK si `use_stpik` está activo. `facing.prepare_timeout` (120 s) limita la espera.

## Pesos articulares

El servidor carga `planning.weights` al configurar su lifecycle. Cada valor añade
el coste `w_i · (q_i - q_inicial_i)²`: cuanto mayor sea, más se penaliza mover esa
articulación respecto a su posición medida al iniciar el plan. Un cero elimina
solo esta penalización adicional; no bloquea la articulación. Los costes generales
de continuidad, postura nominal y suavizado siguen aplicándose.

```yaml
# Orden: alpha_left axis_e,d,c,b; alpha_right axis_e,d,c,b
planning.weights: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

La configuración Alpha usa cero en todas: el facing necesita casi todos los grados
de libertad. Para penalizar una articulación, sube su posición. Los pesos se
asocian al orden de variables activas del grupo MoveIt conjunto, no a
`planning.limit_joint_names`. En Alpha son las ocho articulaciones
indicadas arriba. Al configurar se registra cada índice, nombre y peso para
comprobar la correspondencia con el modelo cargado.

Un vector omitido o vacío equivale a pesos cero. Un vector no vacío debe tener
un valor por articulación activa, todos finitos y no negativos; una longitud
incorrecta, NaN, infinito o un valor negativo impiden configurar el servidor.
El planner también valida esos requisitos cuando se utiliza directamente.
Los pesos se conservan con el plan pendiente. Para modificarlos, edita el YAML,
reinicia el servidor para cargarlo y realiza `configure` otra vez. También puedes
hacer `cleanup`, actualizar el parámetro `planning.weights` y hacer `configure`;
`cleanup` descarta cualquier plan pendiente. Los goals nuevos usarán los pesos
cargados.

Los clientes de la interfaz relativa anterior deben actualizarse y regenerar los
mensajes de la acción.

## Ejemplo Alpha y plan pendiente

Los efectores son `cirtesub/alpha_left/standard_jaws_tool` y `cirtesub/alpha_right/standard_jaws_tool`. El cliente de ejemplo toma por defecto la distancia actual de TF y mantiene el roll:

```bash
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py
# Encararse 2 cm más cerca, al 5 % de velocidad y aceleración:
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py \
  --delta -0.02 --velocity-scaling 0.05 --acceleration-scaling 0.05
# Distancia, roll y ejes explícitos:
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py \
  --distance 0.30 --roll 0.0 --left-axis=+x --right-axis=+x
```

El feedback pasa a `waiting_for_execute` después de publicar el plan. El goal sigue abierto hasta ejecutar, cancelar o reemplazar el plan; la trayectoria ya puede consultarse en su tópico y en RViz. Desde otra terminal:

```bash
ros2 service call \
  /plan_bimanual_cartesian_trajectory_lifecycle_action_server/execute_pending_trajectory \
  std_srvs/srv/Trigger '{}'
ros2 service call \
  /plan_bimanual_cartesian_trajectory_lifecycle_action_server/cancel_pending_trajectory \
  std_srvs/srv/Trigger '{}'
```

La respuesta positiva del servicio de ejecución confirma que se solicitó el intento; el feedback y el resultado de la acción informan de la ejecución real. Si el backend está desactivado o no disponible, el plan permanece pendiente. Activar el backend no ejecuta nada: hay que llamar otra vez al servicio. `--execute` del ejemplo hace el intento inicial automáticamente tras planificar, con la misma política de activación manual.

Antes de despachar se comprueban nuevamente el estado inicial, la base y la escena. Los defaults admiten `0.001 rad` en joints revolute, `0.0001 m` en prismatic y desplazamientos de base de `0.005 m / 0.01 rad`. También se revalida la curva completa con las posiciones actuales de las articulaciones no comandadas y los obstáculos actuales. Un plan que ya no cumple estas condiciones se rechaza y exige nueva planificación. Cancelar una ejecución, un rechazo parcial o un fallo después del despacho cancela todos sus goals, incluidas las aceptaciones tardías.

La coherencia de las referencias guardadas se comprueba contra el estado inicial del propio plan. El facing depende solo de la geometría entre herramientas, por lo que una deriva de base dentro de las tolerancias no cambia la referencia; colisiones y tolerancias se comprueban en la escena actual.

## Activación manual de los backends

Para `controller`, el operador debe cargar y activar ambos `JointTrajectoryController` y desactivar los controladores incompatibles que reclamen esas interfaces. Consulta primero el estado real:

```bash
ros2 control list_controllers -c /cirtesub/alpha/controller/controller_manager
```

Por ejemplo, si los dos controladores forward de velocidad son los que están activos:

```bash
ros2 control switch_controllers -c /cirtesub/alpha/controller/controller_manager \
  --deactivate alpha_left_forward_velocity_controller alpha_right_forward_velocity_controller \
  --activate alpha_left_joint_trajectory_controller alpha_right_joint_trajectory_controller
```

Adapta la lista de desactivación a los controladores activos. El YAML define los nombres, servicios `list_controllers` y actions de cada brazo. El servidor solo consulta su estado. Divide la trayectoria por nombres articulares, conservando posiciones, velocidades, aceleraciones y tiempos. Envía ambos goals antes de esperar respuestas, con una fecha de inicio común tres segundos en el futuro y dos segundos de plazo de aceptación. Solo informa éxito cuando ambos resultados lo confirman. El backend debe respetar los stamps y la interpolación quíntica especificada por posiciones, velocidades y aceleraciones.

Para `task`:

```bash
ros2 launch sura_manipulation_actions manipulation_cartesian_actions.launch.py \
  execution_backend:=task use_rviz:=true
```

El bringup carga `task_priority_controller` sin activarlo. Si `list_controllers` muestra `unconfigured`, configúralo primero:

```bash
ros2 control set_controller_state task_priority_controller inactive \
  -c /cirtesub/alpha/controller/controller_manager
```

Prepara la selección de tareas mientras el controlador está inactivo.

Selecciona `bimanual_joint_trajectory`. Esa tarea está añadida a `ros2_control_params_dual_alpha.yaml`, inicialmente deshabilitada, con los ocho joints del grupo `bimanual` y `interpolation_method: quintic`. Desactiva las tareas incompatibles antes de habilitarla. El ejemplo Alpha también tiene `left_pose` y `right_pose` activos con prioridad mayor: pueden bloquear el seguimiento aunque las otras tareas de trayectoria estén desactivadas. Por ejemplo:

```bash
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: left_joint_trajectory, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: right_joint_trajectory, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: left_pose, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: right_pose, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: bimanual_joint_trajectory, enabled: true}'
```

Cuando esté configurado, activa manualmente `task_priority_controller` desactivando los controladores incompatibles. Por ejemplo, para pasar desde los dos JTC activos:

```bash
ros2 control switch_controllers -c /cirtesub/alpha/controller/controller_manager \
  --deactivate alpha_left_joint_trajectory_controller alpha_right_joint_trajectory_controller \
  --activate task_priority_controller
```

También puedes hacerlo desde el panel de tareas. El nombre del servicio anterior corresponde al plugin de controlador del workspace; si usas el runtime independiente, consulta su namespace. La ruta del backend conjunto es `/cirtesub/alpha/controller/task_priority/tasks/bimanual_joint_trajectory/follow_joint_trajectory`, remapeada por el bringup a partir de `task_ids`.

Se envía una única trayectoria completa a esa tarea, con stamp cero: los dos brazos comparten el reloj y el avance de la misma tarea. `JointTrajectoryTask` usa el reloj monotónico existente; con simulación, su avance sigue tiempo real. No reproduce fechas futuras de ROS como los JTC. La tarea rechaza goals estando deshabilitada, y el plugin los rechaza si el controlador no está activo. Cada tarea admite un único goal simultáneo; cancelar, deshabilitar o desactivar el controlador borra la trayectoria almacenada. Reactivar no reanuda nada. `hold_last_point=true` mantiene la postura después del éxito hasta cancelar, deshabilitar, desactivar o reemplazar la trayectoria. El éxito libera el goal para admitir otro. Las tareas existentes conservan interpolación `linear` por defecto.

## Configurar otros brazos

Usa una copia de `config/bimanual_cartesian_planner.yaml`. Cada brazo requiere un grupo MoveIt de cadena serial, un tip afectado por ese grupo y su endpoint `FollowJointTrajectory`. El grupo de cada pareja debe contener exactamente la unión de las articulaciones activas de sus dos brazos, sin joints compartidos. Se admiten distintos números de articulaciones actuables revolute y prismatic de una variable; los joints pasivos dentro del grupo comandado se rechazan; las cadenas con articulaciones mimic que afecten al tip se rechazan. La configuración se valida al cargar el modelo y al planificar.

Ejemplo de estructura, dentro de `/**: ros__parameters:`:

```yaml
arm_names: [arm_a, arm_b]
relative_reference_frame: robot_base
pair_names: [pair_ab]
arms.arm_a.group: group_a
arms.arm_a.tip_frame: tool_a
arms.arm_a.controller_action: /a/follow_joint_trajectory
arms.arm_a.controller_name: a_controller
arms.arm_a.manager_service: /controller_manager/list_controllers
arms.arm_b.group: group_b
arms.arm_b.tip_frame: tool_b
arms.arm_b.controller_action: /b/follow_joint_trajectory
arms.arm_b.controller_name: b_controller
arms.arm_b.manager_service: /controller_manager/list_controllers
pairs.pair_ab.arms: [arm_a, arm_b]
pairs.pair_ab.group: both_groups
pairs.pair_ab.task_action: /tasks/combined/follow_joint_trajectory
pairs.pair_ab.task_controller_name: task_priority_controller
pairs.pair_ab.task_manager_service: /controller_manager/list_controllers
planning.limit_joint_names: []
planning.lower_limits: []
planning.upper_limits: []
```

Configura también `joint_states_topic`, `relative_reference_frame`, los efectores y los límites de velocidad y aceleración del nuevo modelo. La referencia debe ser un link independiente de los brazos. Para el backend task, configura una única `JointTrajectoryTask` con todos esos joints e interpolación quíntica en su propio paquete de robot. No hay tamaños articulares fijos en el algoritmo.

El launch admite `planner_config_file`, `robot_description_package/file`, `robot_description_semantic_package/file`, `xacro_arguments`, `srdf_xacro_arguments`, `moveit_config_package`, `joint_limits_file`, `kinematics_file`, `joint_states_topic`, `planning_scene_input_topic`, `relative_reference_frame`, `fixed_frame` y `rviz_config_file`. Los paths pueden ser absolutos o relativos al paquete correspondiente. Cambia conjuntamente URDF, SRDF, límites, configuración y RViz:

```bash
ros2 launch sura_manipulation_actions manipulation_cartesian_actions.launch.py \
  robot_namespace:=my_robot planner_config_file:=/ruta/planner.yaml \
  robot_description_package:=my_description robot_description_file:=urdf/robot.urdf.xacro \
  robot_description_semantic_package:=my_description \
  robot_description_semantic_file:=config/robot.srdf \
  xacro_arguments:='' srdf_xacro_arguments:='' \
  moveit_config_package:=my_description joint_limits_file:=config/limits.yaml \
  kinematics_file:=config/kinematics.yaml fixed_frame:=world \
  joint_states_topic:=/my_robot/joint_states rviz_config_file:=/ruta/my_robot.rviz
```

El launch expande `${robot_namespace}` en los valores del YAML del planificador. El YAML MoveIt existente de Alpha y el RViz de ejemplo contienen nombres `cirtesub/...`: al cambiar namespace, adapta también esos archivos o proporciona versiones propias. Los límites adicionales del ejemplo proceden de `tasks.joint_limits` de la configuración de seguridad Alpha, se mapean por nombre y se intersectan con el modelo cargado; solo afectan a este planificador. Una postura nominal opcional se define mediante `planning.nominal_joint_names` y `planning.nominal_positions`; por defecto se usa la inicial.

## RViz y salidas

| Tópico por defecto | Tipo |
| --- | --- |
| `/cirtesub/moveit/monitored_planning_scene` | `moveit_msgs/PlanningScene` |
| `/cirtesub/moveit/display_planned_path` | `moveit_msgs/DisplayTrajectory` |
| `/cirtesub/bimanual_planner/joint_trajectory` | `trajectory_msgs/JointTrajectory` |
| `/cirtesub/bimanual_planner/arm_a_path`, `arm_b_path` | `nav_msgs/Path` de las herramientas izquierda y derecha |
| `/cirtesub/manipulation/markers` | `visualization_msgs/MarkerArray` del último plan |

El resultado de la acción lleva `joint_trajectory`, `nominal_progress`, `facing_path`, `left_path` y `right_path`. Los markers (namespaces `bimanual_cartesian/...`) muestran los paths de ambas herramientas, la línea final entre ellas (`facing_line`), los ejes de apuntado finales (`facing_axes`) y las poses finales. Usan `relative_reference_frame`, son `frame_locked` y comparten el publicador `manipulation_marker_visualization` con las demás acciones: un plan nuevo sustituye al anterior. Los paths llevan timestamp cero y se refrescan a 10 Hz.

Los marcadores interactivos del panel RQT definen distancia y roll arrastrando las herramientas; el apuntado se mantiene siempre encarado.

## Método y resolución de validación

Las seis filas son: apuntado izquierdo (2), apuntado derecho (2), roll y distancia. Las angulares usan `planning.orientation_weight` y la distancia `planning.position_weight`. Los módulos separan contexto de modelo/escena, planificación, visualización y transporte; la temporización tiene su módulo propio y el planificador valida los segmentos geométricos. La optimización secuencial usa Eigen y OSQP (`osqp_vendor`), FK y jacobianos de MoveIt, transformados desde el marco de cada cadena al marco común del modelo. Minimiza error de facing, cambios entre waypoints, movimiento articular ponderado respecto al inicio y distancia a la postura nominal. Los pesos no sustituyen la aceptación por límites articulares, tolerancias de facing y colisiones.

Se usa la solución anterior como semilla, hasta 100 iteraciones, ocho semillas locales deterministas y seis niveles de subdivisión; el máximo es 2.000 waypoints y se respeta el presupuesto del goal. El suavizado mantiene bloqueado el primer punto y solo acepta cambios que conservan restricciones. Se temporiza con `IterativeParabolicTimeParameterization` y los factores efectivos del goal (defaults Alpha `0.1 / 0.1`); después se valida la curva quíntica que se ejecutará. Puede subdividir, volver a temporizar y dilatar uniformemente el tiempo para cumplir los límites dinámicos. Los puntos incluyen q/v/a y tiempos estrictamente crecientes desde cero.

Antes de temporizar, cada segmento articular se muestrea con al menos cuatro subdivisiones, aumentando según el desplazamiento máximo por joint (`0.01 rad` o `0.001 m`). En la curva quíntica se comprueban analíticamente extremos de posición, velocidad y aceleración. FK, restricciones cartesianas y colisiones se muestrean con al menos cuatro subdivisiones, con intervalos máximos de `0.01 s` y desplazamientos por joint acotados por la velocidad máxima analítica y las resoluciones anteriores. Los límites de muestras por segmento son 4.096 y 20.000 respectivamente; superarlos produce fallo. Estos parámetros son configurables.

La validación geométrica tiene resolución numérica finita, no constituye una prueba de colisión continua. Un fallo de búsqueda significa que no se encontró una solución válida dentro del presupuesto, no que la pose sea globalmente inalcanzable. Los obstáculos que cambien después del despacho requieren la supervisión de ejecución del sistema existente.

## Pruebas

Las suites C++ cubren la geometría compartida (jacobiano por diferencias finitas y feedforward), el planner de facing (error inicial cero, roll, rechazos, base fija, revalidación), el servidor, la tarea (`FacingReference` y caducidad) y la simulación STPIK. Desde la raíz del workspace:

```bash
cd /home/salva/sura_ws_new_new
colcon test --packages-select sura_manipulation_actions task_priority_kinematic_control task_priority_kinematic_control_rqt
colcon test-result --verbose
```

## Backend `facing`

Se elige por goal (`execution_backend:=facing`) o como valor del servidor; el valor `pose` del launch común se trata como `facing`. Requisitos, que el servidor comprueba con el servicio `.../task_priority/check_facing_execution` sin cambiar la jerarquía:

- `task_priority_controller` activo.
- `EndEffectors_Facing` habilitada, sobre los mismos frames de herramienta y con `min_distance` no mayor que `planning.facing_min_distance`.
- `robot_nominal` habilitada y con prioridad por debajo de la de facing.
- `left_pose`, `right_pose` y `bimanual_joint_trajectory` deshabilitadas. Las tareas de pose de las puntas no intervienen ni en la simulación ni en la ejecución.

Secuencia, igual que el backend de pose de un brazo: comprobación, STPIK y, cada 10 ms con el reloj ROS, publicación de las dos referencias en el mismo progreso `s(t)`:

- `.../tasks/EndEffectors_Facing/facing_reference`, con feedforward `dref/ds · ds/dt`.
- `.../tasks/robot_nominal/reference`, con las articulaciones y velocidades planificadas y la máscara `activation` que anula las seis filas de base. Con la base anulada, `robot_nominal` solo sigue los brazos y no reapunta `base_pose`.

El servidor mide el facing con los estados articulares: aborta si sale de `execution.facing_path_tolerance` durante el recorrido, termina al entrar en `execution.facing_goal_tolerance` una vez acabado el tiempo y falla si no lo consigue en `execution.goal_time_tolerance`. En todos los casos deja ambas referencias con velocidad cero donde se detuvieron (en el objetivo si tuvo éxito).

```bash
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: left_pose, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: right_pose, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: bimanual_joint_trajectory, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: EndEffectors_Facing, enabled: true}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: robot_nominal, enabled: true}'
```

STPIK recibe en `check_cartesian_trajectory` el `facing_path`, la nominal (articulaciones, base y máscara) y `nominal_progress`, y simula la jerarquía completa. Las tolerancias `execution.facing_path_tolerance` y `execution.facing_goal_tolerance` siguen el orden de las filas; junto a `execution.goal_time_tolerance` y `facing.stpik_timeout` son parámetros del servidor. Con la ganancia por defecto de facing (0.2) la convergencia final puede requerir ampliar `execution.goal_time_tolerance`.

Parámetros por par (`pairs.<nombre>.`): `facing_task`, `nominal_task`, `facing_reference_topic`, `nominal_reference_topic`, `facing_stpik_action`, `facing_ready_service` y `disabled_tasks`.
