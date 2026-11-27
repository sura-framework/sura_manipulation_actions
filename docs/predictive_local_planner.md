# Predicción local por progreso en ROS 2 Humble

En RQT puedes marcar **Use Predictive Local Planner** junto a **Optimize whole trajectory** para activar la predicción en un objetivo. El campo de la acción es `use_predictive_local_planner: true`. Con el launch en su valor por defecto, la casilla decide por objetivo. El argumento `predictive_local_planner:=true` activa la predicción para todos los objetivos con optimización de trayectoria completa.

El modo predictivo se aplica a PlanCartesianTrajectory con backend pose, use_stpik y optimize_trajectory_nominal activados. El progreso geométrico va de 0 a 1. El resultado de preparación asocia nominal_progress a los nudos de base y brazos. La referencia cartesiana usa traslación lineal y el arco SLERP del plan original. Un segmento local tiene tiempos propios y un progreso por nudo, que puede disminuir o repetirse.

El ejecutor usa la acción cancelable CheckCartesianTrajectory. Un goal con prediction.global_progress solicita búsqueda local. Un goal ordinario conserva el checker anterior. El worker captura las tareas efectivas, actualiza cinemática y resuelve el prefijo anterior a robot_nominal. Los candidatos usan la velocidad particular y el espacio nulo en coordenadas escaladas. El ganador se reproduce con la jerarquía completa, los holds sincronizados y la interpolación de ejecución. El resultado incluye estados por paso, rango, márgenes, coste y duración de cálculo. El token liga el resultado a la configuración capturada. El ejecutor verifica el empalme, la medida de punta y la escena antes de adoptar las referencias.

## Simulación

Desde la raíz del workspace, compile y cargue ambos paquetes. Reinicie los consumidores de las acciones al regenerar sus tipos.

```bash
cd /home/salva/sura_ws_new_new
colcon build --symlink-install --packages-select sura_manipulation_actions task_priority_kinematic_control
source install/setup.bash
ros2 launch cirtesub_stonefish cirtesub_cirtesu_zip_tie_pbd.launch.py
```

Con el bringup dual de simulación, el controlador de prioridad y MoveIt disponibles, lance el servidor cartesiano. El relay lleva la pose relativa publicada por Stonefish a la entrada de offset de right_pose.

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py execution_backend:=pose predictive_local_planner:=true zip_tie_tip_offset:=true launch_joint:=false launch_bimanual_joint:=false launch_bimanual_cartesian:=false launch_planning_markers:=true
```

Active right_pose y robot_nominal en la jerarquía. Con la pose de la brida ya disponible, active tasks.right_pose.use_offset en el controlador. Configure y active el nodo plan_cartesian_trajectory_lifecycle_action_server. Envíe un goal para alpha_right con el objetivo expresado para la punta y con optimize_trajectory_nominal y use_stpik activados. La ejecución publica referencias en las tareas existentes. local_replan_required en el feedback y el resultado fallido indican que el supervisor debe intervenir.

El perfil single_arm_cartesian_planner.yaml contiene los parámetros local_planner. horizon y period están en segundos, retreat_window es una fracción geométrica. position_scale y angle_scale normalizan metros y radianes. progress_rate_scale igual a cero congela la referencia. explore igual a false ejecuta la misma predicción nominal sin la búsqueda adicional. La comparación de coste usa el mismo horizonte. El presupuesto vencido se distingue del agotamiento de candidatos.

## Pruebas y reproducción

```bash
cd /home/salva/sura_ws_new_new
source install/setup.bash
colcon test --packages-select sura_manipulation_actions task_priority_kinematic_control
colcon test-result --verbose
SURA_LOCAL_PLANNER_CSV=/tmp/local_steps.csv build/task_priority_kinematic_control/test_stpik_check --gtest_filter='StpikTest.Local*'
python3 src_new_new/src/task_priority_kinematic_control/test/replay_local_prediction.py /tmp/local_replay
```

El ensayo C++ contrasta las mismas perturbaciones sobre copias independientes de la misma captura. El script inicia un runtime aislado con el modelo dual de SURA y reproduce cinco observaciones de la punta procedentes de Stonefish. trials.csv recoge resultado y coste temporal, steps.csv contiene las muestras cinemáticas y timing.json resume el periodo observado en la salida del runtime con y sin peticiones concurrentes. Los ensayos usan escenas congeladas por predicción. Su alcance es cinemático.

## Mediciones de esta revisión

Los ensayos emparejados con KDL conservaron el segmento nominal en el caso factible. Con una perturbación de 0,06 m en la articulación prismática del banco y progreso fijo en 0,40, el error normalizado tras 0,2 s fue 0,600 con STPIK nominal y 0,335 con exploración. El rango pasó de 6 en el caso libre a 7 con evitación activa. Ambas estrategias rechazaron el caso que comenzaba debajo de la separación mínima. [Muestras cinemáticas](local_planner_results/kinematic_steps.csv)

La reproducción de 14 DoF utilizó cinco offsets reales del simulador en dos regímenes de progreso y con ambas estrategias. La postura capturada violó el plano body_guard desde el primer estado. La separación calculada fue −0,1886 m para el brazo izquierdo con un mínimo configurado de 0,05 m. Las veinte solicitudes se rechazaron antes de integrar. La comparación con la postura nominal del YAML también detectó una violación inicial. Estos tiempos corresponden a ese rechazo temprano. [Ensayos con postura capturada](local_planner_results/captured_trials.csv) · [Tiempos](local_planner_results/captured_timing.json) · [Ensayos con postura nominal](local_planner_results/nominal_trials.csv)

El simulador proporcionó las observaciones. La reproducción del runtime se hizo con salidas aisladas y estado de entrada fijado para cada pareja. Queda por medir la recuperación con el simulador en lazo cerrado partiendo de una postura que cumpla la separación.

## Visualización en RViz

Usa el marco de planificación como **Fixed Frame** o uno conectado mediante TF. Configura los displays con durabilidad **Transient Local**.

- **MarkerArray** en `/cirtesub/manipulation/local_path` muestra los candidatos de la última búsqueda. Cada línea usa las poses calculadas por cinemática directa desde el estado capturado hasta el empalme con el global. El cian indica que la pose alcanzó la referencia dentro de las tolerancias de empalme del controlador, `stpik.rejoin_position_tolerance` (0,005 m por defecto) y `stpik.rejoin_orientation_tolerance` (0,01 rad por defecto). El naranja indica un tramo parcial, por ejemplo si la simulación se rechaza o se agota el presupuesto de recuperación. Un candidato que permanece dentro de las tolerancias se muestra como un punto.
- **Path** en `/cirtesub/manipulation/local_predicted_path` muestra únicamente las muestras del segmento que aceptó el ejecutor. La pose corresponde a la punta con el offset activo y a la herramienta con el offset desactivado.
- **Path** en `/cirtesub/manipulation/local_base_path` muestra la referencia de base del segmento aceptado.

La simulación de recuperación usa un presupuesto aparte de la búsqueda, con el mismo valor que `local_planner.budget`. Las visualizaciones se limpian al iniciar otra ejecución de pose.
