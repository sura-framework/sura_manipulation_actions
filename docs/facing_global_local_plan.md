# Plan: facing con planner global, optimización completa y planner local

Estado: fases 1 (secundaria en `robot_nominal` con máscara de base) y 2 (optimizador global con facing) implementadas; ver `bimanual_cartesian_planner.md`. Fase 3 (dirección y roll antes que distancia) implementada. Pendiente: planner local. Retoma el planner local que quedó fuera del plan inicial.

## Objetivo

Aplicar al facing bimanual la cadena del cartesian planner de un brazo:

```
global (camino + postura por punto) → optimización de toda la trayectoria → ejecución con planner local
```

Primaria: `EndEffectors_Facing` siguiendo `facing_path` por progreso `s`. Secundaria: referencia articular de los 8 joints, enviada por streaming. Base fija.

## Lo que ya existe

| Pieza | Dónde | Qué hace | Atado a |
| --- | --- | --- | --- |
| Optimizador | `task_priority_kinematic_control/src/core/nominal_trajectory.cpp` | Rollout STPIK como semilla → SQP sobre todos los puntos → **búsqueda global por capas con programación dinámica** si falla → SQP → validación STPIK | Camino de pose de una punta, 6 DOF de base, `robot_nominal` |
| Predictor local | `.../src/core/local_prediction.cpp` | Simula continuaciones y perturbaciones en el espacio nulo y devuelve el mejor segmento | Tarea Pose, `robot_nominal`, camino de pose |
| Ejecutor local | `sura_manipulation_actions/src/cartesian_pose_execution.cpp` (`executePredictive`) | Pide segmentos, comprueba empalme y seguimiento, publica referencias | Un brazo, medida de la punta |

**El planner global no hay que escribirlo:** el optimizador ya lo incluye. Basta con que acepte el facing como tarea primaria. Esto sustituye al grafo nuevo que propuse antes en el chat.

## Decisiones previas

| Decisión | Recomendación | Motivo |
| --- | --- | --- |
| Tarea secundaria | `robot_nominal`, con la base desactivada | Optimizador y predictor ya dependen de ella. Sustituye a `bimanual_joint_trajectory` en este modo |
| Base | Mantenida por `base_pose`, por encima de facing; el planner no la mueve | El facing no tiene columnas de base: sin esa tarea, el espacio nulo incluiría la base |
| Manipulabilidad (25, 26) | Desactivada durante la ejecución | Está por encima de `robot_nominal` (27) y compite por los 2 grados de libertad libres |
| Planner actual | Se mantiene como modo rápido | El modo optimizado no depende de su IK punto a punto |

La activación de `robot_nominal` es compartida con el modo de un brazo: cambiarla afecta a ambos. Hay que decidir si se cambia por servicio antes de ejecutar o si el servidor solo la comprueba (recomendado, como hasta ahora).

## Fases

### 0. Base de partida
- Corregir la validación entre puntos del planner actual (`Timed trajectory remains invalid after bounded refinement`).
- Capturar 3 o 4 posturas y objetivos reales como casos de prueba, incluidos los que hoy fallan.
- **Hecho cuando:** cada caso está clasificado como alcanzable o no, muestreando posturas a lo largo de `s`.

### 1. Optimizador con facing (global + trayectoria completa)
- Sacar de `nominal_trajectory.cpp` una interfaz de tarea primaria: muestrear la referencia en `s`, error y jacobiano de 6 filas, banda de tolerancia. Implementaciones: pose (la actual) y facing (`facing_geometry.hpp`). Afecta a `evaluate`, `project`, `rollout` y la validación final.
- Base fija: hoy exige los 6 DOF activos y las capas perturban x, y, z y yaw. Congelar esas columnas y no muestrearlas.
- Tiempos: `facing_path` no los tiene. Estimar la duración con los límites de velocidad y los cambios de distancia y ángulo, y dilatar si la validación la rechaza.
- **Hecho cuando:** los tests actuales de pose pasan sin cambios y los casos alcanzables de la fase 0 dan una trayectoria validada.

### 2. Conexión con el planner bimanual
- `PrepareCartesianPosture.action`: aceptar `facing_path`.
- `PlanBimanualCartesianTrajectory.action`: campo `optimize_trajectory_nominal`. El servidor construye `facing_path` desde el estado medido y pide la optimización al controlador.
- Resultado: la misma salida de hoy (`joint_trajectory`, `nominal_progress`, `facing_path`).
- STPIK de facing: admitir `robot_nominal` como secundaria.
- Pestaña RQT: casilla de optimización y mensajes de progreso.
- **Hecho cuando:** se puede planificar desde RQT y ver el resultado, sin ejecutar.

### 3. Ejecución por streaming, todavía sin local
- En cada tick publicar `FacingReference` y la referencia de `robot_nominal` con el mismo `s`.
- Ampliar `check_facing_execution`: `robot_nominal` activa y sin base, `base_pose` activa, manipulabilidad y `bimanual_joint_trajectory` desactivadas.
- Al cancelar o fallar: mantener la postura medida con velocidades cero.
- **Hecho cuando:** en simulación se ejecutan los casos y se registran errores de facing y articulares. Comparar con el backend `facing` actual.

### 4. Planner local
- `local_prediction.cpp`: usar la interfaz de primaria de la fase 1; prefijo de la jerarquía hasta `robot_nominal`; candidatos solo en columnas de brazos.
- `StpikCheck::Rollout`: hoy su referencia es una pose; generalizarla y soportarla en la simulación de facing.
- Ejecutor bimanual equivalente a `executePredictive`: medir facing y progreso confirmado, verificar empalme y escena, adoptar segmentos, `local_replan_required`.
- `use_predictive_local_planner` en la acción bimanual y casilla en RQT.
- **Hecho cuando:** ante una perturbación el local se aparta o retrocede en `s` y recupera el facing, comparado con `explore: false`.

### 5. Validación y documentación
- Ensayos en lazo cerrado en simulación: factible, cerca de límites, cerca de autocolisión y no factible.
- Tiempos de optimización y de predicción frente al periodo.
- Actualizar `bimanual_cartesian_planner.md` y `predictive_local_planner.md`.

## Riesgos

- **Espacio nulo de 2 grados de libertad** como máximo, menos con límites o autocolisión activos. El local no puede cambiar de rama de posturas.
- **Alcanzabilidad:** si en algún `s` no existe postura válida, ninguna fase lo arregla; hay que cambiar el camino (encararse antes en espacio articular).
- **Regresiones en el modo de un brazo**, porque se generaliza código que usa. Cada fase debe pasar sus tests.
- **Coste:** las fases 1 y 4 son las grandes; no hay estimación medida.

## Compilación

`sura_manipulation_actions`, `task_priority_kinematic_control` y `task_priority_kinematic_control_rqt`, desde `/home/salva/sura_ws_new_new`, con `--symlink-install`, `--packages-select` y poco paralelismo.

## Alcance de esta revisión

Leídos: la interfaz y las partes de evaluación, proyección y búsqueda del optimizador; el inicio del predictor y sus dependencias; el inicio de `executePredictive`; los mensajes locales. No se han revisado a fondo `posture_action.cpp` ni el resto del predictor y del ejecutor, así que el detalle de las fases 2 a 4 puede cambiar.
