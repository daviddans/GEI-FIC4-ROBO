# Memoria Práctica 3 — Q-learning para seguimiento de línea

## 1. Introducción y objetivo

Esta práctica implementa un controlador en Python para el robot Khepera IV dentro del simulador Webots. El objetivo es que el robot aprenda de forma autónoma a seguir una línea negra dibujada sobre el suelo mediante **Q-learning**, un algoritmo de aprendizaje por refuerzo que no requiere un modelo del entorno. Además, se incorpora un módulo reactivo de evitación de obstáculos que no forma parte del proceso de aprendizaje.

El código se encuentra en `controllers/p3_controller/p3_controller.py`.

---

## 2. Discretización del problema

### Estados

El espacio de estados se reduce a **tres estados discretos** a partir de las lecturas de los cuatro sensores IR de suelo (lateral izquierdo, central izquierdo, central derecho, lateral derecho). La función `get_state` clasifica la situación del robot en función de dos umbrales definidos en el enunciado:

- **Valor > 750**: el sensor detecta negro (está sobre la línea).
- **Valor < 500**: el sensor detecta blanco (está fuera de la línea).

Los estados resultantes son:

| Estado | Significado | Condición |
|--------|-------------|-----------|
| **S1** | Abandonó la línea por la **izquierda** | central izq. > 750 (negro) **Y** lateral der. < 500 (blanco) |
| **S2** | Abandonó la línea por la **derecha** | central der. > 750 (negro) **Y** lateral izq. < 500 (blanco) |
| **S3** | Resto de casos | Sobre la línea correctamente o totalmente fuera |

Esta discretización es deliberadamente simple para mantener la Q-table pequeña (3 × 3) y permitir una convergencia rápida.

### Acciones

El agente dispone de **tres acciones**, ejecutadas durante un número fijo de pasos de simulación (`ACTION_STEPS = 10`) mediante la función `run_action`:

| Acción | Comportamiento |
|--------|----------------|
| **A1** | Girar a la derecha |
| **A2** | Girar a la izquierda |
| **A3** | Avanzar recto |

Una decisión de diseño importante fue **descartar los giros estáticos** (una rueda hacia adelante y la otra hacia atrás). En las primeras pruebas, este tipo de giro permitía al robot permanecer prácticamente en el mismo sitio mientras alternaba entre detectar y dejar de detectar la línea, generando un **bucle de recompensa parasitario** sin avanzar realmente. Para romper este comportamiento, las acciones de giro implementadas son **curvas dinámicas**: ambas ruedas avanzan, pero la interior lo hace a una velocidad menor (`CURVE_INNER = 1.0 rad/s`) mientras la exterior mantiene la velocidad de crucero (`CRUISE_SPEED = 4.0 rad/s`). Esto fuerza al robot a avanzar mientras corrige su trayectoria, evitando que se estanque en un mismo punto.

---

## 3. Función de recompensa

El enunciado impone una restricción clave: **la recompensa no puede codificarse a priori** (por ejemplo, no está permitido definir que "A1 en S1 siempre da recompensa positiva"). En su lugar, debe derivarse de la **experiencia** del robot.

La función `compute_reward` cumple este requisito comparando, para cada uno de los cuatro sensores de suelo, la lectura **antes** y **después** de ejecutar la acción:

- **Ganar detección de negro** (pasar de blanco a negro): recompensa positiva proporcional al peso del sensor.
- **Perder detección de negro** (pasar de negro a blanco): penalización proporcional al peso.
- **Mantener detección de negro**: recompensa reducida (+0.5×peso).
- **Seguir en blanco**: penalización reducida (-0.5×peso).

Los pesos asignados (`REWARD_WEIGHTS = [1.0, 2.0, 2.0, 1.0]`) dan el doble de importancia a los sensores **centrales** (índices 1 y 2), ya que mantener la línea bajo ellos es el objetivo principal del comportamiento. Esta función es la que guía al agente hacia políticas que centren el robot sobre la línea negra.

---

## 4. Actualización de la Q-table

El aprendizaje se realiza con la regla de actualización del caso **no-determinista** indicada en el enunciado:

```
Q_n(s,a) ← (1 - α_n) · Q_{n-1}(s,a) + α_n · [ r + γ · max_a' Q_{n-1}(s', a') ]
```

La función `update_q` implementa esta ecuación directamente. Los parámetros utilizados son:

| Parámetro | Valor | Significado |
|-----------|-------|-------------|
| **α (alpha)** | `1 / (1 + visits(s,a))` | Tasa de aprendizaje adaptativa. Disminuye a medida que una pareja (estado, acción) se visita más veces, estabilizando los valores en etapas avanzadas. |
| **γ (gamma)** | `0.5` | Factor de descuento. Equilibra la importancia entre la recompensa inmediata y el valor futuro esperado del siguiente estado. |
| **r** | Salida de `compute_reward` | Refuerzo escalar observado tras la transición. |
| **s'** | Salida de `get_state` sobre la lectura posterior | Estado resultante tras ejecutar la acción. |

La Q-table se representa como una matriz de dimensiones **3 estados × 3 acciones**, inicializada a ceros. Además de los valores Q, se mantiene una matriz paralela `visits` que cuenta cuántas veces se ha aplicado cada par (estado, acción), lo que permite calcular `α` de forma dinámica.

---

## 5. Control de exploración y explotación

El algoritmo debe equilibrar dos comportamientos: probar acciones nuevas (exploración) y aprovechar lo aprendido (explotación). Se utiliza una política **ε-greedy** implementada en `select_action`:

- Con probabilidad **ε**, se elige una acción completamente aleatoria.
- Con probabilidad **(1 - ε)**, se elige la acción con mayor valor Q para el estado actual (`argmax`).

El valor de ε no es constante: la función `epsilon` lo hace **decaer linealmente** desde 1 (exploración pura) hasta 0 (explotación pura) a lo largo de `EPS_DECAY_ITERS = 500` iteraciones. A partir de la iteración 500, el robot siempre elige la acción que considera óptima según la Q-table acumulada. Este decaimiento progresivo es esencial para que el agente primero explore el espacio de estados y acciones y luego refine la política aprendida.

---

## 6. Módulo de evitación de obstáculos

El entorno contiene un obstáculo rojo que el robot debe esquivar. Este comportamiento **no se aprende** mediante Q-learning; en su lugar, se implementa como un módulo reactivo en la función `avoid_obstacles`. El módulo consulta los tres sensores de proximidad frontales y, si alguno supera el umbral `OBSTACLE_THR = 300`, ejecuta una maniobra determinista:

1. **Retroceder** durante 5 pasos para alejarse del obstáculo.
2. **Girar sobre sí mismo** hacia la derecha hasta que los sensores frontales dejen de detectar el obstáculo (con un *timeout* de seguridad de 100 pasos).

Tras la maniobra, el estado del agente se reinicia leyendo de nuevo los sensores de suelo, de modo que el Q-learner retoma el control desde una situación actualizada.

---

## 7. Persistencia entre ejecuciones

Una característica práctica del controlador es que la Q-table y la matriz de visitas se **persisten** en el archivo `q_table.npz` (funciones `save_q` y `load_q`). Al arrancar la simulación, el robot intenta cargar los valores aprendidos en ejecuciones anteriores; si el archivo no existe, comienza desde cero. Esto permite entrenar de forma incremental y no perder el progreso al reiniciar Webots.

---

## 8. Bucle principal

El flujo de ejecución en el bloque principal es el siguiente:

1. Inicializar motores (modo velocidad) y habilitar sensores.
2. Cargar la Q-table persistida (si existe).
3. Para cada iteración del bucle:
   a. Comprobar obstáculos (`avoid_obstacles`). Si los hay, ejecutar maniobra y continuar.
   b. Calcular el estado actual (`get_state`).
   c. Calcular ε según la iteración (`epsilon`).
   d. Seleccionar acción (`select_action`).
   e. Ejecutar la acción durante `ACTION_STEPS` (`run_action`).
   f. Calcular recompensa (`compute_reward`).
   g. Calcular estado siguiente (`get_state`).
   h. Actualizar la Q-table (`update_q`).
   i. Guardar la Q-table (`save_q`).
   j. Imprimir información de depuración e incrementar contador.

La duración de `ACTION_STEPS = 10` (320 ms de simulación por acción) fue elegida como compromiso: suficiente para que el robot ejecute un movimiento significativo y las lecturas de suelo reflejen el cambio, pero no tanto como para que la inercia lo lleve demasiado lejos y pierda la línea irremediablemente.

---

## 9. Conclusiones

El controlador desarrollado cumple con los requisitos del enunciado: aprendizaje por refuerzo tabular con recompensa derivada de la experiencia, actualización no-determinista con tasa de aprendizaje adaptativa, y transición controlada de exploración a explotación. La decisión de sustituir los giros estáticos por curvas dinámicas fue clave para evitar comportamientos parasitarios y garantizar que el robot avanzara mientras aprendía. La persistencia de la Q-table permite un entrenamiento progresivo, y el módulo de evitación asegura la robustez frente al obstáculo del entorno.
