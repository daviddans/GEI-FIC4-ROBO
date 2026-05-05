# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Resumen del proyecto

Práctica 3 de Robótica: controlador en **Python** para el robot **Khepera IV** en **Webots** que aprende a seguir una línea negra sobre el suelo mediante **Q-learning tabular**. Un módulo no-aprendido de evitación de obstáculos usa los sensores IR laterales.

Enunciado: `Aprendizaje_por_refuerzo.pdf` (raíz del repo).
Código a desarrollar: `controller/p3_controller.py` (actualmente vacío).
Referencia de estilo: `controller/p2_controller.py` (práctica 2, vigilancia con A*).

> Nota: existe un `CLAUDE.md` en el directorio padre `ROBO/` que documenta una práctica distinta (robot LEGO EV3 en ROBOTC, arquitectura de subsunción). **Esa documentación NO aplica aquí** — esta práctica es Webots + Python + Khepera4 + Q-learning.

## Build & run

No hay toolchain ni tests. El flujo es:

1. Abrir `worlds/khepera4_lines.wbt` en Webots.
2. En el nodo `Khepera4`, asignar el campo `controller` a `p3_controller` (en el `.wbt` actual no está fijado).
3. Webots ejecuta `controller/p3_controller.py` automáticamente. El `venv/` del repo provee `numpy`/`opencv-python`.

`Robot()` y `Supervisor()` se importan desde el módulo `controller` que Webots inyecta en runtime — sólo funciona cuando el script se lanza desde Webots, no como Python suelto.

## Hardware del Khepera IV (Webots)

| Dispositivo | Nombre Webots | Notas |
|---|---|---|
| Motores | `left wheel motor`, `right wheel motor` | velocidad rad/s, MAX_SPEED ≈ 47.6 |
| Encoders | `left wheel sensor`, `right wheel sensor` | posición en rad |
| IR proximidad (8) | `front`, `front right`, `right`, `rear right`, `rear`, `rear left`, `left`, `front left` `infrared sensor` | índices 0..7 — para evitación de obstáculos |
| IR suelo (4) | `ground left infrared sensor`, `ground front left ...`, `ground front right ...`, `ground right ...` | índices 8..11 en el array — para seguir línea |
| Cámara | `camera` | no se usa en P3 |

**Convención de índices del PDF (líneas 8–11):**
- `8` = suelo lateral izquierdo
- `9` = suelo central izquierdo
- `10` = suelo central derecho
- `11` = suelo lateral derecho

Umbrales del enunciado: valor `> 750` → detecta negro (sobre línea); `< 500` → blanco (fuera de línea).

## Algoritmo de Q-learning (especificación del enunciado)

**Estados (3):**
- `S1`: el robot abandonó la línea por la **izquierda** → `ground[9] > 750 AND ground[11] < 500`
- `S2`: el robot abandonó la línea por la **derecha** → `ground[10] > 750 AND ground[8] < 500`
- `S3`: resto de casos (sobre línea o totalmente fuera)

**Acciones (3):** `A1` girar derecha, `A2` girar izquierda, `A3` avanzar recto.

**Recompensa:** debe derivarse de la **experiencia**, no codificarse a priori (regla explícita del enunciado: "no vale poner que A1 en S1 da refuerzo fantástico"). Forma sugerida: comparar lectura previa vs posterior de los sensores de suelo. Si pasó de no-detectar-negro a detectarlo → positivo; si sigue sin detectar → negativo. Matizar según centrales (9, 10) vs laterales (8, 11).

**Actualización (caso no-determinista):**
```
Q_n(s,a) ← (1 - α_n) · Q_{n-1}(s,a) + α_n · {r + γ · max_a' Q_{n-1}(s', a')}
α_n = 1 / (1 + visits_n(s,a))     # o constante 0.5
γ = 0.5
```

**Política exploración → explotación:** ε empieza en 1 (acción aleatoria) y decae linealmente hasta ~0 sobre **~500 iteraciones** (en la iteración 500 siempre se elige `argmax Q`).

**Módulo de evitación (NO se aprende):** se ejecuta tras cada acción Q-learning. Usa los IR de proximidad frontales para esquivar el obstáculo rojo del mundo.

## Mundo

`worlds/khepera4_lines.wbt`: arena rectangular con textura `textures/line.png` (la línea a seguir) y un `Solid` rojo en `(0.609, 0.05, 0.062)` como obstáculo. Sin paredes salvo el contorno.

## Recursos externos relevantes

- **TFG del usuario** en `~/Documents/Fic4/TFG/`: trata exactamente sobre Q-learning y Deep Q-Networks (`Qlearn-test.py` Q-learning tabular sobre CartPole; `DQLearn-test/` DQN). Útil como referencia conceptual y para no duplicar trabajo entre práctica y TFG.
- **`controller/p2_controller.py`**: misma plataforma (Khepera4 + Webots), arquitectura limpia con clases `RobotAPI` / `Controller` / `Director` / `Map`. Reutilizar el patrón de `RobotAPI` (encapsular sensores/motores, exponer métodos de alto nivel) es la guía estilística para P3.

## Convenciones de estilo (heredadas de p2)

- Una clase `RobotAPI` que encapsula `Robot()` y expone métodos de alto nivel (`turn_right()`, `move_forward()`, `read_ground()`, …).
- `TIME_STEP = 32`, esperas de movimiento mediante encoders con tolerancia ~0.01 rad.
- Bucle principal en `if __name__ == "__main__":` con `while robot.step() != -1`.
- Comentarios y prints en castellano/galego mezclados — está bien, no homogeneizar salvo petición.
