"""
Práctica 3 — Q-learning line follower para Khepera IV en Webots.

Estados (3): S1=abandona línea por izquierda, S2=por derecha, S3=resto.
Acciones (3): girar derecha, girar izquierda, avanzar recto.
Q-update no-determinista, alpha = 1/(1+visits), gamma = 0.5.
Epsilon decae linealmente de 1 a 0 en EPS_DECAY_ITERS iteraciones.
Evitación de obstáculos por IR frontal (no se aprende).
"""

import math
import numpy as np
from controller import Robot  # type: ignore


# --- Simulación y movimiento ---
TIME_STEP = 32             # periodo de simulación en ms (un robot.step(TIME_STEP) = 32 ms)
MAX_SPEED = 50             # tope de seguridad (rad/s) aplicado en set_motors() como clamp
CRUISE_SPEED = 8          # velocidad de la rueda exterior en cualquier acción (rad/s)
CURVE_INNER = 4.0          # velocidad de la rueda interior en giros (rad/s); junto a
                           # CRUISE_SPEED define el radio del arco. Ratio actual 10:4 =
                           # 2.5:1 → ~13° de rotación y ~2.8 cm de avance por acción de giro.
ACTION_STEPS_FWD = 10      # nº de ticks que dura A_FORWARD → 10·32 ms ≈ 320 ms de avance
ACTION_STEPS_TURN = 6      # nº de ticks que dura A_RIGHT/A_LEFT → 6·32 ms ≈ 192 ms de giro.
                           # Asimétrico para que FORWARD recorra más línea por iteración y
                           # el Q-learning prefiera el recto frente a "girar en el sitio".

# --- Sensores de suelo (umbrales del enunciado de la práctica) ---
# Las lecturas de los IR de suelo van en el rango aproximado 0..1000:
# valor BAJO = superficie oscura (línea), valor ALTO = superficie clara.
BLACK_THR = 500            # < BLACK_THR  → sensor sobre negro (sobre la línea)
WHITE_THR = 750            # > WHITE_THR  → sensor sobre blanco (fuera de la línea)

# --- Evitación de obstáculos (IR frontales de proximidad) ---
# Los IR de proximidad de Webots devuelven valores más altos cuanto más cerca
# está el objeto. Se usan los 3 sensores que miran hacia adelante:
#   - Central  (F):   apunta de frente. Cualquier lectura significativa es amenaza
#                     real → umbral BAJO para detectar a distancia.
#   - Laterales (FL, FR): apuntan ~45° a los lados. Detectan paredes en paralelo
#                     que no implican colisión → umbral ALTO, sólo cuasi-contacto.
OBSTACLE_THR_CENTER = 150  # dispara la maniobra si front-center ≥ 150 (detección anticipada)
OBSTACLE_THR_SIDE = 550    # dispara si front-left o front-right ≥ 550 (colisión inminente)
AVOID_TURN_SPEED = 14.0    # velocidad de las ruedas durante la maniobra (rad/s).
                           # No afecta al ángulo (lo controla el encoder), sólo a cuán
                           # rápido se completa para reanudar la marcha antes.
AVOID_TURN_DEG = 45        # rotación fija del robot por maniobra (grados, in-place)

# --- Geometría del Khepera IV (para convertir ángulo robot ↔ rotación de rueda) ---
WHEEL_RADIUS = 0.021       # radio de la rueda en metros
WHEELBASE = 0.1054         # distancia entre ruedas en metros (track width)

# --- Q-learning ---
GAMMA = 0.5                # factor de descuento γ del Q-update
EPS_DECAY_ITERS = 500      # iteraciones en las que ε decae linealmente de 1 a 0

# --- Mapeo de dispositivos en Webots ---
# Sensores de suelo del Khepera IV. El orden de este array fija qué índice usar
# en get_state() / compute_reward(); corresponde a los índices 8..11 del PDF:
#   0 = lateral izquierdo, 1 = central izquierdo,
#   2 = central derecho,   3 = lateral derecho.
GROUND_SENSOR_NAMES = [
    "ground left infrared sensor",
    "ground front left infrared sensor",
    "ground front right infrared sensor",
    "ground right infrared sensor",
]
# IR de proximidad que miran hacia adelante: índices 0=FL, 1=F (central), 2=FR.
FRONT_PROX_SENSOR_NAMES = [
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
]
# Encoders de las ruedas (posición acumulada en rad), usados por turn_in_place().
WHEEL_ENCODER_NAMES = ["left wheel sensor", "right wheel sensor"]

# --- Espacio de estados y acciones del MDP ---
S1, S2, S3 = 0, 1, 2                 # 3 estados: fuera-izq, fuera-dcha, resto
A_RIGHT, A_LEFT, A_FORWARD = 0, 1, 2 # 3 acciones
N_STATES, N_ACTIONS = 3, 3

# Pesos de cada sensor de suelo en la recompensa. Los centrales (índices 1 y 2)
# pesan el doble porque son los que indican que estamos correctamente sobre la línea.
REWARD_WEIGHTS = np.array([1.0, 2.0, 2.0, 1.0])


# --- Estado global del controlador (se inicializa en __main__) ---
robot = None
left_motor = None
right_motor = None
left_encoder = None
right_encoder = None
ground_sensors = []
front_sensors = []
Q = None
visits = None


# --- Funciones de bajo nivel sobre el robot ---

def set_motors(vl, vr):
    """Aplica velocidades a las ruedas, recortadas a ±MAX_SPEED.

    Args:
        vl: velocidad de la rueda izquierda en rad/s.
        vr: velocidad de la rueda derecha en rad/s.
    """
    vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
    vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)


def stop():
    """Detiene ambos motores."""
    set_motors(0.0, 0.0)


def read_ground():
    """Lee los 4 IR de suelo en el orden definido por GROUND_SENSOR_NAMES.

    Returns:
        np.ndarray[4] (float64) con [lateral_izq, central_izq, central_der, lateral_der].
        Valor BAJO = sobre negro/línea, valor ALTO = sobre blanco/suelo.
    """
    return np.array([s.getValue() for s in ground_sensors], dtype=np.float64)


def run_action(action_id):
    """Ejecuta una acción del MDP durante un número de ticks dependiente de ella.

    Mapeo:
      - A_FORWARD: ambas ruedas a CRUISE_SPEED  → recto, dura ACTION_STEPS_FWD ticks.
      - A_RIGHT:   izq=CRUISE_SPEED, dch=CURVE_INNER → arco a la derecha, ACTION_STEPS_TURN ticks.
      - A_LEFT:    izq=CURVE_INNER, dch=CRUISE_SPEED → arco a la izquierda, ACTION_STEPS_TURN ticks.

    Los giros son arcos abiertos (no pivote): la ratio CRUISE_SPEED/CURVE_INNER
    determina el radio. Forward dura más ticks que los giros para que la estrategia
    "avanzar recto" recorra más línea por iteración y resulte preferida por Q-learning.

    Args:
        action_id: A_RIGHT, A_LEFT o A_FORWARD.

    Returns:
        False si Webots cerró durante la acción (robot.step == -1), True en otro caso.
    """
    if action_id == A_FORWARD:
        set_motors(+CRUISE_SPEED, +CRUISE_SPEED)
        steps = ACTION_STEPS_FWD
    elif action_id == A_RIGHT:
        set_motors(+CRUISE_SPEED, +CURVE_INNER)
        steps = ACTION_STEPS_TURN
    elif action_id == A_LEFT:
        set_motors(+CURVE_INNER, +CRUISE_SPEED)
        steps = ACTION_STEPS_TURN
    else:
        stop()
        steps = 0

    for _ in range(steps):
        if robot.step(TIME_STEP) == -1:
            return False
    return True


# --- Funciones del problema de aprendizaje ---

def get_state(g):
    """Clasifica el estado discreto a partir de las 4 lecturas de suelo.

    Reglas del enunciado (umbrales BLACK_THR=500, WHITE_THR=750):
      S1 (abandona por la izquierda):  central_izq > WHITE  AND  lateral_der < BLACK
      S2 (abandona por la derecha):    central_der > WHITE  AND  lateral_izq < BLACK
      S3 (resto):                      cualquier otra combinación (sobre línea o fuera total)

    Args:
        g: np.ndarray[4] con [lateral_izq, central_izq, central_der, lateral_der].

    Returns:
        S1, S2 o S3 (constantes enteras 0/1/2).
    """
    g_lat_l, g_cen_l, g_cen_r, g_lat_r = g[0], g[1], g[2], g[3]
    if g_cen_l > WHITE_THR and g_lat_r < BLACK_THR:
        return S1
    if g_cen_r > WHITE_THR and g_lat_l < BLACK_THR:
        return S2
    return S3


def compute_reward(prev_g, curr_g):
    """Recompensa derivada de la experiencia (no codificada a priori, por sensor).

    Por cada uno de los 4 sensores de suelo, con peso w = REWARD_WEIGHTS[i]:
        blanco → negro (gana línea):    +w
        negro  → blanco (pierde línea): -w
        negro  → negro (mantiene):     +0.5·w
        blanco → blanco (sigue fuera): -0.5·w

    Los centrales (índices 1 y 2) pesan 2.0 y los laterales (0 y 3) pesan 1.0,
    así estar centrado sobre la línea recompensa más que rozarla con un lateral.

    Args:
        prev_g: lecturas de suelo antes de la acción (np.ndarray[4]).
        curr_g: lecturas de suelo tras la acción     (np.ndarray[4]).

    Returns:
        Suma de contribuciones de los 4 sensores (float).
    """
    total = 0.0
    for i in range(4):
        was_black = prev_g[i] < BLACK_THR
        is_black = curr_g[i] < BLACK_THR
        w = REWARD_WEIGHTS[i]
        if is_black and not was_black:
            total += w
        elif not is_black and was_black:
            total -= w
        elif is_black and was_black:
            total += 0.5 * w
        else:
            total -= 0.5 * w
    return float(total)


def epsilon(iteration):
    """Decaimiento lineal de ε desde 1 hasta 0 en EPS_DECAY_ITERS iteraciones.

    Iteración 0 → ε=1 (acción aleatoria pura, exploración).
    Iteración EPS_DECAY_ITERS → ε=0 (siempre argmax, explotación pura).
    """
    return max(0.0, 1.0 - iteration / EPS_DECAY_ITERS)


def select_action(s, eps):
    """Política ε-greedy sobre la Q-table global.

    Con probabilidad ε elige una acción uniforme aleatoria entre las N_ACTIONS,
    en caso contrario elige argmax_a Q[s, a].

    Args:
        s:   estado actual (S1, S2 o S3).
        eps: probabilidad de exploración en [0, 1].

    Returns:
        Índice entero de la acción elegida (A_RIGHT, A_LEFT o A_FORWARD).
    """
    if np.random.random() < eps:
        return int(np.random.randint(N_ACTIONS))
    return int(np.argmax(Q[s]))


def update_q(s, a, r, s_next):
    """Aplica la regla de actualización Q-learning no-determinista.

    Ecuación del enunciado (caso no-determinista):
        Q_n(s,a) <- (1 - α_n)·Q_{n-1}(s,a) + α_n·(r + γ·max_a' Q_{n-1}(s',a'))
        α_n = 1 / (1 + visits_n(s,a))

    γ = GAMMA (factor de descuento, constante del módulo).
    El contador visits[s,a] se incrementa ANTES de calcular α, por eso el
    primer paso por (s,a) ya usa α = 1/2 en lugar de α = 1.

    Args:
        s:      estado actual antes de ejecutar la acción.
        a:      acción ejecutada.
        r:      recompensa observada al pasar de s a s_next.
        s_next: estado tras la acción.
    """
    global Q, visits
    visits[s, a] += 1
    alpha = 1.0 / (1 + visits[s, a])
    target = r + GAMMA * float(np.max(Q[s_next]))
    Q[s, a] = (1 - alpha) * Q[s, a] + alpha * target


# --- Módulo de evitación (no se aprende) ---

def turn_in_place(deg, sign):
    """Gira el robot un ángulo fijo midiendo con los encoders de las ruedas.

    Para un giro in-place simétrico cada rueda recorre, en sentidos opuestos:
        Δφ_rueda = θ_robot · (WHEELBASE / 2) / WHEEL_RADIUS  [rad]

    El bucle avanza ticks hasta que cualquiera de las ruedas alcanza la rotación
    objetivo (usar max evita que un encoder con desfase prolongue el giro). La
    velocidad AVOID_TURN_SPEED sólo afecta a cuánto tarda en completarse, no al
    ángulo final, que queda fijado por el chequeo de encoders.

    Args:
        deg:  ángulo a girar en grados (siempre positivo).
        sign: +1 → giro a la derecha (rueda izq adelante, dcha atrás),
              -1 → giro a la izquierda.

    Returns:
        False si Webots cerró durante el giro (robot.step == -1), True en otro caso.
    """
    angle_rad = math.radians(deg)
    target_wheel = angle_rad * (WHEELBASE / 2.0) / WHEEL_RADIUS

    left_start = left_encoder.getValue()
    right_start = right_encoder.getValue()

    set_motors(+sign * AVOID_TURN_SPEED, -sign * AVOID_TURN_SPEED)
    while True:
        if robot.step(TIME_STEP) == -1:
            return False
        left_d = abs(left_encoder.getValue() - left_start)
        right_d = abs(right_encoder.getValue() - right_start)
        if max(left_d, right_d) >= target_wheel:
            break
    stop()
    return True


def avoid_obstacles():
    """Si hay obstáculo frontal, gira AVOID_TURN_DEG grados hacia el lado más despejado.

    Reglas de detección (umbrales asimétricos definidos arriba):
      - Central (F)     ≥ OBSTACLE_THR_CENTER → dispara (detección anticipada).
      - Lateral (FL/FR) ≥ OBSTACLE_THR_SIDE   → dispara (cuasi-colisión).
      Si ninguna condición se cumple, no se hace nada.

    Elección del sentido del giro (cuando ya se ha disparado):
      - FL ≥ FR → obstáculo más a la izquierda → giro a la DERECHA (sign=+1).
      - FL <  FR → obstáculo más a la derecha   → giro a la IZQUIERDA (sign=-1).

    No retrocede ni reintenta: tras el giro fijo devuelve el control al bucle
    Q-learning, que aprenderá la trayectoria adecuada para no volver a chocar.

    Returns:
        True si ejecutó la maniobra (un giro), False si no había obstáculo.
    """
    front_l = front_sensors[0].getValue()
    front_c = front_sensors[1].getValue()
    front_r = front_sensors[2].getValue()

    center_hit = front_c >= OBSTACLE_THR_CENTER
    side_hit = front_l >= OBSTACLE_THR_SIDE or front_r >= OBSTACLE_THR_SIDE
    if not (center_hit or side_hit):
        return False

    sign = +1 if front_l >= front_r else -1
    direction_str = "derecha" if sign > 0 else "izquierda"
    trigger = "central" if center_hit else "lateral"
    print(f"[AVOID] FL/F/FR={front_l:.0f}/{front_c:.0f}/{front_r:.0f} ({trigger}) -> giro {AVOID_TURN_DEG}° {direction_str}")

    turn_in_place(AVOID_TURN_DEG, sign)
    return True


# --- Bucle principal ---

if __name__ == "__main__":
    # Inicialización del robot y dispositivos
    robot = Robot()

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    for m in (left_motor, right_motor):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    left_encoder = robot.getDevice(WHEEL_ENCODER_NAMES[0])
    right_encoder = robot.getDevice(WHEEL_ENCODER_NAMES[1])
    left_encoder.enable(TIME_STEP)
    right_encoder.enable(TIME_STEP)

    for name in GROUND_SENSOR_NAMES:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        ground_sensors.append(s)

    for name in FRONT_PROX_SENSOR_NAMES:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        front_sensors.append(s)

    # Primer step para que las lecturas estén disponibles
    robot.step(TIME_STEP)

    # Q-table y contador de visitas inicializados a ceros en cada arranque.
    Q = np.zeros((N_STATES, N_ACTIONS), dtype=np.float64)
    visits = np.zeros((N_STATES, N_ACTIONS), dtype=np.int64)

    iteration = 0
    prev_g = read_ground()
    s = get_state(prev_g)

    while robot.step(TIME_STEP) != -1:
        if avoid_obstacles():
            prev_g = read_ground()
            s = get_state(prev_g)
            continue

        eps = epsilon(iteration)
        a = select_action(s, eps)

        prev_g = read_ground()
        if not run_action(a):
            break
        curr_g = read_ground()

        r = compute_reward(prev_g, curr_g)
        s_next = get_state(curr_g)
        update_q(s, a, r, s_next)

        print(f"[iter={iteration:4d}] s={s} a={a} r={r:+.2f} s'={s_next} eps={eps:.2f}")

        s = s_next
        iteration += 1
