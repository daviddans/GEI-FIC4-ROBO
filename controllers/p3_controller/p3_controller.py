"""
Práctica 3 — Q-learning line follower para Khepera IV en Webots.

Estados (3): S1=abandona línea por izquierda, S2=por derecha, S3=resto.
Acciones (3): girar derecha, girar izquierda, avanzar recto.
Q-update no-determinista, alpha = 1/(1+visits), gamma = 0.5.
Epsilon decae linealmente de 1 a 0 en EPS_DECAY_ITERS iteraciones.
Evitación de obstáculos por IR frontal (no se aprende).
"""

import os
import numpy as np
from controller import Robot  # type: ignore


# --- Constantes generales ---
TIME_STEP = 32
MAX_SPEED = 47.6
CRUISE_SPEED = 4.0
TURN_SPEED = 2.0
CURVE_INNER = 1.0   # velocidad de la rueda interior en curva (siempre > 0)
ACTION_STEPS = 10

# Umbrales del enunciado
BLACK_THR = 500    # < BLACK_THR -> negro (sobre línea)
WHITE_THR = 750    # > WHITE_THR -> blanco (fuera de línea)
OBSTACLE_THR = 300

# Q-learning
GAMMA = 0.5
EPS_DECAY_ITERS = 500
Q_FILE = "q_table.npz"

# Sensores Khepera4 (orden lateral_izq, central_izq, central_der, lateral_der;
# corresponde a los índices 8..11 del PDF).
GROUND_SENSOR_NAMES = [
    "ground left infrared sensor",
    "ground front left infrared sensor",
    "ground front right infrared sensor",
    "ground right infrared sensor",
]
FRONT_PROX_SENSOR_NAMES = [
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
]

# Estados y acciones
S1, S2, S3 = 0, 1, 2
A_RIGHT, A_LEFT, A_FORWARD = 0, 1, 2
N_STATES, N_ACTIONS = 3, 3

# Pesos de la recompensa por sensor (centrales pesan más)
REWARD_WEIGHTS = np.array([1.0, 2.0, 2.0, 1.0])


# --- Estado global del controlador (se inicializa en __main__) ---
robot = None
left_motor = None
right_motor = None
ground_sensors = []
front_sensors = []
Q = None
visits = None


# --- Funciones de bajo nivel sobre el robot ---

def set_motors(vl, vr):
    """Aplica velocidades (rad/s) clampadas a [-MAX_SPEED, +MAX_SPEED].

    Args:
        vl: velocidad rueda izquierda.
        vr: velocidad rueda derecha.
    """
    vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
    vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)


def stop():
    """Detiene ambos motores."""
    set_motors(0.0, 0.0)


def read_ground():
    """Devuelve las 4 lecturas IR de suelo como np.ndarray[4] de float64.

    Orden: [lateral_izq, central_izq, central_der, lateral_der].
    """
    return np.array([s.getValue() for s in ground_sensors], dtype=np.float64)


def read_front_proximity():
    """Devuelve la lectura máxima de los 3 IR frontales (mayor = más cerca)."""
    return max(s.getValue() for s in front_sensors)


def run_action(action_id):
    """Ejecuta una acción durante ACTION_STEPS ticks de simulación.

    Args:
        action_id: A_RIGHT, A_LEFT o A_FORWARD.

    Returns:
        False si Webots cerró durante la acción (robot.step == -1), True en otro caso.
    """
    if action_id == A_FORWARD:
        set_motors(+CRUISE_SPEED, +CRUISE_SPEED)
    elif action_id == A_RIGHT:
        set_motors(+CRUISE_SPEED, +CURVE_INNER)
    elif action_id == A_LEFT:
        set_motors(+CURVE_INNER, +CRUISE_SPEED)
    else:
        stop()

    for _ in range(ACTION_STEPS):
        if robot.step(TIME_STEP) == -1:
            return False
    return True


# --- Funciones del problema de aprendizaje ---

def get_state(g):
    """Clasifica el estado a partir de las 4 lecturas de suelo.

    Args:
        g: np.ndarray[4] con [lateral_izq, central_izq, central_der, lateral_der].

    Returns:
        S1 si abandonó por la izquierda, S2 por la derecha, S3 en cualquier otro caso.
    """
    g_lat_l, g_cen_l, g_cen_r, g_lat_r = g[0], g[1], g[2], g[3]
    if g_cen_l > WHITE_THR and g_lat_r < BLACK_THR:
        return S1
    if g_cen_r > WHITE_THR and g_lat_l < BLACK_THR:
        return S2
    return S3


def compute_reward(prev_g, curr_g):
    """Recompensa derivada de la experiencia (no codificada a priori).

    Compara lectura previa vs posterior por sensor: ganar negro = +w,
    perderlo = -w, mantenerlo = +0.5·w, seguir en blanco = -0.5·w.
    Los sensores centrales (índices 1 y 2) pesan el doble (REWARD_WEIGHTS).

    Args:
        prev_g: lecturas de suelo antes de la acción.
        curr_g: lecturas de suelo tras la acción.

    Returns:
        Recompensa total (float).
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
    """Decaimiento lineal de ε desde 1 hasta 0 en EPS_DECAY_ITERS iteraciones."""
    return max(0.0, 1.0 - iteration / EPS_DECAY_ITERS)


def select_action(s, eps):
    """Política ε-greedy sobre la Q-table global.

    Args:
        s:   estado actual.
        eps: probabilidad de elegir acción aleatoria (exploración).

    Returns:
        Índice de la acción elegida.
    """
    if np.random.random() < eps:
        return int(np.random.randint(N_ACTIONS))
    return int(np.argmax(Q[s]))


def update_q(s, a, r, s_next):
    """Aplica la regla de actualización Q-learning no-determinista.

    Ecuación del enunciado (caso no-determinista):
        Q_n(s,a) <- (1 - α_n)·Q_{n-1}(s,a) + α_n·(r + γ·max_a' Q_{n-1}(s',a'))
        α_n = 1 / (1 + visits_n(s,a))

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


def save_q():
    """Persiste Q y visits a Q_FILE."""
    np.savez(Q_FILE, Q=Q, visits=visits)


def load_q():
    """Carga Q y visits desde Q_FILE; si no existe, los inicializa a ceros."""
    global Q, visits
    if os.path.exists(Q_FILE):
        data = np.load(Q_FILE)
        Q = data["Q"].astype(np.float64)
        visits = data["visits"].astype(np.int64)
        print(f"[Q] Cargada Q-table de {Q_FILE}")
    else:
        Q = np.zeros((N_STATES, N_ACTIONS), dtype=np.float64)
        visits = np.zeros((N_STATES, N_ACTIONS), dtype=np.int64)
        print("[Q] Empezando con Q-table en ceros")


# --- Módulo de evitación (no se aprende) ---

def avoid_obstacles():
    """Si hay obstáculo frontal cerca, retrocede y gira hasta despejarlo.

    Returns:
        True si se ejecutó maniobra de evitación, False si no había obstáculo.
    """
    proximity = read_front_proximity()
    if proximity < OBSTACLE_THR:
        return False

    print(f"[AVOID] Obstáculo detectado (prox={proximity:.0f}), retrocediendo y girando...")

    # Retroceder primero para alejarse del obstáculo
    set_motors(-CRUISE_SPEED, -CRUISE_SPEED)
    for _ in range(5):
        if robot.step(TIME_STEP) == -1:
            return True

    # Luego girar a la derecha hasta que el camino esté libre
    set_motors(+TURN_SPEED, -TURN_SPEED)
    max_steps = 100  # Safety timeout: ~3.2 segundos de giro
    steps_spun = 0
    while read_front_proximity() >= OBSTACLE_THR and steps_spun < max_steps:
        if robot.step(TIME_STEP) == -1:
            return True
        steps_spun += 1

    if steps_spun >= max_steps:
        print("[AVOID] ⚠ Timeout: no se pudo despejar el obstáculo tras girar 100 pasos")

    stop()
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

    load_q()

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
        save_q()

        print(f"[iter={iteration:4d}] s={s} a={a} r={r:+.2f} s'={s_next} eps={eps:.2f}")

        s = s_next
        iteration += 1
