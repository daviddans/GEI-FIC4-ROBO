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
MAX_SPEED = 50             
CRUISE_SPEED = 14         # velocidad de la rueda exterior en cualquier acción (rad/s)
CURVE_INNER = 1.0          # velocidad de la rueda interior en giros (rad/s)
ACTION_STEPS_FWD = 15     # nº de ticks que dura A_FORWARD
ACTION_STEPS_TURN = 6      # nº de ticks que dura A_RIGHT/A_LEFT

BLACK_THR = 500            # < BLACK_THR  → sensor sobre negro (sobre la línea)
WHITE_THR = 750            # > WHITE_THR  → sensor sobre blanco (fuera de la línea)

OBSTACLE_THR_CENTER = 300  
OBSTACLE_THR_SIDE = 550    
AVOID_TURN_SPEED = 25.0    
AVOID_TURN_DEG = 45     

# --- Geometría del Khepera IV (para convertir ángulo robot a rotación de rueda) ---
WHEEL_RADIUS = 0.021      
WHEELBASE = 0.1054         

# --- Q-learning ---
GAMMA = 0.5                # factor de descuento γ del Q-update
EPS_DECAY_ITERS = 500      

# --- Mapeo de dispositivos en Webots ---
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
S1, S2, S3 = 0, 1, 2                
A_RIGHT, A_LEFT, A_FORWARD = 0, 1, 2
N_STATES, N_ACTIONS = 3, 3

REWARD_WEIGHTS = np.array([1.0, 2.0, 2.0, 1.0])

# Bonus/penalización extra en compute_reward cuando los 4 sensores coinciden:
# +BONUS_FULL si todos están sobre negro (centrado sobre una línea ancha o cruce),
# -BONUS_FULL si todos están sobre blanco (completamente fuera de la línea).
BONUS_FULL = 3.0


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
    vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
    vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)


def stop():
    set_motors(0.0, 0.0)


def read_ground():
    return np.array([s.getValue() for s in ground_sensors], dtype=np.float64)


def run_action(action_id):
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
    g_lat_l, g_cen_l, g_cen_r, g_lat_r = g[0], g[1], g[2], g[3]
    if g_cen_l > WHITE_THR and g_lat_r < BLACK_THR:
        return S1
    if g_cen_r > WHITE_THR and g_lat_l < BLACK_THR:
        return S2
    return S3


def compute_reward(prev_g, curr_g):
    """Recompensa derivada de la experiencia (no codificada a priori, por sensor).

    Los centrales (índices 1 y 2) pesan 2.0 y los laterales (0 y 3) pesan 1.0,
    así estar centrado sobre la línea recompensa más que rozarla con un lateral.
    Bonus extra si los 4 sensores coinciden
    Args:
        prev_g: lecturas de suelo antes de la acción (np.ndarray[4]).
        curr_g: lecturas de suelo tras la acción     (np.ndarray[4]).

    Returns:
        Suma de contribuciones por sensor más el bonus extra (float).
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
            total += 2 * w
        else:
            total -= 2 * w

    if np.all(curr_g < BLACK_THR):
        total += BONUS_FULL
    elif np.all(curr_g > WHITE_THR):
        total -= 1.5 * BONUS_FULL

    return float(total)


def epsilon(iteration):
    return max(0.0, 1.0 - iteration / EPS_DECAY_ITERS)


def select_action(s, eps):
    if np.random.random() < eps:
        return int(np.random.randint(N_ACTIONS))
    return int(np.argmax(Q[s]))


def update_q(s, a, r, s_next):
    global Q, visits
    visits[s, a] += 1
    alpha = 1.0 / (1 + visits[s, a])
    target = r + GAMMA * float(np.max(Q[s_next]))
    Q[s, a] = (1 - alpha) * Q[s, a] + alpha * target


# --- Módulo de evitación (no se aprende) ---

def turn_in_place(deg, sign):
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
