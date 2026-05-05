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


class RobotAPI:
    def __init__(self):
        self.robot = Robot()

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        for m in (self.left_motor, self.right_motor):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        self.ground_sensors = []
        for name in GROUND_SENSOR_NAMES:
            s = self.robot.getDevice(name)
            s.enable(TIME_STEP)
            self.ground_sensors.append(s)

        self.front_sensors = []
        for name in FRONT_PROX_SENSOR_NAMES:
            s = self.robot.getDevice(name)
            s.enable(TIME_STEP)
            self.front_sensors.append(s)

        # Primer step para que las lecturas estén disponibles
        self.robot.step(TIME_STEP)

    def step(self):
        return self.robot.step(TIME_STEP)

    def set_motors(self, vl, vr):
        vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
        vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
        self.left_motor.setVelocity(vl)
        self.right_motor.setVelocity(vr)

    def stop(self):
        self.set_motors(0.0, 0.0)

    def read_ground(self):
        return np.array([s.getValue() for s in self.ground_sensors], dtype=np.float64)

    def read_front_proximity(self):
        return max(s.getValue() for s in self.front_sensors)

    def run_action(self, action_id):
        if action_id == A_FORWARD:
            self.set_motors(+CRUISE_SPEED, +CRUISE_SPEED)
        elif action_id == A_RIGHT:
            self.set_motors(+CRUISE_SPEED, +CURVE_INNER)
        elif action_id == A_LEFT:
            self.set_motors(+CURVE_INNER, +CRUISE_SPEED)
        else:
            self.stop()

        for _ in range(ACTION_STEPS):
            if self.step() == -1:
                return False
        return True


def get_state(g):
    g_lat_l, g_cen_l, g_cen_r, g_lat_r = g[0], g[1], g[2], g[3]
    if g_cen_l > WHITE_THR and g_lat_r < BLACK_THR:
        return S1
    if g_cen_r > WHITE_THR and g_lat_l < BLACK_THR:
        return S2
    return S3


def compute_reward(prev_g, curr_g):
    total = 0.0
    for i in range(4):
        was_on_line = prev_g[i] > WHITE_THR  # > 750 = on black line
        is_on_line = curr_g[i] > WHITE_THR
        w = REWARD_WEIGHTS[i]
        if is_on_line and not was_on_line:
            total += w  # Went from off-line to on-line: positive
        elif not is_on_line and was_on_line:
            total -= w  # Went from on-line to off-line: negative
        elif is_on_line and was_on_line:
            total += 0.5 * w  # Stayed on-line: small positive
        else:
            total -= 0.5 * w  # Stayed off-line: small negative
    return float(total)


class QLearner:
    def __init__(self, path=Q_FILE):
        self.path = path
        if os.path.exists(path):
            data = np.load(path)
            self.Q = data["Q"].astype(np.float64)
            self.visits = data["visits"].astype(np.int64)
            print(f"[Q] Cargada Q-table de {path}")
        else:
            self.Q = np.zeros((N_STATES, N_ACTIONS), dtype=np.float64)
            self.visits = np.zeros((N_STATES, N_ACTIONS), dtype=np.int64)
            print("[Q] Empezando con Q-table en ceros")

    def select_action(self, s, eps):
        if np.random.random() < eps:
            return int(np.random.randint(N_ACTIONS))
        return int(np.argmax(self.Q[s]))

    def update(self, s, a, r, s_next):
        self.visits[s, a] += 1
        alpha = 1.0 / (1 + self.visits[s, a])
        target = r + GAMMA * float(np.max(self.Q[s_next]))
        self.Q[s, a] = (1 - alpha) * self.Q[s, a] + alpha * target

    def save(self):
        np.savez(self.path, Q=self.Q, visits=self.visits)


def avoid_obstacles(robot):
    if robot.read_front_proximity() < OBSTACLE_THR:
        return False
    print("[AVOID] Obstáculo detectado, girando derecha hasta despejar")
    robot.set_motors(+2.0, -2.0)   # giro in-place sólo para evitación de obstáculos
    while robot.read_front_proximity() >= OBSTACLE_THR:
        if robot.step() == -1:
            return True
    robot.stop()
    return True


def epsilon(iteration):
    return max(0.0, 1.0 - iteration / EPS_DECAY_ITERS)


if __name__ == "__main__":
    robot = RobotAPI()
    qlearner = QLearner()

    iteration = 0
    prev_g = robot.read_ground()
    s = get_state(prev_g)

    while robot.step() != -1:
        if avoid_obstacles(robot):
            prev_g = robot.read_ground()
            s = get_state(prev_g)
            continue

        eps = epsilon(iteration)
        a = qlearner.select_action(s, eps)

        prev_g = robot.read_ground()
        if not robot.run_action(a):
            break
        curr_g = robot.read_ground()

        r = compute_reward(prev_g, curr_g)
        s_next = get_state(curr_g)
        qlearner.update(s, a, r, s_next)
        qlearner.save()

        print(f"[iter={iteration:4d}] s={s} a={a} r={r:+.2f} s'={s_next} eps={eps:.2f}")

        s = s_next
        iteration += 1
