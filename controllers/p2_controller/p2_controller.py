
import time
from controller import Robot, Supervisor # type: ignore
from enum import Enum
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import cv2  # Si queremos utilizar OpenCV para procesar la imagen.
import math
from collections import deque
import heapq

WHEEL_RADIUS = 21  # Radius in mm
WHEEL_SPACE = 108.29  # Space between wheels (specs:105.4mm, corrected:108.29mm)
RADIO_ENTRE_RUEDAS = 54.145  # Half of WHEEL_SPACE for Webots model

MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32
DISTANCIA_CUADRADICULA = 250/WHEEL_RADIUS
DISTANCIA_DIAGONAL = (250 * math.sqrt(2)) / WHEEL_RADIUS

#CONSTANTES
#mapa
WORLD_ROWS = 12
WORLD_COLS = 12
UNEXPLORED = -1
BASE = 0
WALL = -2

#robot
#Orientaciones
UP = 0
UPR = 1
R = 2 
DOWNR = 3
DOWN = 4
DOWNL = 5
L = 6
UPL = 7

#Tolerancia de deteccion IR
DETECT_TOL_STRAIGHT = 189 # Para sensores frontais, traseiros e laterais (0, 2, 4, 6)
DETECT_TOL_DIAGONAL = 167  # Para sensores nas esquinas (1, 3, 5, 7)

#Tolerancia de movimiento (encoder, rad)
MOVE_TOLERANCE = 0.01


#Deteccion de intruso (camara)
YELLOW_R_MIN = 180
YELLOW_G_MIN = 180
YELLOW_B_MAX = 80
YELLOW_MIN_PIXELS = 50

MAP_ROWS = WORLD_ROWS * 2 + 1  # 25 — simétrico: 12 pasos en cada dirección desde base
MAP_COLS = WORLD_COLS * 2 + 1  # 25

#Deltas de movimiento por orientación (UP=0 … UPL=7, sentido horario)
DIRECTION_DELTAS = [
    (-1, 0),  # UP
    (-1, 1),  # UPR
    (0, 1),   # R
    (1, 1),   # DOWNR
    (1, 0),   # DOWN
    (1, -1),  # DOWNL
    (0, -1),  # L
    (-1, -1)  # UPL
]

# Nombres de los sensores de distancia basados en infrarrojo.
# Ordeneinos como as orientacións do robot para facilitar a actualización do mapa.
INFRARED_SENSORS_NAMES = [
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
]


#Objeto que representa el estado del robot y contiene funciones de alto nivel para emplearlo
class RobotAPI():
    def __init__(self,):
        self.pos = (WORLD_ROWS, WORLD_COLS)
        self.orientation = UP #Asumimos que el robot empieza mirando al norte.
        self.webots_robot = Robot()

        self.leftWheel = self.webots_robot.getDevice("left wheel motor")
        self.rightWheel = self.webots_robot.getDevice("right wheel motor")
        self.leftWheel.setPosition(0)
        self.rightWheel.setPosition(0)
        self.leftWheel.setVelocity(0)
        self.rightWheel.setVelocity(0)

        self.posL = self.webots_robot.getDevice("left wheel sensor")
        self.posR = self.webots_robot.getDevice("right wheel sensor")
        self.posL.enable(TIME_STEP)
        self.posR.enable(TIME_STEP)

        self.irSensorList = []
        for name in INFRARED_SENSORS_NAMES:
            sensor = self.webots_robot.getDevice(name)
            sensor.enable(TIME_STEP)
            self.irSensorList.append(sensor)

        self.camera = self.webots_robot.getDevice("camera")
        self.camera.enable(TIME_STEP)

        self.webots_robot.step(TIME_STEP)

    def step(self):
        return self.webots_robot.step(TIME_STEP)
        
    def _move_distance(self, increment, velocity=CRUISE_SPEED):
        self.leftWheel.setVelocity(velocity)
        self.rightWheel.setVelocity(velocity)
        self.leftWheel.setPosition(self.posL.getValue() + increment)
        self.rightWheel.setPosition(self.posR.getValue() + increment)

        pL0 = self.posL.getValue()
        while self.posL.getValue() < (pL0 + increment - MOVE_TOLERANCE):
            if self.webots_robot.step(TIME_STEP) == -1:
                break

        if self.webots_robot.step(TIME_STEP) != -1:
            time.sleep(0.3)
        self.webots_robot.step(TIME_STEP)

    def _turn(self, target_orientation, velocity=CRUISE_SPEED):
        # 1. Diferencia mínima (-4 a 4)
        diff = (target_orientation - self.orientation) % 8
        if diff > 4:
            diff -= 8
        # 2. Ángulo en radianes
        angle = diff * (math.pi / 4)
        # 3. Convertir a movimiento de ruedas
        delta = angle * RADIO_ENTRE_RUEDAS / WHEEL_RADIUS
        # 4. Objetivos de ruedas
        targetL = self.posL.getValue() + delta
        targetR = self.posR.getValue() - delta
        self.leftWheel.setVelocity(velocity)
        self.rightWheel.setVelocity(velocity)
        self.leftWheel.setPosition(targetL)
        self.rightWheel.setPosition(targetR)
        # 5. Espera robusta
        while True:
            errorL = abs(self.posL.getValue() - targetL)
            errorR = abs(self.posR.getValue() - targetR)
            if errorL < MOVE_TOLERANCE and errorR < MOVE_TOLERANCE:
                break
            if self.webots_robot.step(TIME_STEP) == -1:
                break
        self.webots_robot.step(TIME_STEP)  # settling: let residual angular velocity decay
        # 6. Actualizar orientación
        self.orientation = target_orientation

    def _detect_yellow(self):
        # 1. Obtener la imagen
        image_data = self.camera.getImage()
        if not image_data:
            return False

        # 2. Convertir a array de NumPy (Webots usa BGRA)
        W = self.camera.getWidth()
        H = self.camera.getHeight()
        
        # Creamos una matriz desde el buffer de la cámara
        img = np.frombuffer(image_data, np.uint8).reshape((H, W, 4))

        # 3. Extraer canales (Recordar: Webots entrega BGRA)
        # img[:,:,0] -> Blue
        # img[:,:,1] -> Green
        # img[:,:,2] -> Red
        
        r = img[:, :, 2]
        g = img[:, :, 1]
        b = img[:, :, 0]

        # 4. Aplicar la máscara (Tus constantes de color)
        # Buscamos donde R y G son altos y B es bajo
        mask = (r > YELLOW_R_MIN) & (g > YELLOW_G_MIN) & (b < YELLOW_B_MAX)

        # 5. Contar píxeles amarillos
        count = np.sum(mask)
        
        if count >= YELLOW_MIN_PIXELS:
            print(f"[CAM] ¡Intruso detectado! Píxeles: {count}")
            return True
            
        return False

    def detect_intruder(self):
        return self._detect_yellow()
    
    def get_orientation(self):
        return self.orientation
    
    def get_position(self):
        return self.pos

    def scan_surroundings(self):
        """Returns list[8] of bool indexed by absolute direction (0=UP, clockwise).
        True = wall detected, False = free."""
        result = [False] * 8
        for i, sensor in enumerate(self.irSensorList):
            tol = DETECT_TOL_DIAGONAL if i % 2 != 0 else DETECT_TOL_STRAIGHT
            abs_dir = (i + self.orientation) % 8
            result[abs_dir] = sensor.getValue() > tol
        return result

    def move_forward(self):
        self._move_distance(DISTANCIA_CUADRADICULA)
        # Update position after movement
        dr, dc = DIRECTION_DELTAS[self.orientation]
        self.pos = (self.pos[0] + dr, self.pos[1] + dc)

    def turn_left90(self):
        self._turn((self.orientation - 2) % 8)

    def turn_right90(self):
        self._turn((self.orientation + 2) % 8)

    def turn_to(self, target_orientation):
        if target_orientation != self.orientation:
            self._turn(target_orientation)

    def turn_left45(self):
        self._turn((self.orientation - 1) % 8)

    def turn_right45(self):
        self._turn((self.orientation + 1) % 8)

    def move_forward_diagonal(self):
        self._move_distance(DISTANCIA_DIAGONAL)
        dr, dc = DIRECTION_DELTAS[self.orientation]
        self.pos = (self.pos[0] + dr, self.pos[1] + dc)


#Modulos de comportamiento.
# Clase que decide la siguiente accion del robot.
class Director:
    def __init__(self):
        self.mode = 'explore'
        self.has_left_base = False
        self._return_planned = False

    def plan_action(self, robot, map_obj, controller):
        if robot.get_position() != (WORLD_ROWS, WORLD_COLS):
            self.has_left_base = True
        else:
            map_obj.print_map()

        if self.mode == 'explore':
            if self.has_left_base and robot.get_position() == (WORLD_ROWS, WORLD_COLS):
                print("=" * 40)
                print("EXPLORATION COMPLETE — entering PATROL mode")
                print("=" * 40)
                self.mode = 'patrol'
            else:
                self._plan_explore(robot, map_obj, controller)
        elif self.mode == 'patrol':
            self._plan_patrol(robot, map_obj, controller)
        elif self.mode == 'return':
            self._plan_return(robot, map_obj, controller)
        elif self.mode == 'stop':
            return

    def _plan_explore(self, robot, map_obj, controller):
        r, c = robot.get_position()
        current_orient = robot.get_orientation()

        abs_left  = (current_orient - 2) % 8
        abs_front = current_orient
        abs_right = (current_orient + 2) % 8
        abs_back  = (current_orient + 4) % 8

        def is_blocked(direction_abs):
            dr, dc = DIRECTION_DELTAS[direction_abs]
            nr, nc = r + dr, c + dc
            if 0 <= nr < MAP_ROWS and 0 <= nc < MAP_COLS:
                return map_obj.getmap()[nr, nc] == WALL
            return True

        if not is_blocked(abs_left):
            target = abs_left
        elif not is_blocked(abs_front):
            target = abs_front
        elif not is_blocked(abs_right):
            target = abs_right
        else:
            target = abs_back

        print(f"Pos: {robot.get_position()}, Orient: {current_orient} | target abs dir: {target}")
        controller.add_action(target)

    def _plan_patrol(self, robot, map_obj, controller):
            # Hacemos que el robot se detenga un momento para "mirar" bien
            # o simplemente chequeamos la cámara.
            print("[CHECKING] Buscando intrusos...")
            if robot.detect_intruder():
                print("=" * 40)
                print("!!! INTRUSO DETECTADO !!!")
                print("=" * 40)
                self.mode = 'return'
                self._return_planned = False
            else:
                # Si no hay intruso, sigue explorando normalmente
                self._plan_explore(robot, map_obj, controller)

    def _plan_return(self, robot, map_obj, controller):
        if robot.get_position() == (WORLD_ROWS, WORLD_COLS):
            print("[RETURN] Arrived at base! Stopping...")
            self.mode = 'stop'
            self._return_planned = False
            return

        # Calcular el camino y encolar SOLO el primer paso.
        # De esta forma repensamos la ruta a cada casilla por si el mapa se actualiza
        # o necesitamos corregir, en lugar de hacerlo a ciegas.
        path = self._astar(robot.get_position(), map_obj)
        if path:
            # print(f"[RETURN] Path found: {len(path)} steps. Next dir: {path[0]}")
            controller.queue.clear() # Vaciamos por si acaso
            controller.add_action(path[0])
        else:
            print("[RETURN] No path found to base!")

    def _astar(self, start, map_obj):
        """A* from start to BASE (WORLD_ROWS, WORLD_COLS). Returns list of absolute directions."""
        goal = (WORLD_ROWS, WORLD_COLS)
        grid = map_obj.getmap()

        def h(pos):
            return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

        DIRS = [UP, R, DOWN, L]
        counter = 0
        # Estado: (f_score, counter, g_score, pos, path_dirs, prev_dir)
        open_set = [(h(start), counter, 0.0, start, [], None)]
        
        # Diccionario para guardar el menor costo a cada estado (posición, direccion).
        # Agregamos la dirección porque girar ahora tiene penalización.
        visited = {}

        while open_set:
            _, _, g, pos, path, prev_dir = heapq.heappop(open_set)

            state_key = (pos, prev_dir)
            if state_key in visited and visited[state_key] <= g:
                continue
            visited[state_key] = g

            if pos == goal:
                return path

            r, c = pos
            for abs_dir in DIRS:
                dr, dc = DIRECTION_DELTAS[abs_dir]
                nr, nc = r + dr, c + dc
                next_pos = (nr, nc)

                if not (0 <= nr < MAP_ROWS and 0 <= nc < MAP_COLS):
                    continue
                if grid[nr, nc] == WALL or grid[nr, nc] == UNEXPLORED:
                    continue

                # CRÍTICO: Penalización por giro.
                # A* estándar en cuadrículas crea rutas "zig-zag" (escalera) porque miden lo mismo,
                # pero en robótica física las rotaciones acumulan errores y destrozan el rastro ("drift").
                # Obligamos a que prefiera siempre líneas rectas (0 zig-zag).
                turn_penalty = 0.0
                if prev_dir is not None and prev_dir != abs_dir:
                    turn_penalty = 0.5 

                step_cost = 1.0 + turn_penalty
                new_g = g + step_cost
                
                next_state_key = (next_pos, abs_dir)
                if next_state_key in visited and visited[next_state_key] <= new_g:
                    continue

                counter += 1
                heapq.heappush(open_set, (new_g + h(next_pos), counter, new_g, next_pos, path + [abs_dir], abs_dir))

        return None


# Clase que se encarga de controlar el movimiento del robot.
class Controller:
    def __init__(self):
        self.queue = deque()
        self.status = None  # None | 'success' | 'failure'

    def add_action(self, abs_direction):
        """Queue an absolute direction (0-7) to move toward."""
        self.queue.append(abs_direction)

    def get_status(self):
        return self.status

    def act(self, robot):
        self.status = None
        if not self.queue:
            self.status = 'idle'
            return

        # Extraemos y ejecutamos SOLO el primer paso de la cola,
        # dejando que el bucle principal retome el control y actualice los sensores.
        abs_direction = self.queue.popleft()
        robot.turn_to(abs_direction)
        if abs_direction % 2 != 0:
            robot.move_forward_diagonal()
        else:
            robot.move_forward()
        print(f"[ACT] dir={abs_direction} pos={robot.get_position()}")

        if not self.queue:
            self.status = 'success'
        else:
            self.status = 'running'


# Clase que se encarga de generar el mapa en memoría y actualizarlo.
class Map:
    def __init__(self):
        self.map = np.full((MAP_ROWS, MAP_COLS), UNEXPLORED)
        self.map[WORLD_ROWS, WORLD_COLS] = BASE

    def update(self, robot):
        r, c = robot.get_position()
        dist = abs(r - WORLD_ROWS) + abs(c - WORLD_COLS)
        self.map[r, c] = BASE if dist == 0 else dist
        walls = robot.scan_surroundings()
        for abs_dir, is_wall in enumerate(walls):
            dr, dc = DIRECTION_DELTAS[abs_dir]
            nr, nc = r + dr, c + dc
            if 0 <= nr < MAP_ROWS and 0 <= nc < MAP_COLS:
                if is_wall:
                    self.map[nr, nc] = WALL
                else:
                    dist = abs(nr - WORLD_ROWS) + abs(nc - WORLD_COLS)
                    self.map[nr, nc] = BASE if dist == 0 else dist
        self.print_map()

    def getmap(self):
        return self.map

    def print_map(self):
        print("\n--- MAP - Status ---")
        for r in range(MAP_ROWS):
            row_str = ""
            for c in range(MAP_COLS):
                val = self.map[r, c]
                if val == WALL:
                    row_str += " W "
                elif val == UNEXPLORED:
                    row_str += " ? "
                elif val == BASE:
                    row_str += " B "
                else:
                    row_str += f"{int(val):2d} "
            print(row_str)
        print("-----------------\n")

if __name__ == "__main__":

    robot = RobotAPI()
    controller = Controller()
    director = Director()
    map_obj = Map()

    while robot.step() != -1:
        map_obj.update(robot)
        director.plan_action(robot=robot, map_obj=map_obj, controller=controller)
        controller.act(robot=robot)
