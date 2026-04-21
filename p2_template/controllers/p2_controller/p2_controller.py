#########################################################################################
# Robótica: código de ejemplo para el simulador Webots
# Grado en Ingeniería Informática
# Universidade da Coruña
# Author: Alejandro Paz
#
# Función de ejemplo para mover el robot Khepera IV por posición.
#########################################################################################
import time
from controller import Robot, Supervisor # type: ignore
from enum import Enum
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import cv2  # Si queremos utilizar OpenCV para procesar la imagen.
import math
from collections import deque

WHEEL_RADIUS = 21  # Radius in mm
WHEEL_SPACE = 108.29  # Space between wheels (specs:105.4mm, corrected:108.29mm)
RADIO_ENTRE_RUEDAS = 54.145  # Half of WHEEL_SPACE for Webots model

MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32
DISTANCIA_CUADRADICULA = 250/WHEEL_RADIUS

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
DETECT_TOL_STRAIGHT = 150 # Para sensores frontais, traseiros e laterais (0, 2, 4, 6)
DETECT_TOL_DIAGONAL = 140  # Para sensores nas esquinas (1, 3, 5, 7)

#Tolerancia de movimiento (encoder, rad)
MOVE_TOLERANCE = 0.02


#Deteccion de intruso (camara)
YELLOW_R_MIN = 200
YELLOW_G_MIN = 200
YELLOW_B_MAX = 50
YELLOW_MIN_PIXELS = 5

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
        self.camera.enable(TIME_STEP * 10)

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
        # 6. Actualizar orientación
        self.orientation = target_orientation

    def _detect_yellow(self):
        image = self.camera.getImage()
        W = self.camera.getWidth()
        H = self.camera.getHeight()
        count = 0
        for x in range(W):
            for y in range(H):
                r = self.camera.imageGetRed(image, W, x, y)
                g = self.camera.imageGetGreen(image, W, x, y)
                b = self.camera.imageGetBlue(image, W, x, y)
                if r > YELLOW_R_MIN and g > YELLOW_G_MIN and b < YELLOW_B_MAX:
                    count += 1
                    if count >= YELLOW_MIN_PIXELS:
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


#Modulos de comportamiento.
# Clase que decide la siguiente accion del robot.
class Director:
    def __init__(self):
        self.mode = 'explore'
        self.has_left_base = False

    def plan_action(self, robot, map_obj, controller):
        if robot.get_position() != (WORLD_ROWS, WORLD_COLS):
            self.has_left_base = True
        else:
            map_obj.print_map()

        if self.mode == 'explore':
            if self.has_left_base and robot.get_position() == (WORLD_ROWS, WORLD_COLS):
                print("Exploration complete. Switching to patrol.")
                self.mode = 'patrol'
            else:
                self._plan_explore(robot, map_obj, controller)
        elif self.mode == 'patrol':
            self._plan_patrol(robot, map_obj, controller)
        elif self.mode == 'return':
            self._plan_return(robot, map_obj, controller)

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
        if robot.detect_intruder():
            print("Intruder detected! Returning to base.")
            self.mode = 'return'
        else:
            self._plan_explore(robot, map_obj, controller)

    def _plan_return(self, robot, map_obj, controller):
        pass  # A* pendiente


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

        while self.queue:
            abs_direction = self.queue.popleft()

            diff = (abs_direction - robot.get_orientation()) % 8
            if diff == 2:
                robot.turn_right90()
            elif diff == 4:
                robot.turn_right90()
                robot.turn_right90()
            elif diff == 6:
                robot.turn_left90()

            print(f"Moving toward abs dir: {abs_direction}, orient: {robot.get_orientation()}")
            robot.move_forward()

        self.status = 'success'


# Clase que se encarga de generar el mapa en memoría y actualizarlo.
class Map:
    def __init__(self):
        self.map = np.full((MAP_ROWS, MAP_COLS), UNEXPLORED)
        self.map[WORLD_ROWS, WORLD_COLS] = BASE

    def update(self, robot):
        self.print_map()
        r, c = robot.get_position()
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

    def mark_explored(self, pos):
        r, c = pos
        dist = abs(r - WORLD_ROWS) + abs(c - WORLD_COLS)
        self.map[r, c] = BASE if dist == 0 else dist

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
        map_obj.mark_explored(robot.get_position())