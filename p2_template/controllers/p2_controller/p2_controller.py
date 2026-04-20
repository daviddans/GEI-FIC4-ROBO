#########################################################################################
# Robótica: código de ejemplo para el simulador Webots
# Grado en Ingeniería Informática
# Universidade da Coruña
# Author: Alejandro Paz
#
# Función de ejemplo para mover el robot Khepera IV por posición.
#########################################################################################
import time
from controller import Robot, Supervisor
from enum import Enum
import time  # Si queremos utilizar time.sleep().
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import cv2  # Si queremos utilizar OpenCV para procesar la imagen.
import math

WHEEL_DIAMETER = 42  # Diameter in mm
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

#Tolerancia de deteccion
DETECT_TOL_STRAIGHT = 189   # Para sensores frontais, traseiros e laterais (0, 2, 4, 6)
DETECT_TOL_DIAGONAL = 167  # Para sensores nas esquinas (1, 3, 5, 7)

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


def enable_distance_sensors(robot, timeStep, sensorNames):
    """
    Obtener y activar los sensores de distancia.

    Return: lista con los sensores de distancia activados, en el mismo orden
    establecido en la lista de  nombres (sensorNames).
    """

    sensorList = []

    for name in sensorNames:
        sensorList.append(robot.getDevice(name))

    for sensor in sensorList:
        sensor.enable(timeStep)

    return sensorList

def process_image_rgb(camera):
    """
    Procesamiento del último frame capturado por el dispositivo de la cámara
    (según el time_step establecido para la cámara).
    ¡ATENCIÓN!: Esta función no es thread-safe, ya que accede al buffer en memoria de la cámara.

    RECOMENDACIÓN: utilizar OpenCV para procesar más eficientemente la imagen
    (ej. hacer una detección de color en HSV).
    """

    W = camera.getWidth()
    H = camera.getHeight()

    image = camera.getImage()

    # Si es suficiente, podríamos procesar solo una parte de la imagen para optimizar .
    for x in range(0, W):
        for y in range(0, H):
            b = camera.imageGetBlue(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            r = camera.imageGetRed(image, W, x, y)

            # TODO: Procesar el pixel (x,y) de la imagen.
            # ...


            
def move_distance_position(increment, velocity, timeStep, robot, leftWheel, rightWheel, posL, posR):
    """
    Mover el robot en línea recta una distancia determinada utilizando movimiento por posición.

    leftWheel: motor del lado izquierdo
    rightWheel: motor del lado derecho
    posL: sensor de posición del motor del lado izquierdo
    posR: sensor de posición del motor del lado derecho
    increment: distancia a recorrer (en metros)
    velocity: velocidad a la que se moverá el robot (en m/s)
    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores.
    """
    leftWheel.setVelocity(velocity)
    rightWheel.setVelocity(velocity)
    leftWheel.setPosition(posL.getValue() + increment)
    rightWheel.setPosition(posR.getValue() + increment)

    pL0 = posL.getValue()
    pR0 = posR.getValue()

    while posL.getValue() < (pL0 + increment - 0.01):
        if robot.step(timeStep) == -1:
            break

    # Opcionalmente se puede introducir una espera para asegurar que el robot está parado
    # al finalizar la ejecución del movimiento
    if robot.step(timeStep) != -1:
        time.sleep(0.01)
    robot.step(timeStep)

def turn_angle(target_orientation, current_orientation, velocity, timeStep, robot, leftWheel, rightWheel, posL, posR):
    """
    Girar el robot sobre su propio eje hasta la orientación objetivo.

    target_orientation: orientación objetivo (0-7)
    current_orientation: orientación actual del robot (0-7)
    velocity: velocidad a la que se girará el robot (en m/s)
    timeStep: tiempo (en milisegundos) de actualización
    """
    angle_diff = (target_orientation - current_orientation) % 8
    if angle_diff > 4:
        angle_diff -= 8
    angle_rad = angle_diff * (math.pi / 4)
    #Cálculo dos radians de xiro
    delta = angle_rad * RADIO_ENTRE_RUEDAS / WHEEL_RADIUS
    leftWheel.setVelocity(velocity)
    rightWheel.setVelocity(velocity)
    leftWheel.setPosition(posL.getValue() + delta)
    rightWheel.setPosition(posR.getValue() - delta)

    pL0 = posL.getValue()
    while posL.getValue() < (pL0 + delta - 0.01) if delta > 0 else posL.getValue() > (pL0 + delta + 0.01):
        if robot.step(timeStep) == -1:
            break

    if robot.step(timeStep) != -1:
        time.sleep(0.01)
    robot.step(timeStep)

def initialization():

    # Instanciar objeto Robot (alternativamente, Supervisor)
    robot = Robot()

    # Si queremos obtener el timestep de la simulación.
    # timeStep = int(robot.getBasicTimeStep())
    timeStep = 32

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener y activar los sensores de posición de las ruedas (encoders).
    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timeStep)
    posR.enable(timeStep)
    robot.step(timeStep)

    # Obtener una lista con los sensores infrarrojos ya activados
    irSensorList = enable_distance_sensors(robot, timeStep, INFRARED_SENSORS_NAMES)

    # Obtener el dispositivo de la cámara
    camera = robot.getDevice("camera")
    # Activar el dispositivo de la cámara (el tiempo de actualización de los frames
    # de la cámara no debería ser muy alto debido al alto volumen de datos que implica).
    camera.enable(timeStep * 10)

    return robot, leftWheel, rightWheel, irSensorList, posL, posR, camera

#Objeto que representa el estado del robot y contiene funciones de alto nivel para emplearlo
class RobotAPI():
    deltas_movement = [
        (-1, 0),  # UP
        (-1, 1),  # UPR
        (0, 1),   # R
        (1, 1),   # DOWNR
        (1, 0),   # DOWN
        (1, -1),  # DOWNL
        (0, -1),  # L
        (-1, -1)  # UPL
    ]
    
    def __init__(self, webots_robot, leftWheel, rightWheel, posL, posR):
        #Posición robot
        self.pos = (WORLD_ROWS, WORLD_COLS)
        self.orientation = 0
        self.webots_robot = webots_robot
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        #Posición rodas
        self.posL = posL
        self.posR = posR
        
    def look(self, direction):
        if not (0 <= direction < 8):
            raise ValueError("Invalid direction")
        turn_angle(direction, self.orientation, CRUISE_SPEED, TIME_STEP, self.webots_robot, self.leftWheel, self.rightWheel, self.posL, self.posR)
        self.orientation = direction

    def update_position(self, orientation):
        dr, dc = self.deltas_movement[orientation]
        self.pos = (self.pos[0] + dr, self.pos[1] + dc)

        #Implementar control del robot


#Modulos de comportamiento.
# Clase que decide la siguiente accion del robot.
class Director:
    def __init__(self):
        pass

    def plan_action(self, robot, map, controller):
        r, c = robot.pos
        deltas = [
            (-1, 0),  # UP
            (-1, 1),  # UPR
            (0, 1),   # R
            (1, 1),   # DOWNR
            (1, 0),   # DOWN
            (1, -1),  # DOWNL
            (0, -1),  # L
            (-1, -1)  # UPL
        ]
        
        # Chekear en que direccions hai muros
        directions_with_walls = []
        #d sirve como acumulador e indica a dirección que se está comprobando (mesma notación que nas variables gloabais)
        for d in range(8):
            dr, dc = deltas[d]
            nr = r + dr
            nc = c + dc
            if 0 <= nr < WORLD_ROWS * 2 - 1 and 0 <= nc < WORLD_COLS * 2 - 1:
                if map.getmap()[nr, nc] == WALL:
                    directions_with_walls.append(d)
        
        # Calcular posicións relativas dos muros (0=Fronte, 2=Dereita, 4=Atrás, 6=Esquerda, 1/3/5/7=Diagonais)
        rel_walls = [(d - robot.orientation) % 8 for d in directions_with_walls]
        print(f"Map detected walls (relative): {rel_walls}")
        
        if 5 in rel_walls and 6 not in rel_walls:
            # O muro á nosa esquerda rematou (descubrimos unha esquina cara á esquerda).
            # Debemos xirar á esquerda, incluso se hai un muro en fronte (posición 0), 
            # para seguir pegados á parede.
            if 7 in rel_walls and 0 not in rel_walls:
                controller.action = 'move_forward'  # Si hay muro en la diagonal izquierda, giramos a la derecha para evitar quedarnos atrapados
            else:
                controller.action = 'turn_left'
        elif 0 in rel_walls:
            # Hai muro en fronte pero NON hai esquina á esquerda. 
            # O paso está bloqueado, debemos virar á dereita.
            controller.action = 'turn_right'
        elif 6 in rel_walls:
            # Muro á esquerda, e libre en fronte, seguir avanzando para bordealo
            controller.action = 'move_forward'
        else:
            # Sen muros coñecidos nas posicións clave. Avanzar para explorar
            controller.action = 'move_forward'


# Clase que se encarga de controlar el movimiento del robot.
class Controller:
    def __init__(self):
        self.action = None
        self.target_orientation = None
    
    def act(self, robot, map):
        print(f"Action: {self.action}, Orientation: {robot.orientation}")
        if self.action == 'move_forward':
            move_distance_position(DISTANCIA_CUADRADICULA, CRUISE_SPEED, TIME_STEP, robot.webots_robot, robot.leftWheel, robot.rightWheel, robot.posL, robot.posR)
            robot.update_position(robot.orientation)
            map.mark_explored(robot.pos)
        elif self.action == 'turn_to':
            turn_angle(self.target_orientation, robot.orientation, CRUISE_SPEED, TIME_STEP, robot.webots_robot, robot.leftWheel, robot.rightWheel, robot.posL, robot.posR)
            robot.orientation = self.target_orientation
        elif self.action == 'turn_left':
            target_orientation = (robot.orientation - 2) % 8
            turn_angle(target_orientation, robot.orientation, CRUISE_SPEED, TIME_STEP, robot.webots_robot, robot.leftWheel, robot.rightWheel, robot.posL, robot.posR)
            robot.orientation = target_orientation
        elif self.action == 'turn_right':
            target_orientation = (robot.orientation + 2) % 8
            turn_angle(target_orientation, robot.orientation, CRUISE_SPEED, TIME_STEP, robot.webots_robot, robot.leftWheel, robot.rightWheel, robot.posL, robot.posR)
            robot.orientation = target_orientation
        # Reseteamos a acción
        self.action = None
        self.target_orientation = None


# Clase que se encarga de generar el mapa en memoría y actualizarlo.
class Map:
    def __init__(self):
        self.map = np.full((WORLD_ROWS*2-1, WORLD_COLS*2-1), UNEXPLORED)
        self.map[WORLD_ROWS, WORLD_COLS] = BASE  # Assuming BASE is 0

    def update(self, irSensorList, robot):
        r, c = robot.pos
        deltas = [
            (-1, 0),  # UP
            (-1, 1),  # UPR
            (0, 1),   # R
            (1, 1),   # DOWNR
            (1, 0),   # DOWN
            (1, -1),  # DOWNL
            (0, -1),  # L
            (-1, -1)  # UPL
        ]
        for i in range(8):
            value = irSensorList[i].getValue()
            # Determinar a tolerancia en función de se é un sensor recto ou en diagonal
            tol = DETECT_TOL_DIAGONAL if i % 2 != 0 else DETECT_TOL_STRAIGHT
            
            if value > tol:
                direction = (i + robot.orientation) % 8
                dr, dc = deltas[direction]
                nr = r + dr  # Posición na que está o muro como a posición do robot + o incremento correspondente á dirección do sensor
                nc = c + dc
                if 0 <= nr < WORLD_ROWS * 2 - 1 and 0 <= nc < WORLD_COLS * 2 - 1:
                    # Sobrescribimos a WALL SOLO si esa celda aún era UNEXPLORED
                    if self.map[nr, nc] == UNEXPLORED:
                        self.map[nr, nc] = WALL
    
    def mark_explored(self, pos):
        dist = abs(pos[0] - WORLD_ROWS) + abs(pos[1] - WORLD_COLS)
        # Siempre forzamos a sobreescribir la celda con su distancia al explorarla (esto elimina los "muros fantasma" falsos)
        self.map[pos[0], pos[1]] = dist
    
    def getmap(self):
        return self.map
        
    def print_map(self):
        print("\n--- FINAL MAP ---")
        for r in range(WORLD_ROWS*2-1):
            row_str = ""
            for c in range(WORLD_COLS*2-1):
                val = self.map[r, c]
                if val == WALL:
                    row_str += " W "
                elif val == UNEXPLORED:
                    row_str += " ? "
                elif val == BASE:
                    row_str += " B "
                else:
                    # Formatear el número para que ocupe 2 espacios y sea legible
                    row_str += f"{int(val):2d} "
            print(row_str)
        print("-----------------\n")

if __name__ == "__main__":

    #Inializacion dos parametros do robot KeperaIV en webbots e toma de variables
    kepir, lWheel, rWheel, irSensorList, leftWheelPos, rightWheelPos, camera = initialization()

    #Inializacion de los modulos de comportamiento del robot.
    robot = RobotAPI(kepir, lWheel, rWheel, leftWheelPos, rightWheelPos)
    controller = Controller()
    director = Director()
    map = Map()

    # Loop infinito sincronizado con Webots
    has_left_base = False
    returned_to_base = False
    while kepir.step(TIME_STEP) != -1:
        # Print raw sensor values rounded for easier reading
        raw_sensors = [round(sensor.getValue(), 2) for sensor in irSensorList]
        print(f"Raw Sensors (0-7): {raw_sensors}")
        
        map.update(irSensorList, robot)
        
        if robot.pos != (WORLD_ROWS, WORLD_COLS):
            has_left_base = True
        
        # Check if robot returned to base after moving
        if robot.pos == (WORLD_ROWS, WORLD_COLS) and has_left_base and not returned_to_base:
            print("Robot returned to base!")
            map.print_map()
            returned_to_base = True
            # Optional: break the loop or continue if desired
            # break
        
        # Decidir accion a tomar
        director.plan_action(robot=robot, map=map, controller=controller)
        # Ejecutar la accion
        controller.act(robot=robot, map=map)