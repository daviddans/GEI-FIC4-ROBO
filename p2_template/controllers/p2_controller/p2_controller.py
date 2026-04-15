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

WHEEL_DIAMETER = 42  # Diameter in mm
WHEEL_RADIUS = 21  # Radius in mm
WHEEL_SPACE = 108.29  # Space between wheels (specs:105.4mm, corrected:108.29mm)

MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32

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
DETECT_TOL = 0

# Nombres de los sensores de distancia basados en infrarrojo.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
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
        robot.step(timeStep)

    # Opcionalmente se puede introducir una espera para asegurar que el robot está parado
    # al finalizar la ejecución del movimiento
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
    def __init__(self):
        self.pos = (WORLD_ROWS, WORLD_COLS)
        self.orientation = 0
        
    def look(self, direction):
        if not (0 <= direction < 8):
            raise ValueError("Invalid direction")
        self.orientation = direction
        #Implementar control del robot


#Modulos de comportamiento.
# Clase que decide la siguiente accion del robot.
class Director:
    def __init__(self):
        pass

    def plan_action(self, robot, map, controller):
        pass

# Clase que se encarga de controlar el movimiento del robot.
class Controller:
    def __init__(self, director):
        pass
    
    def act(self, map):
        pass


# Clase que se encarga de generar el mapa en memoría y actualizarlo.
class Map:
    def __init__(self, director):
        self.map = np.full((WORLD_ROWS*2-1, WORLD_COLS*2-1), UNEXPLORED)
        self.map[WORLD_COLS,WORLD_ROWS] = 0

    def update():
        pass
    
    def getmap(self):
        return self.map
if __name__ == "__main__":

    #Inializacion de los parametros del robot KeperaIV en webbots
    initialization()

    #Inializacion de los modulos de comportamiento del robot.
    robot = RobotAPI()
    controller = Controller()
    director = Director()
    map = Map()

     #Loop infinito
    while(True):
        #Decidir accion a tomar
        director.plan_action(robot=robot, map=map, controller=controller)
        #Ejecutar la accion
        controller.act(map=map)