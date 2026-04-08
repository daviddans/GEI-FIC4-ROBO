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

WHEEL_DIAMETER = 42  # Diameter in mm
WHEEL_RADIUS = 21  # Radius in mm
WHEEL_SPACE = 108.29  # Space between wheels (specs:105.4mm, corrected:108.29mm)


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


if __name__ == "__main__":

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

    # Mover el robot 250mm en línea recta con una 'velocidad' de 10.
    move_distance_position(
        increment=250 / WHEEL_RADIUS,
        velocity=10,
        timeStep=timeStep,
        robot=robot,
        leftWheel=leftWheel,
        rightWheel=rightWheel,
        posL=posL,
        posR=posR,
    )
