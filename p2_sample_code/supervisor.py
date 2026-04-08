#########################################################################################
# Robótica: código de ejemplo para el simulador Webots
# Grado en Ingeniería Informática
# Universidade da Coruña
# Author: Alejandro Paz
#
# Modo "supervisor" para mover programáticamente un objeto estático en el mundo.
#########################################################################################

from controller import Supervisor


def move_object(supervisor, objectRef, deltaX, deltaY):
    """
    Desplazamiento de un objeto del mundo en coordenadas X,Y.
    El robot debe instanciarse como Supervisor() y no como Robot().
    Debe establecerse a TRUE el atributo "supervisor" del robot en el mundo de Webots.

    supervisor: instancia del Supervisor (el robot se debe instanciar en modo supervisor)
    objectRef: referencia del objeto a desplazar (def name en el mundo)
    deltaX: desplazamiento en coordenada X (en metros)
    deltaY: desplazamiento en coordenada Y (en metros)
    """
    target_object = supervisor.getFromDef(objectRef)
    if target_object is None:
        print(f"Error: Element {objectRef} not found in the world.")
        exit(1)
    else:
        translation_field = target_object.getField("translation")
        current_position = translation_field.getSFVec3f()
        translation_field.setSFVec3f([current_position[0] + deltaX, current_position[1], current_position[2] + deltaY])
