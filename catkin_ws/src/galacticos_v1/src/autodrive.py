#!/usr/bin/env python

from typing import List
import cv2
import numpy as np
from functools import partial
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Image

MARGEN_ABAJO = 70
ALTURA = 480 - MARGEN_ABAJO

LINEA_Y = ALTURA - 30
MARGEN_LINEA = 25

# SPEED_TOPIC = '/AutoNOMOS_mini/manual_control/speed'
# STEERING_TOPIC = '/AutoNOMOS_mini/manual_control/steering'

CAM_TOPIC = '/app/camera/rgb/image_raw'
SPEED_TOPIC = '/AutoModelMini/manual_control/speed'
STEERING_TOPIC = '/AutoModelMini/manual_control/steering'

LINEAS_Y = [
    LINEA_Y - MARGEN_LINEA,
    LINEA_Y,
    LINEA_Y + MARGEN_LINEA,
]

def girar(grados):
    if grados:
        gir_pub.publish(grados)


def avanzar(velocidad, tiempo):
    vel_pub = rospy.Publisher(
        SPEED_TOPIC,
        Int16,
        queue_size=10
    )
    frecuencia = 1000
    rate = rospy.Rate(frecuencia) # 1hz

    i = 0
    while i < tiempo * frecuencia:
        vel_pub.publish(velocidad)
        rate.sleep()
        i += 1


def procesar_imagen_1era(message):
    global binary_image, cv_image
    bridge_object = CvBridge()

    try:
        cv_image = bridge_object.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        lineas_y = LINEAS_Y + [
            LINEA_Y - (i * MARGEN_LINEA / 2)
            for i in range(2, 8)
        ]

        pm = determina_puntos_medios(lineas_y, True)
        grados = None

        for punto in pm:
            if punto > 0:
                grados = (320 - punto) / 2
                break

        if not grados:
            raise Exception('no se encontro ningun punto medio')

        dir = 90 + grados

        girar(dir)
        avanzar(-150, 2)
        avanzar(0, 1)

        rospy.Subscriber(
            CAM_TOPIC,
            Image,
            procesar_imagen
        )

    except CvBridgeError as e:
        print(e)


def determina_puntos_medios(lineas_y=LINEAS_Y, inverso=False):
    global binary_image

    puntos_medios = []
    r = range(len(lineas_y))

    if inverso:
        r = tuple(reversed(r))

    for i in r:
        blancos = []
        inicio, fin = None, None

        for x, px in enumerate(binary_image[lineas_y[i]].tolist()):
            if px == (255):
                if not inicio:
                    inicio = x
                fin = x
            else:
                if inicio and fin:
                    blancos.append((inicio + fin) // 2)
                    inicio = fin = None

        if inicio and fin:
            blancos.append((inicio + fin) // 2)
            inicio = fin = None

        if len(blancos) > 1 and len(blancos) < 3:
            puntos_medios.append((blancos[0] + blancos[1]) // 2)
        else:
            puntos_medios.append(-1)
    return puntos_medios


def procesar_imagen(message):
    global binary_image, cv_image, recuerda_grados

    bridge_object = CvBridge()

    try:
        cv_image = bridge_object.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        dir = determina_direccion()

        if dir:
            if dir > 95:
                dir += 20
            elif dir < 60:
                dir -= 10

        girar(dir)
        avanzar(-125, 0.1)

    except CvBridgeError as e:
        print(e)


def determina_direccion():
    pm = determina_puntos_medios()

    for punto in pm:
        if punto > 0:
            grados = 90 + (rectangulo[0] - punto) / 3
            return grados

    return None


if __name__ == '__main__':
    try:
        rospy.init_node('funciones', anonymous=True)

        cv_image = None
        binary_image = None
        rectangulo = (319, 321)

        gir_pub = rospy.Publisher(
            STEERING_TOPIC,
            Int16,
            queue_size=10
        )
        rospy.sleep(1)

        procesar_imagen_1era(rospy.wait_for_message(CAM_TOPIC, Image))

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("fin")

    except rospy.ROSInterruptException:
        pass
