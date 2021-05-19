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

lineas_y = (
    LINEA_Y - MARGEN_LINEA,
    LINEA_Y,
    LINEA_Y + MARGEN_LINEA,
)


def girar(grados):
    gir_pub.publish(grados)


def avanzar(velocidad, tiempo):
    vel_pub = rospy.Publisher(
        '/AutoNOMOS_mini/manual_control/speed',
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
    global binary_image, suscriptor, cv_image

    rospy.loginfo('1era')
    bridge_object = CvBridge()

    try:
        cv_image = bridge_object.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        determina_rectangulo()
        rospy.Subscriber(
            '/app/camera/rgb/image_raw',
            Image,
            procesar_imagen
        )

    except CvBridgeError as e:
        print(e)

    suscriptor.unregister()


def determina_puntos_medios():
    global binary_image

    puntos_medios = []

    for i in range(len(lineas_y)):
        blancos = []
        inicio, fin = None, None

        for x, px in enumerate(binary_image[lineas_y[i]].tolist()):
            if px == (255):
                if not inicio:
                    inicio = x
                fin = x

            else:
                if inicio and fin:
                # and fin-inicio < 10:
                    blancos.append((inicio + fin) // 2)
                    inicio = fin = None

        if inicio and fin:
        # and fin-inicio < 10:
            blancos.append((inicio + fin) // 2)
            inicio = fin = None

        if len(blancos) > 1:
            puntos_medios.append((blancos[0] + blancos[1]) // 2)
        else:
            puntos_medios.append(-1)
    return puntos_medios


def determina_rectangulo():
    global binary_image, rectangulo
    RECT_X = 1

    puntos_medios = determina_puntos_medios()

    for punto in puntos_medios:
         if punto > 0:
            rectangulo = (punto - RECT_X, punto + RECT_X)
            rospy.loginfo(rectangulo)
            return

    raise Exception('No se encontro ningun punto medio')


def procesar_imagen(message):
    global binary_image, rectangulo, cv_image, recuerda_grados

    bridge_object = CvBridge()

    try:
        cv_image = bridge_object.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        puntos_medios = determina_puntos_medios()
        rospy.loginfo(puntos_medios)

        for i in range(len(lineas_y)):
            cv2.line(cv_image, (0, lineas_y[i]), (640, lineas_y[i]), (0, 255, 0), 1)
            cv2.line(
                cv_image,
                (puntos_medios[i], lineas_y[i] - 3),
                (puntos_medios[i], lineas_y[i] + 3),
                (0, 150, 200),
                1
            )

        cv2.rectangle(
            cv_image,
            (rectangulo[0],LINEA_Y - MARGEN_LINEA),
            (rectangulo[1],LINEA_Y + MARGEN_LINEA),
            (255, 0, 0), 1
        )

        cv2.imshow('RGB image', cv_image)
        # cv2.imshow("binary image", binary_image)
        cv2.waitKey(3)

        dir = determina_direccion()
        girar(dir)
        avanzar(-125, 0.1)

    except CvBridgeError as e:
        print(e)


def determina_direccion():
    global rectangulo

    pm = determina_puntos_medios()

    for punto in pm:
        if punto > 0:
            grados = 90 + (rectangulo[0] - punto) / 3
            rospy.loginfo('g: ' + str(grados))
            return grados

    rospy.loginfo('aaa')


if __name__ == '__main__':
    try:
        rospy.init_node('funciones', anonymous=True)

        cv_image = None
        binary_image = None
        rectangulo = None
        primera_vez = True
        grados_anteriores = 90
        recuerda_grados = False
        integral = 0

        gir_pub = rospy.Publisher(
            '/AutoNOMOS_mini/manual_control/steering',
            Int16,
            queue_size=10
        )

        suscriptor = rospy.Subscriber(
            '/app/camera/rgb/image_raw',
            Image,
            procesar_imagen_1era
        )

        rospy.loginfo('hola')

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("fin")

    except rospy.ROSInterruptException:
        pass
