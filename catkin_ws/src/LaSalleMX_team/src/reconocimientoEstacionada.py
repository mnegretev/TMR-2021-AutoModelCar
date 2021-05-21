#! /usr/bin/env python
from clasesReconocimientoEstacionarse import *
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge as cvpuente
from cv_bridge import CvBridgeError as cverr
import rospy
import cv2 as cv2
from std_msgs.msg import Int16
import threading

#Objetivo 1, tomar un cuadro , cambiarlo a blanco y negro y guardarla en el directorio  CUMPLIDO BEBEEEEEEEEEEEEEEEE
#nombredelnodo=reconcarretera
#Suscrito a:
#     /app/camera/rgb/image_raw

##------------------------------------------------
#Publica a:

#    /instruccionesAutomodel

##------------------------------------------------

####fUNCIONES
def callbacksubs(data):
    #Obtenemos la imagen del topico y la transformamos a un formato que entienda opencv
    threading.current_thread()
    if 'banderaEstacionamiento' in globals():
        #print ("GIROOOOO",globals()['giro'])
        global banderaEstacionamiento
        bandera=banderaEstacionamiento
    else:
        print ("Variable no ha sido inicializada, por ende su default es cero")
        bandera=0
    cv=cvpuente()
    imagen=cv.imgmsg_to_cv2(data,"bgr8")
    #Esta es una clase proveniente de clasesReconocimientoTest, la idea es que esta clase contiene todo lo relacionado al procesamiento de la imagen
    recon=streetDetection(imagen)
    objetivo=recon.reconoceCalle(imagen,bandera)
    publisher(objetivo,imagen)
    rospy.Rate(60).sleep()

def publisher(instruccion,imagen):
    topicoToPublish="/instruccionesAutomodel"
    recon=streetDetection(imagen)
    publishervel = rospy.Publisher(topicoToPublish,String,queue_size=1)
    publishervel.publish(instruccion)

def subscriber():
    topicToSub="/app/camera/rgb/image_raw"
    rospy.Subscriber(topicToSub,Image,callbacksubs)

def JalamosBandera(data):
    threading.current_thread()
    print("#################################################################################################################")
    print(data.data)
    print("#################################################################################################################")
    global banderaEstacionamiento
    banderaEstacionamiento=data.data
    rospy.Rate(60).sleep()

def subscriberBandera():
    topicToSub="/banderaEstacionamiento"
    rospy.Subscriber(topicToSub,Int16,JalamosBandera)
#Main
if __name__ == "__main__":
    rospy.init_node("reconcarretera")
    #1 Nos subscrimos para recibir la imagen de la camara del video
    subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Adiosito")
