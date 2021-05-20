#! /usr/bin/env python

import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
import rospy
from std_msgs.msg import Int16
#NOTAS
    #90 en el topico de giro es el que endereza las llantas
    #Numeros menores a 90 giran derecha, mayores a 90 giran izquierda
#Para aumentar la velocidad, tienes que ir de 30 en 30 masomenos

#Suscrito a:
#     /app/camera/rgb/image_raw

##------------------------------------------------
#Publica a:
#    /AutoModelMini/manual_control/speed
#    /AutoModelMini/manual_control/steering
##------------------------------------------------

def controlTeclado():
    print("Porfavor inserta gentilmente el comando")
    print("                 w                   ")
    print("a                                   d")
    print("                 s                   ")
    decision = raw_input()
    return decision

def cambiospertinentes(desicion,vel,giro):
    #Relacionado con velocidad de avance
    print(vel>0)
    if desicion == 's':
        if vel>0 or vel==0:
            vel=vel+30
            print("Velocidad aumentada",vel)
        if vel<0:
            vel=0
            print("STAAAAAAAAAAAAAAAAP")
    #Relacionado con velocidad de reversa
    if desicion == "w":
        if vel>0:
            vel=0
            print("STAAAAAAAAAAAAAAAAP")
        if vel<0 or vel==0:
            vel=vel-30
            print("Reversa aumentada",vel)
    #Relacionado con giro a la izquierdo
    if desicion == "a":
        if giro>90 or giro == 90:
            giro=giro+10
            print("Izquierda")
        if giro<90:
            giro=90
            print("Vamos recto")
    #Giro a la derecha
    if desicion == "d":
        if giro>90:
            giro=90
            print("Vamos recto")
        if giro<90 or giro == 90:
            giro=giro-10
            print("Derecha")

    return vel,giro

def publisher(giro,velocidad):
    topicoToPublish="/AutoModelMini/manual_control/speed"
    topicoToPublish2="/AutoModelMini/manual_control/steering"
    publishervel = rospy.Publisher(topicoToPublish,Int16,queue_size=1)
    publishergiro = rospy.Publisher(topicoToPublish2,Int16,queue_size=1)
    publishervel.publish(velocidad)
    publishergiro.publish(giro)
    print("Publicado")


#YA compila, no hace los cambios y no se repite el ciclo, checa esos paps



#Main
if __name__ == "__main__":
    vel=0
    giro=90
    while not rospy.is_shutdown():
        rospy.init_node("controldeb")
        desicion = controlTeclado()
        vel,giro=cambiospertinentes(desicion,vel,giro)
        publisher(giro,vel)
        print("cicloterminado","Vel:",vel,"Giro",giro)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
