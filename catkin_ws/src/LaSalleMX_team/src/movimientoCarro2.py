#! /usr/bin/env python
import time
from std_msgs.msg import String
import rospy
from std_msgs.msg import Int16



def cambiospertinentes(desicionObject,vel):
    #Relacionado con velocidad de avance
    if 'giro' in globals():
        #print ("GIROOOOO",globals()['giro'])
        giroAux=globals()['giro']
    else:
        print ("var2: variable does not exist")
    desicion = desicionObject.data
    imprimeUnMonton(desicion)
    global giro
    global bandgiro
    global switch
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
        if giroAux>90 or giroAux== 90:
            giro=giro+2
            print("Izquierda")
        if giroAux<90:
            giro=90
            print("Vamos recto")
    #Giro a la derecha
    if desicion == "d":
        if giroAux>90:
            giro=90
            print("Vamos recto")
        if giroAux<90 or giroAux == 90:
            giro=giro-2
            print("Derecha")
    if desicion == "stop" or switch==1:
        vel=0
        giro=90
    if desicion=="inter":
        print("Interseccion")
        publisher(90,-90)
        time.sleep(0.25)

    if desicion== "esquiva":
        esquive()

        ##########################################################
        #AQUI METAN TODO LO DEL ESQUIVE
        ##########################################################
        print("esquivando")
        for i in range(200):
            publisher(390,-120)
            print("alteramos")
        for i in range(250):
            publisher(-50,-120)
        #switch=1
        
        

    return vel,giro
#Subscriber

def imprimeUnMonton(string):
    for i in range(10):
        print("Valor que recibe el subscriber:",string)

def callbacksubs(data):
    imprimeUnMonton(data)
    vel=-90
    #imprimeUnMonton(data.data)
    vel,giro=cambiospertinentes(data,vel)
    publisher(giro,vel)
    print("cicloterminado","Vel:",vel,"Giro",giro)



def subscriber():
    topicToSub="/instruccionesAutomodel"
    rospy.Subscriber(topicToSub,String,callbacksubs, queue_size = 1, buff_size = 1)



#Publisher de la velocidad y el pues, Giro

def publisher(giro,velocidad):
    topicoToPublish="/AutoModelMini/manual_control/speed"
    topicoToPublish2="/AutoModelMini/manual_control/steering"
    publishervel = rospy.Publisher(topicoToPublish,Int16,queue_size=1)
    publishergiro = rospy.Publisher(topicoToPublish2,Int16,queue_size=1)
    publishervel.publish(velocidad)
    publishergiro.publish(giro)
    #print("Publicado")

#Obviamente este es el main  y si, si es necesario
if __name__ == "__main__":
    global giro
    global bandgiro
    global switch
    giro=90
    bandgiro=0
    switch=0
    rospy.init_node("movimientovehiculo")
    subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
