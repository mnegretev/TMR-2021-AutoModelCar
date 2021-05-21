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
    global giro
    global bandgiro
    if desicion == 's':
        if vel>0 or vel==0:
            vel=vel+30
            #print("Velocidad aumentada",vel)
        if vel<0:
            vel=0
            #print("STAAAAAAAAAAAAAAAAP")
    #Relacionado con velocidad de reversa
    if desicion == "w":
        if vel>0:
            vel=0
            # print("STAAAAAAAAAAAAAAAAP")
        if vel<0 or vel==0:
            vel=vel-30
            # print("Reversa aumentada",vel)
    #Relacionado con giro a la izquierdo
    if desicion == "a":
        if giroAux>90 or giroAux== 90:
            giro=giro+2
            # print("Izquierda")
        if giroAux<90:
            giro=90
            # print("Vamos recto")
    #Giro a la derecha
    if desicion == "d":
        if giroAux>90:
            giro=90
            # print("Vamos recto")
        if giroAux<90 or giroAux == 90:
            giro=giro-2
            # print("Derecha")
    if desicion == "stop":
        vel=0
        giro=90
    if desicion=="inter":
        print("Interseccion")
        publisher(90,-90)
        time.sleep(0.25)
    if desicion == "estacionate":
        publisher(90,-90)
        #time.sleep(0.15)
        cont = 0
        estacionarse(cont)
    return vel,giro
#Subscriber




def imprimeUnMonton(string):
    for i in range(10):
        print("Valor que recibe el subscriber:",string)

def callbacksubs(data):
    #imprimeUnMonton(data)
    vel=-90
    #imprimeUnMonton(data.data)
    vel,giro=cambiospertinentes(data,vel)
    publisher(giro,vel)
    #print("cicloterminado","Vel:",vel,"Giro",giro)



def subscriber():
    topicToSub="/instruccionesAutomodel"
    rospy.Subscriber(topicToSub,String,callbacksubs, queue_size = 1, buff_size = 1)


def estacionarse(cont):
    if cont==0:
        vel=300
        giro=-30
        publisher(giro,vel)
        print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        cont=cont+1
        time.sleep(1)
    if cont==1:
        vel=300
        giro=-30
        publisher(giro,vel)
        print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        cont=cont+1
        time.sleep(3)
    if cont==2:
        vel=220
        giro=220
        publisher(giro,vel)
        print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        time.sleep(3.3)
        cont=cont+1
    if cont==3:
        vel=-130
        giro=-30
        publisher(giro,vel)
        print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        time.sleep(1.5)
        cont=cont+1
    if cont==4:
        vel=170
        giro=200
        publisher(giro,vel)
        print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        time.sleep(1.5)
        cont=cont+1
    if cont==5:
        vel=0
        giro=90
        publisher(giro,vel)
        print("YA ME CANSE Oropeza","Vel:",vel,"Giro",giro,"Contador",cont)



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
    giro=90
    bandgiro=0
    rospy.init_node("movimientovehiculo")
    subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
