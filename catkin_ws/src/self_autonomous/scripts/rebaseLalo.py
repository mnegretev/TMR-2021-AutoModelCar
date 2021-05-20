#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from time import sleep
automatico = True
estado= 0

def callback(data):
    global pil
    #Aquie va lo de los angulos y pa publicacion
    #print(data.data)
    pil = data.data
    #res = 1450-res
    #pil =((180.0/1100.0)*float(res))+90
    if automatico == True:
    
        dire.publish(int(pil))
    #print(int(pil))
    
def laser(data):
    
    global estado
    global automatico
    flag = False
    pub.publish(-350)
    if estado == 0: 
        flag =False                        ###Buscando el pinshi objeto
        automatico = True
        #print("Andamo buscando el pinshi objeto")
        if data.ranges[350] < 1.8 or data.ranges[0]<1.8 or data.ranges[10]<1.8:
            automatico = False
            estado = 1
        if data.ranges[30] <2  or data.ranges[25] <2: 
            automatico = False
            estado=4
        """if data.ranges[329] < 1.5 or data.ranges[327] <1.5:
            automatico = False 
            estado = 7
"""
    if estado == 1:                         ###Cambiando de carril en Rectas
        #print("Andamo en el estao 1")
        dire.publish(110)
        pub.publish(-300)
        flag =False 
        #print(data.ranges[305])
        if data.ranges[350] < 0.4 or data.ranges[0]<0.4 or data.ranges[10]<0.4:
            
            pub.publish(0)
            sleep(.5)
            dire.publish(170)
            pub.publish(-100)
            
        if data.ranges[325] < 1.5 or data.ranges[327] < 1.5:
            estado=2



    if estado ==2:                          ###Mantener en el carril izquierdo
        #
        flag =False 
        pub.publish(-350)
        #print("Andamo en el estao 2")
        automatico = True
        #dire.publish(0)
        #pub.publish(-350)
        #sleep(2)
        #print(data.ranges[270])
        if data.ranges[270] < 0.5:
            sleep(1)
            estado = 3

    if estado == 3:                         ###Cambio de carril derecho
        #print("Andamo en el estao 3")
        automatico = False 
        dire.publish(10)
        pub.publish(-350)
        sleep(1.7)
        estado = 0
        flag =False 
    if estado == 4:                         ###Cambiando de carril  en Curva
        #print("Andamo en el estao 4")
        
        pub.publish(-350)

        if pil > 115:
            #print("entre")
            dire.publish(180)
            sleep(.7)
            flag = True
            #print("sali")
        else:
            dire.publish(140)
        #print(data.ranges[305])
        if data.ranges[325] < 1.3 or data.ranges[327] < 1.3:
            estado=5

    if estado ==5:                          ###Mantener en el carril izquierdo
        #
        #print("Andamo en el estao 5")
        #sleep(0.3)
        automatico = True
        #dire.publish(0)
        #pub.publish(-350)
        #sleep(2)
        #print(data.ranges[270])
        if data.ranges[270] < 0.9:
            sleep(.6)
            estado = 6


    if estado == 6:                         ###Cambio de carril derecho
        #print("Andamo en el estao 6")
        automatico = False 
        
        pub.publish(-350)
        #if flag == True:
           # print("aquiestoy")
          #  dire.publish(60)
         #   sleep(.2)
            #pub.publish(0)
            #dire.publish(180)
            #pub.publish(-350)
        #else:
        dire.publish(10)
        sleep(1)
        estado = 0
        flag == False
"""
    if estado == 7:                     ###Cambiando de carril  en Cruce
        print("Andamo en el estao 7")
        
        pub.publish(-350)

        dire.publish(130)
        #print(data.ranges[305])
        if data.ranges[325] < 1.3 or data.ranges[327] < 1.3:
            estado=8

    if estado == 8:
        pub.publish(-350)
        dire.publish(90)
        sleep(2)
        estado = 6
"""





def listener():
    global dire
    global pub
    pub = rospy.Publisher('AutoModelMini/manual_control/speed', Int16, queue_size = 10)
    dire = rospy.Publisher('AutoModelMini/manual_control/steering',Int16, queue_size = 10)
    rospy.init_node('Convert', anonymous=True)
    rospy.Subscriber("Angulo", Int16, callback)
    sub = rospy.Subscriber('scan',LaserScan, laser)
    rospy.spin()

if __name__ == '__main__':
    sleep(2.5)
    listener()
