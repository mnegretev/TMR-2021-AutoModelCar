#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import numpy as np
from time import sleep

dst = None
dst1= None
dst2 = None
#pts1 = None
#Pts2 = None
QUEUE_LENGTH=50

id_imagen = 1
automatico=True
lastError=0
paso = 0
pub = None
pub2 = None
b=0
contlaser=0
caso=0

def callback(data):

    #Aquie va lo de los angulos y pa publicacion
    #print(data.data)
    res = data.data
    res = 1450-res
    pil =((180.0/1100.0)*float(res))+90
    if automatico == True:

        dire.publish(int(pil))
    #print(int(pil))
    
def laser(data):
    global paso 
    global laser
    global pub
    global pub2
    global laser1
    global cajaF2
    global cajaA2
    global cajaFi
    global cajaAi
    global b
    global automatico
    global lidar270
    global lidar280
    global caso

    #if automatico==True:
    #    print("a")
    #else:
    #    print("m")

    lidar270=data.ranges[265]
    lidar_range=data.ranges[237:303]
    lidar165=data.ranges[165]
    lidar170=data.ranges[170]
    lidar180=data.ranges[170:181]

    if b == 0:
        #pub2.publish(1450)
        # publicando velocidad inicial
        pub.publish(-300)

        if caso==0:
            if lidar270<0.35:
                #print("Deteccion de objeto")
                caso=1
        if caso==1:
            datos=np.array(lidar_range)
            filter_data=datos[datos<0.35]

            #print(len(filter_data))
            #if len(filter_data)==1:
                #print(filter_data)
            if len(filter_data)==0:
                caso=2
        if caso==2:
            if lidar270<0.35:
                pub.publish(50)
                sleep(1)
                #print("Deteccion del segundo obstaculo")
                caso=3
                b=1
                automatico=False

    elif b==1:
        if paso == 0:
            #print 'Paso 0'
            dire.publish(0)
            pub.publish(50)
            paso=1
        elif paso==1:
            #print 'Paso 1'
            if lidar165<=0.8: 
                dire.publish(180) #antes era 900
                paso=2
        elif paso==2:
            #print 'Paso 2'
            if lidar170 <=0.5:
                pub.publish(0)
                sleep(0.5)
                dire.publish(60)
                pub.publish(-50)
                paso=3
        elif paso==3:
            #print 'Paso 3'
            datos=np.array(lidar180)
            filter_data=datos[datos<0.65]
            #print(len(filter_data))
            if len(filter_data) ==0:
                dire.publish(90)
                pub.publish(0)
                paso=4




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
    listener()