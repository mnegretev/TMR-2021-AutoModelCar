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
    global pub
    global pub2
    global automatico
    global lidar270
    global lidar280
    global caso

    #if automatico==True:
    #    print("a")
    #else:
    #    print("m")

    lidar270=data.ranges[265]
    lidar_range=data.ranges[237:300] #237:303
    lidar_range2=data.ranges[293:303] #290:300
    lidar_range3=data.ranges[325:335]
    lidar_range4=data.ranges[19:30]

    lidar_range5=np.concatenate((data.ranges[345:360], data.ranges[0:16]))

    if caso==0:
        pub.publish(-600)
        if lidar270<0.35:
             #print("Deteccion de objeto")
             caso=1
    elif caso==1:
        datos=np.array(lidar_range)
        filter_data=datos[datos<0.35]
        #print(len(filter_data))
        if len(filter_data)==0:
            dire.publish(90)
            pub.publish(300)
            caso=2

    elif caso==2:
        datos=np.array(lidar_range2)
        filter_data=datos[datos<0.4]
        #print(len(filter_data))
        if len(filter_data)!=0:
            automatico=False
            dire.publish(0)
            pub.publish(-350)
            caso=3

    elif caso==3:
        datos=np.array(lidar_range3)
        filter_data=datos[datos<0.4]
        #print(len(filter_data))
        if len(filter_data)!=0:
            dire.publish(180)
            caso=4
    elif caso==4:
        print('caso: ',caso)
        datos=np.array(lidar_range4)
        filter_data=datos[datos<1]
        print(len(filter_data))
        if len(filter_data)==0:
            dire.publish(0)
            pub.publish(200)
            sleep(1.5)
            pub.publish(0)
            dire.publish(90)
            caso=5





                




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