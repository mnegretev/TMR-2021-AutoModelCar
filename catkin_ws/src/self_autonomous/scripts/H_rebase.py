#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from time import sleep
import numpy as np

automatico = True
estado= 0
vel=0



def callback(data):
    global pil
    #Aquie va lo de los angulos y pa publicacion
    #print(data.data)
    pil = data.data
    #res = 1450-res
    #pil =((180.0/1100.0)*float(res))+90
    if automatico == True:
        #print(pil)
        dire.publish(int(pil))
    #print(int(pil))
    
def laser(data):
    global estado
    global automatico
    global vel

    if vel==0:
        pub.publish(-350)
    else:
        pub.publish(0)

    rango_frontal=np.concatenate((data.ranges[345:360], data.ranges[0:16]))
    datos_frontal=np.array(rango_frontal)
    filter_data_frontal=datos_frontal[datos_frontal <=1]

    rango_frontal2=data.ranges[305:310]
    datos_frontal2=np.array(rango_frontal2)
    filter_data_frontal2=datos_frontal2[datos_frontal2 <=1]


    rango_frontal3=data.ranges[280:310]
    datos_frontal3=np.array(rango_frontal3)
    filter_data_frontal3=datos_frontal3[datos_frontal3 <=0.39]
    
    lidar_range=data.ranges[260:310]
    datos=np.array(lidar_range)
    filter_data=datos[datos<1]


    rango_t=data.ranges[240:265]
    datos_t=np.array(rango_t)
    filter_data_t=datos_t[datos_t<12]

    ## Angulos en la curva
    lidar_c1=data.ranges[25:35]
    datos_c1=np.array(lidar_c1)
    filter_dc1=datos_c1[datos_c1<2]

    lidar_c2=np.concatenate((data.ranges[350:360], data.ranges[0:35]))
    datos_c2=np.array(lidar_c2)
    filter_dc2=datos_c2[datos_c2<2]

    lidar_c3=data.ranges[238:310]
    datos_c3=np.array(lidar_c3)
    filter_dc3=datos_c3[datos_c3<1.5]

   

    if estado == 0:
        #print('estado 0')
        #print(len(filter_data_frontal))
        if len(filter_data_frontal) != 0:
            automatico = False
            dire.publish(180)
            estado=1
        if len(filter_dc1)!=0:
            automatico = False
            dire.publish(175)
            estado=6


    if estado ==1:
        #print('estado 1')
        #print(len(filter_data_frontal2))
        if len(filter_data_frontal2) != 0:
            dire.publish(0)
            estado=2
    if estado==2:
        #print('estado 2')
        #print(len(filter_data_frontal3))
        
        if len(filter_data_frontal3) !=0:
            dire.publish(90)
            automatico = True
            estado=3

    if estado==3:
        #print('estado 3')
        #print(len(filter_data))
        
        if len(filter_data)==0:
            automatico = False
            dire.publish(0)
            estado=4

    if estado==4:
        #print('estado 4')
        #print(len(filter_data_t))
        if len(filter_data_t) !=0:
            estado=5
    if estado==5:
        #print('estado 5')
        #print(len(filter_data_t))
        if len(filter_data_t)==0:
            sleep(0.2)
            automatico=True
            vel=0
            estado=0
    if estado==6:
        #print('estado 6')
        #print(len(filter_dc2))
        if len(filter_dc2)==0:
            dire.publish(90)
            estado=7
    if estado==7:
        #print('estado 7')
        #print(len(filter_dc3))
        if len(filter_dc3) !=0:
            estado=8
    if estado==8:
        #print('estado 8')
        #print(len(filter_dc3))
        if len(filter_dc3)==0:
            dire.publish(60)
            sleep(0.7)
            automatico=True
            estado=0


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