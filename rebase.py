import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from time import sleep
import numpy as np
automatico = True
estado= 0
estadoCU = 0
pil = 90
def callback(data):
    global pil
    #Aquie va lo de los angulos y pa publicacion
    #print(data.data)
    res = data.data
    res = 1450-res
    pil =((180.0/1100.0)*float(res))+90
    if automatico == True:
        print(pil)
        dire.publish(int(pil))
    #print(int(pil))
    
def laser(data):
    global estado
    global automatico
    medida= data.ranges[0:35]
    medidaCU = data.ranges[270:290]
    datosCU = np.array(medidaCU)
    datos = np.array(medida)
    filter_dataR = datosCU[datosCU<0.45]
    filter_data_CU = datos[datos<1.4]
    filter_data=datos[datos<0.9]
    #print(medida)
    #print(data.ranges[0])
    if pil>110 :
        if estadoCU==0:
            if len(filter_data_CU)>0:
                pub.publish(0)
                automatico = False
                sleep(1)
                dire.publish(160)
                estadoCU == 2
        if estadoCU==2:
            pub.publish(-100)
            if len(filter_dataR)>0:
                pass



        
    else:

        if estado == 0:
            if len(filter_data)>0:
                pub.publish(0)
                estado = 1
                automatico = False
                sleep(2)
        if estado == 1:
        
            dire.publish(180)
            pub.publish(-50)
            print(data.ranges[308])
            if data.ranges[308]< 0.55:
                estado=2
        if estado ==2:
            dire.publish(20)
            pub.publish(-400)
            sleep(2)
            automatico = True
            pub.publish(-100)
            estado = 3
        if estado == 3:
            lateral=data.ranges[260:270]
            lateral = np.array(lateral)
            lateral_filter = lateral[lateral<0.45]
            if len(lateral_filter) >0:
                estado = 4
                automatico = False
        if estado == 4:
            dire.publish(0)
            pub.publish(-400)
            sleep(2.5)
            automatico=True
            pub.publish(-100)
            estado = 0





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