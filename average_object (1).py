import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from time import sleep
import numpy as np
from numpy.core.numeric import Inf
from os import confstr

automatico = True
block = 0
def callback(data):
    global pil
    #Aquie va lo de los angulos y pa publicacion
    #print(data.data)
    res = data.data
    res = 1450-res
    pil =((180.0/1100.0)*float(res))+90
    if automatico == True:
        #print(pil)
        dire.publish(int(pil))
    #print(int(pil))
    
def laser(data):
    global block
    global automatico
    # Primer contacto con un objeto por la parte delantera
    if (block == 0):
        # Rango de angulos a monitorear por la parte delantera
        front_range = np.concatenate((data.ranges[0:35], data.ranges[330:360]))

        # Valores mayores a 3.5 metros, indica que no hay objeto cercano
        infinits = len(front_range[front_range > 3.5]) 
        # Porcentaje de infinitos en la recoleccion de datos
        infinite_percentage = (infinits*(len(front_range/100)))
        if (infinite_percentage < 30):
            #Se ha detectado un objeto, ya que el porcentaje de infinitos es minimo
            pub.publish(-50) # Reducir velocidad a 50
            gir.publish(140) # Girar a la izquierda
            sleep(2000) # 
            automatico = True # Activar automatico
            block = 1

     # Segundo contacto con un objeto por la derecha
    if (block == 1):
        # Rango de angulos a monitorear por la derecha
        right_range = data.ranges[260:280]

        # Valores mayores a 3.5 metros, indica que no hay objeto cercano
        infinits = len(right_range[right_range > 3.5]) 
        # Porcentaje de infinitos en la recoleccion de datos
        infinite_percentage = (infinits*(len(right_range/100)))
        
        if (infinite_percentage > 30):
            # Se deja de detectar un objeto, ya que el porcentaje de infinitos es mayor,
            # Desactivar la conduccion automatica
            automatico = False
            block = 2

    if (block == 2):
        # Rango de angulos a monitorear por la derecha
        back_range = data.ranges[215: 225]
        
        # Valores mayores a 3.5 metros, indica que no hay objeto cercano
        infinits = len(back_range[back_range > 3.5]) 
        # Porcentaje de infinitos en la recoleccion de datos
        infinite_percentage = (infinits*(len(back_range/100)))
        # Segundo contacto con un objeto por la derecha
        if (infinite_percentage < 30):
            # Se ha detectado un objeto, ya que el porcentaje de infinitos es minimo
            pub.publish(-50) # Reducir velocidad a 50
            gir.publish(40) # Girar a la derecha
            sleep(2000) # 
            automatico = True # Activar automatico
            block = 0

def listener():
    global dire
    global pub
    pub = rospy.Publisher('AutoModelMini/manual_control/speed', Int16, queue_size = 10)
    gir = rospy.Publisher('AutoModelMini/manual_control/steering',Int16, queue_size = 10)
    rospy.init_node('Convert', anonymous=True)
    rospy.Subscriber("Angulo", Int16, callback)
    sub = rospy.Subscriber('scan',LaserScan, laser)
    rospy.spin()

if __name__ == '__main__':
    listener()