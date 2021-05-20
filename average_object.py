from os import confstr
import numpy as np
from numpy.core.numeric import Inf
block = 0

# Primer contacto con un objeto por la parte delantera
if (block == 0):
    # Rango de angulos a monitorear por la parte delantera
    #front_range = np.concatenate((data.ranges[0:35], data.ranges[330:360]))
    front_range = np.array([0.5, 0.6, 0.7, 0.8, Inf, 1, 1.4, 4, 0.3])

    # Valores mayores a 3.5 metros, indica que no hay objeto cercano
    infinits = front_range[front_range > 3.5] 
    # Porcentaje de infinitos en la recolección de datos
    infinite_percentage = (infinits*(len(front_range/100)))
    if (infinite_percentage < 30):
        # Se ha detectado un objeto, ya que el porcentaje de infinitos es minimo
        # pub.publish(-50) # Reducir velocidad a 50
        # gir.publish(140) # Girar a la izquierda
        # sleep(2000) # 
        # automatico = True # Activar automatico
        block = 1

 # Segundo contacto con un objeto por la derecha
if (block == 1):
    # Rango de angulos a monitorear por la derecha
    #right_range = data.ranges[260:280]
    right_range = np.array([0.5, 0.6, 0.7, 0.8, Inf, 1, 1.4, 4, 0.3])

    # Valores mayores a 3.5 metros, indica que no hay objeto cercano
    infinits = right_range[right_range > 3.5] 
    # Porcentaje de infinitos en la recolección de datos
    infinite_percentage = (infinits*(len(right_range/100)))
    
    if (infinite_percentage > 30):
        # Se deja de detectar un objeto, ya que el porcentaje de infinitos es mayor,
        # Desactivar la conduccion automatica
        # automatico = False
        block = 2

# Tercer contacto por la parte trasera
if (block == 2):
    # Rango de angulos a monitorear por la derecha
    #back_range = data.ranges[215: 225]
    back_range = np.array([0.5, 0.6, 0.7, 0.8, Inf, 1, 1.4, 4, 0.3])

    # Valores mayores a 3.5 metros, indica que no hay objeto cercano
    infinits = back_range[back_range > 3.5] 
    # Porcentaje de infinitos en la recolección de datos
    infinite_percentage = (infinits*(len(back_range/100)))
    # Segundo contacto con un objeto por la derecha
    if (infinite_percentage < 30):
        # Se ha detectado un objeto, ya que el porcentaje de infinitos es minimo
        # pub.publish(-50) # Reducir velocidad a 50
        # gir.publish(40) # Girar a la derecha
        # sleep(2000) # 
        # automatico = True # Activar automatico
        block = 0
