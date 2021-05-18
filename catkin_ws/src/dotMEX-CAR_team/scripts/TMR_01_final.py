#! /usr/bin/env python
# Nodo para la prueba: Navegacion SIN obstaculos
# Equipo: dotMEX-CAR 2021
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16
