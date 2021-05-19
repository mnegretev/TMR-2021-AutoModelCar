#! /usr/bin/env python
# Nodo para la prueba: Navegacion SIN obstaculos
# Equipo: dotMEX-CAR 2021
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16

bridge = CvBridge()
FT = 0
l = 60 
x_ref = 120
x1 = 120
x2 = 120
x1_h = 120
x2_h = 120
th_h = 0.0
h_vis =1.0/30.0
u = 90
v = -1000 #-1000

def tip(imagenN):
	H=np.array([[-7.98362236e-02,-4.79765416e-01,1.23982766e+02],[1.05493081e-03,-1.61957424,3.77026220e+02],[7.48177877e-06,-4.86995945e-03,1.0]]) 
	imagenH = cv2.warpPerspective(imagenN, H, (200,300),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH

def roi_zone(x):
	assert (x>=0) and (x<=199), 'x out of limits'
	if (x>130) and (x<=199):
		y = int(round(-1.6875*x+499.8125))
	if (x>=69) and (x<=130):
		y = 280
	if (x>=0) and (x<69):
		y = int(round(1.7375*x+160.0))
	return y

def vec_create(x,stride):
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	return xv

def line_detector(imagen0,x1,l,side):
	K = True
	stride = 6
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stride)
	while (K==True):
		if (y1+stride>280): m = 280-y1
		else: m = stride
		for j in range(y1+m,y1-stride,-1):
			for i in x1v:
				if imagen0[j][i]==255:
					x1 = i
					y1 = j
					K = False
					break
			x1v = vec_create(x1,stride)
			if (K==False): break
		if (K==True): 
			x1 = x1-1*side
			y1 = roi_zone(x1)
	x2 = x1
	x2v = vec_create(x2,stride)
	for j in range(y1-1,y1-l,-1):
		for i in x2v:
			if imagen0[j][i]==255:
				x2 = i
				y2 = j
				K = False
				break
		x2v = vec_create(x2,stride)			
	return x1,y1,x2,y2

def callback_V(data0):
	global u, v
	global FT, th_h
	global x1, x2, x1_h, x2_h
	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 	
	imagenG = cv2.cvtColor(imagen0,cv2.COLOR_BGR2GRAY) 					
	imagenT = tip(imagenG)
	_,imagenB = cv2.threshold(imagenT,75,255,cv2.THRESH_BINARY)
	imagenF = cv2.Sobel(imagenB,cv2.CV_8U,1,0, ksize=3)
	y1 = 0
	y2 = 0
	if (FT<=90):
		x1 = 180
		FT = FT+1
	else: 
		x1 = x1_h
	x1,y1,x2,y2 = line_detector(imagenF,x1,l,True)
	x1_h = x1
	x2_h = x2

	kx = 0.09187505 
	kth = 0.1993331
	kdth = 0.3
	ex = x1-x_ref
	th = np.arctan2(x2-x1,l)
	dth = (th-th_h)/h_vis
	th_h = th
	u = int(round(90-np.arctan(kx*ex+kth*th+kdth*dth)*(180/np.pi))) 
	print('steering ',u)
	Vpub.publish(v) 
	Spub.publish(u)

if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_01.py")
	rospy.init_node('TMR_01',anonymous=True)												
	Vpub = rospy.Publisher('/AutoModelMini/manual_control/speed',Int16,queue_size=15)				 
	Spub = rospy.Publisher('/AutoModelMini/manual_control/steering',Int16,queue_size=15)
	rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	 						
	rospy.spin()
