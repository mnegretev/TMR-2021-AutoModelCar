#! /usr/bin/env python
import rospy
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16 

flag = False
f_imu = 50.0 
h = 1.0/f_imu
yaw = 0.0
yaw0 = 180.0
yaw_h = 0.0
Dyaw = 0.0
FTY = True
D = 29.79
step = 0
d_ref = 45.0
u = 90
v = -250

ransac = RANSACRegressor()

def steer_control0(m,b,th_min):
	global iex_h
	gamma = th_min-(3*np.pi/2)
	d = abs(b/np.sqrt(m**2+1))
	ex = (d-d_ref)
	th = -np.arctan(m) 
	Kx = 0.06
	Kth = 1.25 
	u = 90-np.arctan(Kx*ex+Kth*th)*(180/np.pi) 
	return(u,d)

def fit_ransac(R,rmax):
	L = len(R)
	X = []
	Y = []
	for i in range (0,L):
		if (R[i]<rmax) and (R[i]>0.11):
			r = R[i]*100
			th = i*(np.pi/180.0)
			X.append(r*np.cos(th))
			Y.append(r*np.sin(th))
	L = len(X)
	X = np.reshape(np.array(X),(L,1))
	Y = np.reshape(np.array(Y),(L,1))
	reg = ransac.fit(X,Y)
	x1 = 0
	x2 = 1
	X_m = np.reshape(np.array([x1, x2]),(2,1))
	y1,y2 = reg.predict(X_m)
	m = (y2-y1)/(x2-x1)
	b = y2-m*x2
	return(m,b)

def extract_outfits(R,m,b):
	k = 0
	S = []
	for s in R:
		th_s = k*(np.pi/180.0)
		x_s = 100.0*s*np.cos(th_s) 
		y_s = 100.0*s*np.sin(th_s)
		d_s = abs((m*x_s-y_s+b)/np.sqrt(m**2+1))		
		if (s<3.0) and (d_s<=5.0): S.append(3.0)
		else: S.append(s)
		k = k+1
	return S

def measure_D(rho_a,rho_b,d,r_min):
	D_est = 0.0
	rho_l = d/np.cos(np.pi/6.0)
	if (abs(rho_a-rho_l)<=5.0) and (abs(rho_b-rho_l)<=5.0) and (abs(r_min*100.0-d)<5.0):
		D_est = np.sqrt(rho_a**2+rho_b**2-2*rho_a*rho_b*np.cos(np.pi/3.0))
	return D_est 

def aling_E(R,d):
	gamma = 2*np.arctan(0.2/d)	
	return gamma

def callback_R(data0):
	global u, v
	global step, FTY, flag
	R=[]
	for r in data0.ranges:
		if (r>=3.0) or (r<=0.11): r = 3.0
		R.append(r)
	r270 = R[269]
	r_min = np.amin(R)
	th_min = np.argmin(R)*(np.pi/180.0) 
	rho_a = R[239]*100.0
	rho_b = R[299]*100.0
	r_est = np.amin(R[179:224])
	rmax = 3.0
	m,b = fit_ransac(R,rmax) 		
	S = extract_outfits(R,m,b)	
	r180 = np.amin(S[149:209])
	th180 = (149+np.argmin(S[149:209]))*(np.pi/180.0) 
	R0_i = S[0:29] 		
	R0_d = S[329:359] 	
	R0 = np.concatenate((R0_i,R0_d))
	r0 = np.amin(R0)
	if (step == 0):
		u,d = steer_control0(m,b,th_min)
		Dest = measure_D(rho_a,rho_b,d,r_min)
		if (Dest>0.0) and (r_est<0.5):
			flag =True
			v = -250
		if (flag==True) and (r270>=0.25) and (r270<=0.35):
			FTY = False
			step = step+1
	if (step == 1):
		print('Inicio de la maniobra')
		u = 0
		v = 250
		if (Dyaw<=-D):
			step = step + 1
	if (step == 2):
		u = 180
		v = 250
		if (Dyaw>=0.0) or (r180<=0.45): #0.42 ###
			step = step + 1
	if (step == 3):
		if (Dyaw<-1): u = 0
		if (Dyaw>=-1): u = 90
		v = -250
		if (r0<=0.7): #0.6
			step = step + 1
	if (step == 4):
		if (Dyaw<-3): step = 2
		if (Dyaw>=-3):
			print('Fin de la maniobra')
			u = 90
			v = 0
	print('u ',u)
	print('step ',step)
	print('****************************')
	Vpub.publish(v) 
	Spub.publish(u)	

def callback_imu(data2):
	global yaw0, yaw_h, Dyaw
	gz = data2.angular_velocity.z
	yaw = (yaw_h+h*gz)		
	if (yaw>60*np.pi) or (yaw<-60*np.pi): 
		yaw = 0.0
		yaw0 = 0.0
	yaw_h = yaw
	if (FTY==True): yaw0 = yaw
	Dyaw = (yaw0-yaw)*(180.0/np.pi)
		
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_04.py")
	rospy.init_node('TMR_04',anonymous=True)
	Vpub = rospy.Publisher('/AutoModelMini/manual_control/speed',Int16,queue_size=15)				 
	Spub = rospy.Publisher('/AutoModelMini/manual_control/steering',Int16,queue_size=15)				 															 							
	rospy.Subscriber('/scan', LaserScan, callback_R)
	rospy.Subscriber('/AutoModelMini/imu', Imu, callback_imu)	
	rospy.spin()

