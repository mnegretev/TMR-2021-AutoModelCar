#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os, glob
from collections import deque
import csv
import math
import time
dst = None
dst1= None
dst2 = None
#pts1 = None
#Pts2 = None
pub = None
QUEUE_LENGTH=50

id_imagen = 1

lastError=0

if (len(sys.argv)>1):
	debug =True
else:
	debug =False
def save_image(image,volante, debug):
	global id_imagen
	if (debug):
		folder = sys.argv[1]
		# make a separate image to draw lnes and combine with the orignal later
		if not os.path.exists(folder):
			os.makedirs(folder)
		cv2.imwrite(folder+"/"+str(id_imagen)+".jpg",image)
		fd = folder + "/" + 'lista_imgs.csv'
		print(fd)
		fields=[str(id_imagen),volante]
		with open(fd, 'ab') as f:
			writer = csv.writer(f, delimiter=',')
			writer.writerow(fields)
		id_imagen = id_imagen + 1

def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def apply_smoothing(image):
    """
    kernel_size must be postivie and odd
    """
    #return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
    #return cv2.bilateralFilter(image, 15, 75,75)
    #return cv2.medianBlur(image,31)
    return cv2.threshold(image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


def detect_edges(image, low_threshold=50, high_threshold=150):
    return cv2.Canny(image, low_threshold, high_threshold)

def filter_region(image, vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
    return cv2.bitwise_and(image, mask)

    
def select_region(image):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
    """
    # first, define the polygon by vertices
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.1, rows*0.95]
    top_left     = [cols*0.1, rows*0.01]
    bottom_right = [cols*0.9, rows*0.95]
    top_right    = [cols*0.9, rows*0.01] 
    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)

def hough_lines(image):
    """
    `image` should be the output of a Canny transform.
    
    Returns hough lines (not the image with lines)
    """
    return cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=400)

def draw_lines(image, lines, color=[255, 0, 0], thickness=2, make_copy=True):
    # the lines returned by cv2.HoughLinesP has the shape (-1, 1, 4)
    if make_copy:
        image = np.copy(image) # don't want to modify the original
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(image, (x1, y1), (x2, y2), color, thickness)
    return image

def average_slope_intercept(lines):
    left_lines    = [] # (slope, intercept)
    left_weights  = [] # (length,)
    right_lines   = [] # (slope, intercept)
    right_weights = [] # (length,)
    if lines is not None:
	    for line in lines:
	    	#print(line)
	        for x1, y1, x2, y2 in line:
	            if x2==x1:
	                continue # ignore a vertical line
	            aux1 = (y2-y1)
	            aux2 = (x2-x1)
	            slope = float(aux1)/float(aux2)
	            #print(str(aux1) + "<->" + str(aux2) + "<->" +str(slope)) 

	            if abs(slope) < 0.15:
	                continue # ignore a horizontal line
	            intercept = y1 - slope*x1
	            length = np.sqrt((y2-y1)**2+(x2-x1)**2)
	            
	            if slope < 0: # y is reversed in image
	                left_lines.append((slope, intercept))
	                left_weights.append((length))
	            else:
	                right_lines.append((slope, intercept))
	                right_weights.append((length))
	    		'''
	            if (x2 < 300): #iz
	            	left_lines.append((slope, intercept))
	                left_weights.append((length))
	            else:
	            	right_lines.append((slope, intercept))
	                right_weights.append((length))
	            '''
    # add more weight to longer lines    
    left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
    right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
    
    return left_lane, right_lane # (slope, intercept), (slope, intercept)

def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None
    
    slope, intercept = line
    #print(slope)
    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return ((x1, y1), (x2, y2))

def lane_lines(image, lines):

    '''image2 = image.copy()
    image2 = cv2.cvtColor(image2,cv2.COLOR_GRAY2RGB)
    for line in lines:
    	print(line)
    	cv2.line(image2, (line[0][0],line[0][1]),(line[0][2],line[0][3]), [255, 0, 0], 2)
    cv2.imshow('modified2', image2)
    '''
    left_lane, right_lane = average_slope_intercept(lines)
    
    y1 = image.shape[0] # bottom of the image
    y2 = y1*0.1         # slightly lower than the middle

    left_line  = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)
    
    return left_line, right_line

    
def draw_lane_lines(image, lines, color=[255, 255, 255], thickness=8):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
        	cv2.line(line_image, (line[0][0],line[0][1]),(line[1][0],line[1][1]),  color, thickness)
    #return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)
    return line_image
class LaneDetector:
    def __init__(self):
        self.left_lines  = deque(maxlen=QUEUE_LENGTH)
        self.right_lines = deque(maxlen=QUEUE_LENGTH)

    def process(self, image):
        #white_yellow = select_white_yellow(image)
        #gray         = convert_gray_scale(white_yellow)
        #smooth_gray  = apply_smoothing(image)
        #edges        = detect_edges(smooth_gray)
        #ret2,smooth_gray = cv2.threshold(image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        _,smooth_gray = cv2.threshold(image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #cv2.adaptativeThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY+cv2.THRESH_OTSU,11,2)
        edges        = detect_edges(smooth_gray)
        regions      = select_region(edges)
        lines        = hough_lines(regions)
       
        left_line, right_line = lane_lines(image, lines)
        
        def mean_line(line, lines):
            if line is not None:
                lines.append(line)

            if len(lines)>0:
                line = np.mean(lines, axis=0, dtype=np.int32)
                line = tuple(map(tuple, line)) # make sure it's tuples not numpy array for cv2.line to work
            return line

        left_line  = mean_line(left_line,  self.left_lines)
        right_line = mean_line(right_line, self.right_lines)
        return draw_lane_lines(image, (left_line, right_line)),left_line, right_line

			#fin nuevo codigo

#def lineLR(right_line,left_line,img_bco_negro):
#	if(right_line is not None):
#		print("DERECHA")
		#Vuelta a la Izquierda
		#	continue # ignore a vertical line
#		aux1 = (right_line[1][1]-right_line[0][1])
#		aux2 = (right_line[1][0]-right_line[0][0])
#		slope = float(aux1)/float(aux2)
#		der = 0
#		factor = 0
#		for o in range (0,299):
#			if img_bco_negro[75,300+o] != 0:
#				der = o
#		if ((der>=0) and (der <=75)):
#			factor = 50
#		elif ((der>=76) and (der <=150)):
#			factor = 25
#		elif ((der>=151) and (der <=225)):
#			factor = 10
#	
#		print("SLOPE**:",math.degrees(math.atan(slope)))
#		degrees = math.degrees(math.atan(slope))
#
#		if ( (abs(degrees) > 50.0)):
#			giro=1300-factor
#			volante.publish(giro)
#			print("Regresar a la pista: ",giro)
#
#		#if ((abs(slope) >= 0.15) and (abs(slope) < 0.38) ): 
#		if ((abs(degrees) >= 45.0) and (abs(degrees) < 50.0)):
#			#Centrado
#			giro=1450-factor
#			volante.publish(giro)	
#			print("centrado: ",giro)
#		#if ((abs(slope) >= 0.38) and (abs(slope) < 0.4) ): 
#		if ( (abs(degrees) >= 40.0) and (abs(degrees) < 45.0)):
#			giro=1420-factor
#			print("Giro: ",giro)
#			
#			volante.publish(giro)
#			
#		if ( (abs(degrees) >= 35.0) and (abs(degrees) < 40.0)):
#			giro=1390-factor
#			print("Giro: ",giro)
#
#		#if ((abs(slope) >= 0.4) and (abs(slope) < 0.45) ): 
#		if ( (abs(degrees) >= 30.0) and (abs(degrees) < 35.0)):
#			giro=1330-factor
#			print("Giro: ",giro)
#			
#			volante.publish(giro)
#			
#		#if ((abs(slope) >= 0.45) and (abs(slope) < 0.47) ): 
#		if ( (abs(degrees) >= 20.0) and (abs(degrees) < 30.0)):
#			giro=1300-factor
#			print("Giro: ",giro)
#			
#			volante.publish(giro)
#			
#		#if ((abs(slope) >= 0.48) and (abs(slope) < 0.70) ): 
#		if ( (abs(degrees) >= 10.0) and (abs(degrees) < 20.0)):
#			giro=1270-factor
#			print("Giro Brusco: ",giro)
#			volante.publish(giro)
#			
#		if ( (abs(degrees) < 10.0)):
#				giro=1450
#				volante.publish(giro)
#				print("Horizontal: ",giro)
		
	# elif (left_line is not None):
	# 		#Vuelta a la Izquierda
	# 		print("IZQUIERDA")
	# 		#	continue # ignore a vertical line
	# 		aux1 = (left_line[1][1]-left_line[0][1])
	# 		aux2 = (left_line[1][0]-left_line[0][0])
	# 		slope = float(aux1)/float(aux2)
		
	# 		degrees = math.degrees(math.atan(slope))
	# 		print("SLOPE:",degrees)

	# 		if ( (abs(degrees) > 50.0)):
	# 			giro=1550
	# 			print("Regresar a la pista: ",giro)

	# 		#if ((abs(slope) >= 0.15) and (abs(slope) < 0.38) ): 
	# 		if ((abs(degrees) >= 45.0) and (abs(degrees) < 50.0)):
	# 			#Centrado
	# 			giro=1450
	# 			volante.publish(giro)	
	# 			print("centrado: ",giro)
	# 		#if ((abs(slope) >= 0.38) and (abs(slope) < 0.4) ): 
	# 		if ( (abs(degrees) >= 40.0) and (abs(degrees) < 45.0)):
	# 			giro=1480
	# 			print("Giro: ",giro)
				
	# 			volante.publish(giro)
				
	# 		if ( (abs(degrees) >= 35.0) and (abs(degrees) < 40.0)):
	# 			giro=1510
	# 			print("Giro: ",giro)

	# 		#if ((abs(slope) >= 0.4) and (abs(slope) < 0.45) ): 
	# 		if ( (abs(degrees) >= 30.0) and (abs(degrees) < 35.0)):
	# 			giro=1540
	# 			print("Giro: ",giro)
				
	# 			volante.publish(giro)
				
	# 		#if ((abs(slope) >= 0.45) and (abs(slope) < 0.47) ): 
	# 		if ( (abs(degrees) >= 20.0) and (abs(degrees) < 30.0)):
	# 			giro=1570
	# 			print("Giro: ",giro)
				
	# 			volante.publish(giro)
				
	# 		#if ((abs(slope) >= 0.48) and (abs(slope) < 0.70) ): 
	# 		if ( (abs(degrees) >= 10.0) and (abs(degrees) < 20.0)):
	# 			giro=1600
	# 			volante.publish(giro)
	# 			print("Giro Brusco: ",giro)
	# 		if ( (abs(degrees) < 10.0)):
	# 			giro=1450
	# 			volante.publish(giro)
	# 			print("Horizontal: ",giro)

def lineLR_Nuevo(right_line,left_line,img_bco_negro):
	global lastError
	kp= 0.2; #This is the proporational value
	kd= 1; #This is the derivative value

	giro = 1650
	if(right_line is not None):
		#print("DERECHA")
		#Vuelta a la Izquierda
		#	continue # ignore a vertical line
		aux1 = (right_line[1][1]-right_line[0][1])
		aux2 = (right_line[1][0]-right_line[0][0])
		slope = float(aux1)/float(aux2)
		der = 0
		factor = 0
		for o in range (0,599):
			if img_bco_negro[40,o] != 0:
				der = o
		error = (der)-520
		PV = kp * error + kd * (error - lastError);
		lastError = error;
		# elif ((der>=151) and (der <=225)):
		# 	factor = 10
		#print("Distancia:",der)
		#print("Pv:",PV)

	
		#print("SLOPE**:",math.degrees(math.atan(slope)))
		degrees = math.degrees(math.atan(slope))

		if ( abs(PV) < 10.0):
			giro=1450
			#print("Voy derecho: ",giro)
			volante.publish(giro)
			return str(giro)
		if ( abs(PV) < 25.0):
			giro=1400-int(abs(PV*2))
			#print("leve a la izq: ",giro)
			volante.publish(giro)
			return str(giro)
		if ( abs(PV) < 30.0):
			giro=1300-int(abs(PV*2))
			#print("leve mas a la izq: ",giro)
			volante.publish(giro)
			return str(giro)
		if ( abs(PV) < 45.0):
			giro=1300-int(abs(PV*2.5))
			#print("giro a la izq: ",giro)
			volante.publish(giro)
			return str(giro)
		if ( abs(PV) < 50.0):
			giro=1300-int(abs(PV*3))
			#print("giro fuerte a la izq: ",giro)
			volante.publish(giro)
			return str(giro)
		if ( abs(PV) > 50.0):
			giro=1300-int(abs(PV*4))
			#print("giro fuerte a la izq: ",giro)
			volante.publish(giro)
			return str(giro)
	
	elif(left_line is not None):
		#print("IZQUIERDA")
		#Vuelta a la Izquierda
		#	continue # ignore a vertical line
		aux1 = (left_line[1][1]-left_line[0][1])
		aux2 = (left_line[1][0]-left_line[0][0])
		slope = float(aux1)/float(aux2)
		der = 0
		factor = 0
		for o in range (0,599):
			if img_bco_negro[40,o] != 0:
				der = o
		error = (der)-110
		PV = kp * error + kd * (error - lastError);
		lastError = error;
		# elif ((der>=151) and (der <=225)):
		# 	factor = 10
		#print("Distancia:",der)
		#print("Pv:",PV)

	
		##print("SLOPE**:",math.degrees(math.atan(slope)))
		degrees = math.degrees(math.atan(slope))
		if ( abs(PV) > 120):
			volante.publish(1950)
			#return str(1950)

		elif ( abs(PV) < 10.0):
			giro=1450
			volante.publish(giro)
			#print("Voy derecho: ",giro)
			#volante.publish(giro)
			#return str(giro)
		elif ( abs(PV) < 25.0):
			giro=1500+int(abs(PV*2))
			volante.publish(giro)
			#print("leve a la izq: ",giro)
			#volante.publish(giro)
			#return str(giro)
		elif ( abs(PV) < 30.0):
			giro=1500+int(abs(PV*2))
			volante.publish(giro)
			#print("leve mas a la izq: ",giro)
			#volante.publish(giro)
			#return str(giro)
		elif ( abs(PV) < 45.0):
			giro=1500+int(abs(PV*2.5))
			volante.publish(giro)
			#print("giro a la izq: ",giro)
			#volante.publish(giro)
			#return str(giro)
		elif ( abs(PV) < 50.0):
			giro=1500+int(abs(PV*3))
			volante.publish(giro)
			#print("giro fuerte a la izq: ",giro)
			#volante.publish(giro)
			#return str(giro)
		elif ( abs(PV) > 50.0):
			giro=1500+int(abs(PV*4))
			volante.publish(giro)
			#print("giro fuerte a la izq: ",giro)
			#volante.publish(giro)
			#return str(giro)

	#else:
	#	volante.publish(giro)
	#return str(giro)




class image_converter:
#	dst =None
	def __init__(self):
		#self.image_pub = rospy.Publisher("image_topic_2",dst)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback)
		#self.image_pub = rospy.Publisher("image_topic_2",dst)

	def callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
			crop_img = cv_image[330:480, 20:620]
			#cv2.imwrite('imagen7.jpg',crop_img)
			#time.sleep(20)
			gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)


			#Nuevo codigo

			frame = gray.copy()
			detector = LaneDetector()

			img_bco_negro, left_line, right_line = detector.process(frame)

			## Derecho
			der=0
			izq=0
			giro=0
			if ((left_line is None ) and (right_line is None)):
				volante.publish(1520)

			if ((left_line is not None ) and (right_line is not None)):
					
					for o in range (0,299):
						if img_bco_negro[75,300+o] != 0:
							der = 300 + o
						if img_bco_negro[75,300-o] != 0:
							izq = 300 - o

					cv2.line (img_bco_negro, (300,80), (der,80), (255,255,255),3)
					cv2.line (img_bco_negro, (300,80), (izq,80), (255,255,255),3)
					der = der - 300
					izq = 300 - izq
			
					
					dif=der-izq
					desfase=int((abs(dif)*550)/300)
					if abs(desfase)>550:
						desfase=550
					if abs(dif)>10:
						if izq>der:
						 	giro=1450-desfase
						#cargados a la derecha
						else:
							giro=1450+desfase
						if giro >= 1350 and giro <1550:
						#cargados a la izquierda
							volante.publish(giro)

							#print("Giro",giro)
					
			else:					
				#lineLR(right_line,left_line,img_bco_negro)
				lineLR_Nuevo(right_line,left_line,img_bco_negro)
						
				
			#save_image(crop_img,giro,debug)			
	
		except CvBridgeError as e:
			print(e)

def main(args):
	global volante
	ic = image_converter()
	volante = rospy.Publisher('Angulo', Int16, queue_size=10)
	rospy.init_node('image_converter', anonymous=True)
#	self.image_pub = rospy.Publisher("image_topic_2",dst)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
