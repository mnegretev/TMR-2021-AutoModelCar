from sensor_msgs.msg import LaserScan
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

#pts1 = None
#Pts2 = None
pub = None
QUEUE_LENGTH=50

##pendR=0
#f = open ('datos.txt','w+')






def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def apply_smoothing(image):
    
    #return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
    #return cv2.bilateralFilter(image, 15, 75,75)
    #return cv2.medianBlur(image,31)
    return cv2.threshold(image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


def detect_edges(image, low_threshold=50, high_threshold=150):
    return cv2.Canny(image, low_threshold, high_threshold)

def filter_region(image, vertices):
   
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
    return cv2.bitwise_and(image, mask)

    
def select_region(image):
   
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.1, rows*0.95]
    top_left     = [cols*0.1, rows*0.01]
    bottom_right = [cols*0.9, rows*0.95]
    top_right    = [cols*0.9, rows*0.01] 
    
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)

def hough_lines(image):
    
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
  ##  global pendR
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
       
    left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
    right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
    
    return left_lane, right_lane # (slope, intercept), (slope, intercept)

def make_line_points(y1, y2, line):

    if line is None:
        return None
    
    slope, intercept = line
    
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return ((x1, y1), (x2, y2))

def lane_lines(image, lines):

    left_lane, right_lane = average_slope_intercept(lines)
    
    y1 = image.shape[0] 
    y2 = y1*0.1         

    left_line  = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)
    
    return left_line, right_line

def draw_lane_lines(image, lines, color=[255, 255, 255], thickness=8):
    
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
        global f
        LineaR = [[0,0],[0,0]]
        LineaL = [[0,0],[0,0]]
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
                line = tuple(map(tuple, line)) 
            return line




        left_line  = mean_line(left_line,  self.left_lines)
        right_line = mean_line(right_line, self.right_lines)
        
    
        LineaR = right_line
        LineaL = left_line

        if LineaR == None:
            LineaR = ((0,0),(0,0))
        if LineaL == None:
            LineaL = ((0,0),(0,0))
        #print(LineaR)
        #print(LineaL)
        #if  medida == "inf":
        ##        v2            v3          v4              v6          v7          v8
        global gir
        gir = (0.05274671*float(LineaL[0][0]))+(0.03002426*float(LineaL[0][1])) + (0.03945892*float(LineaL[1][0]))+(0.04439124* float(LineaR[0][0])) + (-0.64796478* float(LineaR[0][1]))+ (0.15142310*float(LineaR[1][0]))
        gir = (-(gir))+90
    
    
       # else:
        #    vel.publish(0)            

    


        #f.write(str(p1il)+","+str(LineaL)+str(LineaR)+"|"+"\n")
          
##        if right_line is not None & left_line is not None:
  ##          dire.publish(90)

    ##        vel.publish(-120)                    

        return draw_lane_lines(image, (left_line, right_line)),left_line, right_line


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback_img)
    def callback_img(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            crop_img = cv_image[330:480, 20:620]
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            frame = gray.copy()
            cv2.imshow("Original", frame)
            detector = LaneDetector()
            img_bco_negro, left_line, right_line = detector.process(frame)
            cv2.imshow("salida", img_bco_negro)


        except CvBridgeError as e:
            print(e)

        cv2.waitKey(3)


def callback(data):
    
    global medida

    medida = data.ranges[0]

    medidac = data.ranges[20]

    print('lidar r', medida)
    print('lidar c', medidac)
    #print('lidar d', medidad)


    
    vel.publish(-250)   
    dire.publish(gir)   

    if (medida < 0.9 and medida > 0.88):

     ##   vel.publish(-220)
        dire.publish(179)
        time.sleep(1.5)
       
        dire.publish(90)
        time.sleep(0.62)


        dire.publish(10)
        time.sleep(1)
        dire.publish(90)
        time.sleep(1)


    if (medidac <= 0.999 and medidac >= 0.97):

        vel.publish(100)
        time.sleep(2)
        dire.publish(179)
        vel.publish(-200)   
        time.sleep(3.5)
        dire.publish(90)
        time.sleep(3)
        dire.publish(10)
        time.sleep(2)
        dire.publish(170)
        time.sleep(5)
    else:
        vel.publish(-250)

def Ansrec(data):
    global pil
    pil = data.data

def start(args):
    global vel
    global dire
    global sub
    #sub = rospy.Subscriber('Angulo',Int16, Ansrec)
    vel = rospy.Publisher('AutoModelMini/manual_control/speed', Int16,queue_size = 10)
    dire = rospy.Publisher('AutoModelMini/manual_control/steering', Int16,queue_size = 10)
    sub = rospy.Subscriber('scan', LaserScan, callback)
    ic = image_converter()
    rospy.init_node('scan_values','image_converter', anonymous=True)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':

    start(sys.argv)
    #f.close()  
