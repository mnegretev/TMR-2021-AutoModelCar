#!/usr/bin/env python2
import roslib
import sys
import rospy
import cv2
import numpy as np
import glob
import time
import rosnode
import csv
#import matplotlib.pylab as plt

from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int32

from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

from detector import *




#pesos = np.genfromtxt('weights.csv', delimiter=',')
current_values = []
values = []
cont=0

thetas = np.array([
	[ 1.27238108],
	[-0.13525588],
	[ 0.67917269],
	[ 0.06864843],
	[ 0.26515200]
])

class input_printer:
	
	def __init__(self):
		self.img_counter = 0
		self.bridge = CvBridge()
		self.current_img_path = None
		self.sub_img = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.img_callback)
		self.sub_joy = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
		self.pub_speed = rospy.Publisher('AutoModelMini/manual_control/speed', Int16, queue_size=10)
		self.pub_steer = rospy.Publisher('AutoModelMini/manual_control/steering', Int16, queue_size=10)
		self.mode = "joy"
		pass
   
	def start(self):
		pass
	
	def set_mode(self, mode):
		self.mode = mode
  
	def normalize(self, start0, finish0, start1, finish1, value):
		diference0 = finish0 - start0
		diference1 = finish1 - start1
		relationship = diference1 / diference0
		fragment0 = value - start0
		fragment1 = fragment0 * relationship
		return(start1 + fragment1)

	def img_callback(self, data):
		global current_values
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			result_set = process_image(cv_image)
			image_with_lines = result_set[0]
			
			lines = result_set[1]
			size = len(lines)
			
			x1 = 0
			y1 = 0
			x2 = 0
			y2 = 0
			
			for line in lines[0]:
				x1 += line[0]
				y1 += line[1]
				y2 += line[2]
				x2 += line[3]
			current_values = [size, x1, y1, x2, y2]
			
			if self.mode == "drive":
				if size == 0:
					self.pub_steer.publish(180)
					self.pub_speed.publish(-1000)
					return
				
				steering_angle = np.dot(current_values, thetas)
				speed_value = -100
				if (steering_angle >= 0) and (steering_angle <= 90):
					steering_angle *= 1
					speed_value = -100
				steering_angle += 1
				self.pub_steer.publish(steering_angle)
				self.pub_speed.publish(speed_value)
				
		except Exception, e:
			if type(e) == ValueError or type(e) == TypeError:
				self.pub_steer.publish(180)
				self.pub_speed.publish(-1000)
			else:
				print(type(e))

	def joy_callback(self,data):
		if self.mode == "joy":
			steering_angle = self.normalize(-1, 1, 0, 180, data.axes[0])
			speed_value = self.normalize(0, 1, 0, -300, data.axes[1])
			self.pub_steer.publish(steering_angle)
			self.pub_speed.publish(speed_value)
			global current_values
			global values
			if (len(current_values) == 5) and (data.axes[1] != 0):
				current_values.append(steering_angle)
				values.append(current_values)
	
	def stop(self):
		for i in range(100000):
			self.pub_steer.publish(0)
			self.pub_speed.publish(0)
		


def main(args):
  ip = input_printer()
  ip.set_mode("drive")
  rospy.init_node('input_printer', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    ip.stop()
  cv2.destroyAllWindows()
  ip.stop()

if __name__ == '__main__':
    main(sys.argv)
    with open('speed_and_steering.csv', 'w') as csvfile:
      writer = csv.writer(csvfile, delimiter=',')
      for row in values:
        writer.writerow(row)
