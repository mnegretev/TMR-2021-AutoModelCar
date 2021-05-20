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

from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int32

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge, CvBridgeError

seven_sides = ["angle_1", "angle_2", "angle_3", "angle_4", "angle_5", "angle_6", "angle_7"]

angles = {
	"angle_1": 165,
	"angle_2": 135,
	"angle_3": 105,
	"angle_4": 90,
	"angle_5": 75,
	"angle_6": 45,
	"angle_7": 15
}

class SimpleNeuralNetwork:
	
	def __init__(self, weights, layers, single_output=False, activation="sigmoid"):
		self.weights       = weights
		self.layers        = layers
		self.activation    = activation
		self.single_output = single_output
		self.output_names  = []
		for index in range(len(layers)):
			self.output_names.append("CLASS " + str(index))
	
	def set_output_names(self, names):
		self.output_names = names
	
	def normalize(self, value, limits=(-1, 1), target_limits=(0, 1)):
		limits = (float(limits[0]), float(limits[1]))
		target_limits = (float(target_limits[0]), float(target_limits[1]))
		value = float(value)

		size = limits[1] - limits[0]
		proportion = value - limits[0]
		other_size = target_limits[1] - target_limits[0]

		new_proportion = (proportion * other_size) / size
		return target_limits[0] + new_proportion

	def activation_sigmoid(self, input_value):
		negative_value = np.multiply(input_value, -1)
		output = 1 / (1 + np.exp(negative_value))
		return output
	
	def classify_image(self, any_image):
		crop_image  = np.array(any_image[219:378, 0:640])
		gray_image  = cv2.cvtColor(crop_image, cv2.COLOR_BGR2GRAY)
		array_image = np.asarray(gray_image)
		
		row    = np.array(np.arange(0, 160, 10),  dtype=np.intp)
		column = np.array(np.arange(0, 640, 10), dtype=np.intp)
		
		pixels = np.copy(array_image)
		pixels = pixels[row[:, np.newaxis], column]
		pixels = pixels.flatten()
		
		input_values = pixels
		pixel = 0
		
		for layer in range(1, len(self.layers)):
			input_size = self.layers[layer - 1] + 1
			outputs = []
			for neuron in range(self.layers[layer]):
				print("SIZES")
				print(input_size)
				temporal_weights = self.weights[pixel:pixel + input_size]
				print(temporal_weights)
				pixel += input_size
				transposed = np.transpose(np.append([1], input_values))
				print("LENS")
				size_w = len(temporal_weights)
				size_i = len(transposed)
				print(temporal_weights)
				if size_w < size_i:
					difference = size_i - size_w
					for index in range(difference):
						temporal_weights += [0.5]
				print(temporal_weights)
				print(len(temporal_weights))
				print(len(transposed))
				weighted_sum = np.multiply(temporal_weights, transposed).sum(axis=0)
				output = self.activation_sigmoid(weighted_sum)
				outputs.append(output)
			input_values = outputs
		
		if self.single_output:
			output = outputs[0]
			print("OUTPUT:     " + str(output))
			output = self.normalize(output, (0,1), (-1,1))
			print("NORMALIZED: " + str(output))
			print("++++++++++++++++++++++++++")
			return output * 5
		
		max_output = max(outputs)
		index = outputs.index(max_output)
		class_name = self.output_names[index]

		print("CLASS NAME: " + class_name)
		print("+++++++++++++++++++++++++++++++")

		return class_name

PESOS_SIETE_LADOS = np.genfromtxt('weights_seven_sides.csv', delimiter=',')
RED_SIETE_LADOS = SimpleNeuralNetwork(PESOS_SIETE_LADOS,  [1024, 5, 3, 7])
RED_SIETE_LADOS.set_output_names(seven_sides)

class input_printer:
  
	def __init__(self):
		self.img_counter = 0
		self.bridge = CvBridge()
		self.current_img_path = None
		self.sub_img = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.img_callback)
		self.pub_speed = rospy.Publisher('AutoModelMini/manual_control/speed', Int16, queue_size=10)
		self.pub_steer = rospy.Publisher('AutoModelMini/manual_control/steering', Int16, queue_size=10)

	def start(self):
		pass

	def img_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			angle = clasificar_imagen(cv_image, 7)
			self.pub_steer.publish(angles[angle])
			self.pub_speed.publish(-100)
		except:
			pass
    
def main(args):
	ip = input_printer()
	rospy.init_node('input_printer', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
