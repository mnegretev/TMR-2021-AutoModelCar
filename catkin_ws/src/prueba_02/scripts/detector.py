import matplotlib.pylab as plt
import cv2
import numpy as np

def region_of_interest(image, vertices):
	mask = np.zeros_like(image)
	#channel_count = image.shape[2]
	match_mask_color = 255
	cv2.fillPoly(mask, vertices, match_mask_color)
	masked_image = cv2.bitwise_and(image, mask)
	return masked_image

def draw_lines(image, lines):
	image = np.copy(image)
	blank_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
	
	for line in lines:
		for x1, y1, x2, y2 in line:
			cv2.line(blank_image, (x1, y1), (x2, y2), (255, 255, 0), thickness=4)
	
	image = cv2.addWeighted(image, 0.8, blank_image, 1, 0.0)
	return image

def process_image(image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	canny_image = cv2.Canny(gray_image, 100, 200)
	cropped_image = canny_image[219:378, 200:640]
	
	height = cropped_image.shape[0]
	width = cropped_image.shape[1]

	small_region_vertices = [
		(0, height),
		(width/2, height/2),
		(width, height)
	]

	lines = cv2.HoughLinesP(
		cropped_image, rho=6,
		theta=np.pi/60,
		threshold=160,
		lines=np.array([]),
		minLineLength=40,
		maxLineGap=25
	)
	
	image_with_lines = draw_lines(image, lines)
	
	#plt.imshow(image_with_lines)
	#plt.show()
	
	return (image_with_lines, lines)

#image = cv2.imread("image.png")
#result_set = process_image(image)
#lines = result_set[1]
#print(len(lines))
