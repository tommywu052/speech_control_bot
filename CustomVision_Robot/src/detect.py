#! /usr/bin/python
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import sys
import requests, json
from arm_simulation.msg import BoundingBox
from time import time
from pprint import pprint

# API configuration
URL = 'http://127.0.0.1:8787/image'
content_type = 'application/octet-stream'


height = 640
width = 480
# Configure color streams
try:
	pipeline = rs.pipeline()
	config = rs.config()
	config.enable_stream(rs.stream.color, height, width, rs.format.bgr8, 30)

except:
	sys.stderr.write("Error occurs while configuring camera!!\n")
	sys.exit(-1)


# Initialize ROS configuaration
try:
	topic = rospy.Publisher('BoundingBox', BoundingBox, queue_size=10)
	rospy.init_node('detect', anonymous=True)
except:
	sys.stderr.write("Unable to intialize ROS!!\n")
	sys.exit(-1)



def main(show_img=True):
	rospy.on_shutdown(detect_shutdown)
	pipeline.start(config)
	
	try:
	    while True:
	    	t0 = time()
	        # Wait for a coherent pair of frames: depth and color
	        frames = pipeline.wait_for_frames()
	        color_frame = frames.get_color_frame()
	        if not color_frame:
	            continue

	        color_image = np.asanyarray(color_frame.get_data())
	        #resize_image = cv2.resize(color_image, dsize=(192, 144))
	        print "copy2ndarray time : {}".format(time() - t0)
	        predictions = call_api(color_image)
	        
	        # Show images
	        boundingBoxes = {'red': None, 'yellow': None, 'green': None}
	        for item in predictions:
	        	if item['probability'] > 0.14:
	        		if boundingBoxes[item['tagName']] == None:
	        			boundingBoxes[item['tagName']] = item
	        		elif boundingBoxes[item['tagName']]['probability'] < item['probability']:
	        			boundingBoxes[item['tagName']] = item

	        for k, item in boundingBoxes.items():
	        	if item is not None:
	        		x = int(height * item['boundingBox']['left'])
	        		y = int(width * item['boundingBox']['top'])
	        		w = int(height * item['boundingBox']['width'])
	        		h = int(width * item['boundingBox']['height'])
	        		send_message(item['boundingBox'])
	        		if show_img:
	        			if k == 'red':
	        				cv2.rectangle(color_image, (x,y), (x+w, y+h), (0,0,200), 2)
	        			elif k == 'green':
	        				cv2.rectangle(color_image, (x,y), (x+w, y+h), (0,200,0), 2)
	        			else:
	        				cv2.rectangle(color_image, (x,y), (x+w, y+h), (153,255,255), 2)
	        
	        if show_img:
	        	cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
	        	cv2.imshow('RealSense', color_image)
	        
	        if cv2.waitKey(30) & 0xff == ord('q'):
	        	if show_img:
	        		cv2.destroyAllWindows()
	        	break

	        print "Totol time: {}".format(time() - t0)
	        print "========================================="
	
	finally:
		print("Exit from while loop")


def call_api(image_array):
	#return [{'boundingBox': {'left': 0.25, 'top': 0.25, 'width': 0.5, 'height': 0.5}, 'probability': 0.9}]
	t0 = time()
	img_bytes = cv2.imencode('.jpg', image_array)[1].tobytes()
	t1 = time(); print "encode  time : {}".format(t1 - t0)
	try:
		#response = requests.post(URL, headers={'Prediction-Key': prediction_key, 'Content-Type': content_type}, data=img_bytes)
		response = requests.post(URL, headers={'Content-Type': content_type}, data=img_bytes)
		print "calling time : {}".format(time() - t1)
		predictions = json.loads(response.text)['predictions']
		pprint(predictions)
		return predictions
	except:
		return []


def send_message(coor):
	msg = BoundingBox()
	msg.left = float(coor['left'])
	msg.top = float(coor['top'])
	msg.width = float(coor['width'])
	msg.height = float(coor['height'])
	#rospy.loginfo(msg)
	topic.publish(msg)


def detect_shutdown():
	print("Quit")
	pipeline.stop()


if __name__ == '__main__':
	main(1)

