#! /usr/bin/env python
import requests
import rospy
import actionlib
import base64

from perception_msgs.msg import ReadCardAction

API_ENDPOINT = "https://vision.googleapis.com/v1/images:annotate"

key = "27f3b075acdf78ffe33942b9703d36d075e14b3b"

def send_vision_request_read_response(cv2_frame):
	# save the cv2 frame as an image file
	image_file_name = "saved_frame.png"
	cv2.imwrite("saved_frame.png", cv2_frame)

	# encode image as base64 string
	with open("image_file_name", "rb") as image_file:
    	encoded_string = base64.b64encode(image_file.read())

    	# create POST request body
	data = {
	'image':{'content': encoded_string}
	'features':{'type': "TEXT_DETECTION"}
	}

	# make POST request
	response = requests.post(url = API_ENDPOINT + "?" + key, data = data)

	# we return a list of all the descriptions within "textAnnotations" response 
	return [a["description"] for a in response["textAnnotations"]]

class ReadCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('read_card', ReadCardAction, self.execute, False)
    print("Starting ReadCard Server")
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("executing ReadCard goal")

if __name__ == '__main__':
  rospy.init_node('read_card_server')
  server = ReadCardServer()
  rospy.spin()
