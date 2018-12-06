#! /usr/bin/env python
import requests
#import rospy
#import actionlib
import base64
import cv2
import json

#from perception_msgs.msg import ReadCardAction

API_ENDPOINT = "https://vision.googleapis.com/v1/images:annotate"

key = "27f3b075acdf78ffe33942b9703d36d075e14b3b"
key = "ya29.c.El9oBvxdIZvnvPep9YMN80CZITQK-F9iCPP6RmILbANX44Nj98ZkNvMuKCaP-i94YgoqPi491ASRqijT--8QUpiiq_ijTyguoMZ3NOEN4qiBQB3NIgx-WJZRHQa3UOBerw"
key = "ya29.c.El9oBuSXn7By0E-ss97Oqt1fCOokbwPYKn5FvPWkP4otmfEmYFV1ayM0u8_6aqsxSM0HWJPWVwamtnw6NEl8-rnqCq-hFfJ7BB3aVkPXJdoXjVvxG7qpkl-a7Xh-bLhhzw"

def send_vision_request_read_response(cv2_frame):
        # save the cv2 frame as an image file
        image_file_name = "saved_frame.png"
        cv2.imwrite("saved_frame.png", cv2_frame)

        # encode image as base64 string
        with open(image_file_name, "rb") as image_file:
            encoded_string = base64.b64encode(image_file.read())

        # create POST request body
        #data = {
        #'image':{'content': encoded_string},
        #'features':{'type': "TEXT_DETECTION"}
        #}
        data = {
                "requests":[
                    {
                        "image":{
                            "content": encoded_string
                            },
                        "features":[
                            {
                                "type":"TEXT_DETECTION",
                                }
                            ]
                        }
                    ]
                }

        # make POST request
        print(API_ENDPOINT + "?key=" + key)
        response = requests.post(API_ENDPOINT , json.dumps(data), headers={
            "Authorization": "Bearer {}".format(key),
            "Content-Type": "application/json"
            })

        # we return a list of all the descriptions within "textAnnotations" response 
        import ipdb; ipdb.set_trace()
        return [a["description"] for a in response["textAnnotations"]]

class ReadCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('read_card', ReadCardAction, self.execute, False)
    print("Starting ReadCard Server")
    self.server.start()

  def execute(self, goal):
    #goal.do_task = 1 => show card on screen
    # Do lots of awesome groundbreaking robot stuff here
    # use stddev of cropped part of image to determine if card is there
    print("executing ReadCard goal")

if __name__ == '__main__':
    img = cv2.imread("./videos/test/img202.png")
    print(send_vision_request_read_response(img))
    rospy.init_node('read_card_server')
    server = ReadCardServer()
    rospy.spin()
