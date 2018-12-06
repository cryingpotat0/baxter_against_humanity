#! /usr/bin/env python
import requests
import rospy
import actionlib
import base64
import cv2
import json
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
from perception_msgs.msg import ReadCardAction, ReadCardResult
import os

API_ENDPOINT = "https://vision.googleapis.com/v1/images:annotate"

key = "27f3b075acdf78ffe33942b9703d36d075e14b3b"
key = "ya29.c.El9oBvxdIZvnvPep9YMN80CZITQK-F9iCPP6RmILbANX44Nj98ZkNvMuKCaP-i94YgoqPi491ASRqijT--8QUpiiq_ijTyguoMZ3NOEN4qiBQB3NIgx-WJZRHQa3UOBerw"
key = "ya29.c.El9oBuSXn7By0E-ss97Oqt1fCOokbwPYKn5FvPWkP4otmfEmYFV1ayM0u8_6aqsxSM0HWJPWVwamtnw6NEl8-rnqCq-hFfJ7BB3aVkPXJdoXjVvxG7qpkl-a7Xh-bLhhzw"
key = "ya29.c.El9qBvjQJtTIEEI3x1rH6f-55vO3shPbgMbTrguReucgk3gPFM5Rj_0I_9a12yjRrU62R02WCy-MQy2WCpCpAY3QREQY-cCB6AdDayz2vrMz01ckQaQukEHbRZEwiL2irA"
key = "-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQCQmRWqW/BZ8axC\nbpyntkvSfPThSuhV721FXwm99xqVOc4YxLSpJzOWuh0BYD0ZCJTLOHEUEB+gF6zl\njqfGBUuaj2fm5kndsyVl55fCNwbvPM0Gw3lseXS7ArUXHfFPnoSjOqMUtDRJ/8uY\nUEPD9Bo3GIL2OMANXqYPNGbx2Cc94KWq3tjQnUPT3RsRI00XzluGvlqEnKKH49Ad\naB2ncLa5jot1ph5lXgmS0yAtbfABgxQ87yAxp0Cu37HdCL4Cz5lsRmTE9uavDeqD\noGC2E7zPwQJAasXTqGANJ6zFTjmpYm3bWRSl58I4QPP3MYW6tHEFqSKddzqvE6Hw\npB+PdY1pAgMBAAECggEACScvmGeWeNMUAOaIkxM2jH3tcF9yApKT26rO7o/4Rk7w\n7V9yQizhlp50XJRveaix5oVNnyQAJH2rK6LSmvXV2dRPEu4r6dU/vkohMKGUFkvG\nfLGd+5Bao8XhmbODBXO22LBLB3H6x15iUGNwTrSifi9WhuiDu7TslRTPmWqB3WhF\nJSTiJAwYMhs1Mv5AkJDqWvtzRuhNcFXsQsxyu9DWZx6Tdwf+AZFOHuxFuJ3zj+OH\nz8fo0HLcs9t+9IjIF7MfKgZFf/xD1ZKgsbB/O9gd8DiMLuebMfwfb7aTEMhFLmbK\nNQSfa3qbxpg3kEb6V6L9ZjWTrjD/KNuKvN+T2vbAcwKBgQDHCCZLfl7ms9ImE+/O\nlg6udU83wOi3NtixWhqqGsk7IF1eivRPZaq/IHlR4C9YXeEyMrNY7daqG3OsvPgt\n9e9Jn5syjEsHwSThprttdzniVaAfxFP7eGu2u0oNpketBp4gdeW/KegimpLGmm7F\nKub5lt2WIXLtlYjh+upwhQrI8wKBgQC5/Fn/BkPnEDhq7vrvKvyuJ4HRUTpE8MFH\nghAsGtu8pHSOeN/286XbgrXWBa6lGgP1BvMEfTF+L+9JZ4o+v/FSKpAin71wK+4j\n10xBgJx3YeCurtOs+i4KjXHz8vm3U+9uJRFQHAIjSl+khds+SW8MqfHYlgQn9b0B\nh+zrRCOnMwKBgHWUxfLhlDUc1LC0JH7aPZApM4SPjav3VerofUozCuDITQC2fICP\nYmEJUULie3Xr+EdMlpGZ6LCUCTFqbf/RA+1twiUJpqXNH/nDI4UW43Zn+XHq7WZp\njNDmMmh/7GY2u6IXpIPbwQz4Xm+/+mK25r9atkukNR5Qsg42yttwbZ+rAoGAF7aZ\nMu71kWx8BQZbcEsm1H2tC6czqr6XpiprLXRvN+owAkpfI/mE2CweSx+GWP6sKn2s\nVpIv0UsDU3SkJ8QJXNSYdBGx8a9oOaGl02Q2jvIn153q108+t5G5ScVdGYbWLFEF\nmBU6II3HDA8VGSf7xL2WXGskNIQciSlvh7+0XAkCgYBm5fTn26jA0ovAVv7c0jBN\nnskj0RYphBXgvihRHCX3sRhPW/jmxTR2zSk66qMqXAX+WxCcN21Xr4aCFmiKCvRv\nRkmtcHEJrfnQ0oHYwqfAkKYXACTZoq13yqjEOJJ4oYB+9Elwfi7JJNlJC1cH4fww\nJRqiHXuE0aq+sTfhX+e8kA==\n-----END PRIVATE KEY-----\n"
key = "AIzaSyA5ELuUAnM9Pmx1TdV7e8AbeBIi5Sh_yeo"
def send_vision_request_read_response(cv2_frame):
  # save the cv2 frame as an image file
  image_file_name = "/home/cc/ee106a/fa18/class/ee106a-abs/baxter_against_humanity/src/perception/src" + "saved_frame.png"
  try:
    os.remove(image_file_name)
  except Exception:
    pass

  cv2.imwrite(image_file_name, cv2_frame)
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
  response = requests.post(API_ENDPOINT + "?key=" + key , data=json.dumps(data), headers={
      #"Authorization": "Bearer {}".format(key),
      "Content-Type": "application/json"
      })

  # we return a list of all the descriptions within "textAnnotations" response 
  #import ipdb; ipdb.set_trace()
  #print(response.json()["responses"]["description"])
  try:
    text=  response.json()["responses"][0]["textAnnotations"][0]["description"]
    print(text, "WORKED")
    #time.sleep(50)
    return text
  except Exception as e:
    print(response.json())
    print(response, "FAILED")
    #time.sleep(50)
    return response.json()

class ReadCardServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('read_card', ReadCardAction, self.execute, False)
    print("Starting ReadCard Server")
    self.server.start()
    self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    self.cv_bridge = CvBridge()

  def move_arm(self, pos):
    # Do lots of awesome groundbreaking robot stuff here
    compute_ik = self.compute_ik
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_hand_camera"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Move gripper to an example place, like 0.695 -0.063 -0.222
    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = pos[0]# 0.267#pos[0]#0.8
    request.ik_request.pose_stamped.pose.position.y = pos[1]#-0.027#pos[1]#0.04#goal.card_pos[1]
    request.ik_request.pose_stamped.pose.position.z = pos[2]#0.675#pos[2]#0.1 #goal.card_pos[2]       
    request.ik_request.pose_stamped.pose.orientation.x = pos[3]#0.493
    request.ik_request.pose_stamped.pose.orientation.y = pos[4]#0.027
    request.ik_request.pose_stamped.pose.orientation.z = pos[5]#0.868
    request.ik_request.pose_stamped.pose.orientation.w = pos[6]#0.045

    try:
        #Send the request to the service
        #print("REQUEST:", request)
        response = compute_ik(request)
        
        #Print the response HERE
        #print("RESPONSE:", response)
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        #group.set_position_target(goal.card_pos)

        # Plan IK and execute
        group.go()
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("READCARD MOVE ARM")
    time.sleep(5)
    self.move_arm([0.752, 0.216, 0.166, -0.382, 0.595, 0.664, 0.242])
    self.move_arm([0.337, -0.010, 0.664, -0.406, 0.021, 0.914, 0.010])
    frame = rospy.wait_for_message("cameras/head_camera/image", Image)
    frame = self.cv_bridge.imgmsg_to_cv2(frame, "bgr8")
    text = send_vision_request_read_response(frame)
    result = ReadCardResult(str(text))
    print(text)
    self.move_arm([0.752, 0.216, 0.166, -0.382, 0.595, 0.664, 0.242])
    self.server.set_succeeded(result)

if __name__ == '__main__':
  # img = cv2.imread("./videos/test/img202.png")
  # print(img)
  # a = send_vision_request_read_response(img)
  # print(a)
  rospy.init_node('read_card_server')
  server = ReadCardServer()
  rospy.spin()
