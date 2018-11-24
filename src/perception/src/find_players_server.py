#! /usr/bin/env python

import rospy
import actionlib
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
from perception_msgs.msg import FindPlayersAction, FindPlayersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FindPlayersServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_players', FindPlayersAction, self.execute, False)
    print("Starting FindPlayers Server")
    self.server.start()
    self.cv_bridge = CvBridge()
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    self.hog = hog

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("goal received")
    #cv2.destroyAllWindows()
    if goal.do_task == 0: 
      self.server.set_succeeded()
      return

    #self.server.publish_feedback(pan_angle)
    img = rospy.wait_for_message("cameras/head_camera/image", Image)
    img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
    #cv2.imshow("converted", img)
    #cv2.waitKey(0)
    num_ppl = self.detect_people(img)
    result = FindPlayersResult(num_ppl)
    print("numppl: ", result)
    self.server.set_succeeded(result)

  def detect_people(self, image):
    #image = cv2.imread(image)
    image = imutils.resize(image, width=min(400, image.shape[1]))
    orig = image.copy()
   
    # detect people in the image
    #(rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4),
    #  padding=(8, 8), scale=1.05)
    (rects, weights) = self.hog.detectMultiScale(image, winStride=(8, 8))
      #padding=(8, 8), scale=1.05)
   
    # draw the original bounding boxes
    for (x, y, w, h) in rects:
      cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
   
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
   
    # draw the final bounding boxes
    for (xA, yA, xB, yB) in pick:
      cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
   
    # show some information on the number of bounding boxes
    #print("[INFO] {}: {} original boxes, {} after suppression".format(
    #  "image", len(rects), len(pick)))
   
    # show the output images
    #cv2.imshow("Before NMS", orig)
    cv2.imshow("After NMS", image)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()
    return len(pick)


if __name__ == '__main__':
  rospy.init_node('find_players_server')
  server = FindPlayersServer()
  rospy.spin()