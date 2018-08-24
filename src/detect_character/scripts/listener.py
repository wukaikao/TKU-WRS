#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from keras.models import load_model
from keras import backend as K

from sklearn.preprocessing import LabelEncoder

from subprocess import call
import threading
##=================Start of the class image_get=================
class image_get:

  def __init__(self):
    self.capturing = False
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
  
  def callback(self,data):
    try:
      self.capturing = True
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      self.capturing = False
      print(e)
    (rows,cols,channels) = self.cv_image.shape
    # cv2.imshow("Image window", self.cv_image)
    # cv2.waitKey(3)
  def grab_image(self):
    return self.cv_image

  def state_check(self):
    recieve = self.capturing
    self.capturing=False
    return recieve
    

##=================End of the class image_get=================
class mnist_detected:
  def __init__(self):
    self.SIZE = 28
    self.img_rows, self.img_cols = 28, 28
    
    if K.image_data_format() == 'channels_first':
        self.input_shape = (1, self.img_rows, self.img_cols)
        self.first_dim = 0
        self.second_dim = 1
    else:
        self.input_shape = (self.img_rows, self.img_cols, 1)
        self.first_dim = 0
        self.second_dim = 3

    print("loading model")
    self.model = load_model("/home/iclab/Desktop/WRS/ObjectPart/WRS_ws/src/detect_character/scripts/full_model.mnist")
    self.labelz = dict(enumerate(["zero", "one", "two", "three", "four",
                           "five", "six", "seven", "eight", "nine"]))
  
  def annotate(self,frame, label, location = (20,30)):
      #writes label on image#
      font = cv2.FONT_HERSHEY_SIMPLEX
      cv2.putText(frame, label, location, font,
                  fontScale = 0.5,
                  color = (255, 255, 0),
                  thickness =  1,
                  lineType =  cv2.LINE_AA)

  def extract_digit(self,frame, rect, pad = 10):
      x, y, w, h = rect
      cropped_digit = frame[y-pad:y+h+pad, x-pad:x+w+pad]
      cropped_digit = cropped_digit/255.0

      #only look at images that are somewhat big:
      if cropped_digit.shape[0] >= 32 and cropped_digit.shape[1] >= 32:
          cropped_digit = cv2.resize(cropped_digit, (self.SIZE, self.SIZE))
      else:
          return
      return cropped_digit


  def img_to_mnist(self,frame, tresh = 90):
      gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
      #adaptive here does better with variable lighting:
      gray_img = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, blockSize = 321, C = 28)

      return gray_img
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  def detecte_run(self,frame):
    final_img = self.img_to_mnist(frame)
    image_shown = frame
    _, contours, _ = cv2.findContours(final_img.copy(), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)

    rects = [cv2.boundingRect(contour) for contour in contours]
    rects = [rect for rect in rects if rect[2] >= 3 and rect[3] >= 8]

    #draw rectangles and predict:
    for rect in rects:

      x, y, w, h = rect
      mnist_frame = self.extract_digit(final_img, rect, pad = 10)

      if mnist_frame is not None: #and i % 25 == 0:
        mnist_frame = np.expand_dims(mnist_frame, self.first_dim) #needed for keras
        mnist_frame = np.expand_dims(mnist_frame, self.second_dim) #needed for keras

        class_prediction = self.model.predict_classes(mnist_frame, verbose = False)[0]
        prediction = np.around(np.max(self.model.predict(mnist_frame, verbose = False)), 2)
        label = str(prediction) # if you want probabilities

        cv2.rectangle(image_shown, (x - 15, y - 15), (x + 15 + w, y + 15 + h),
                      color = (255, 255, 0))
        label = self.labelz[class_prediction] + " " + label
        self.annotate(image_shown, label, location = (rect[0], rect[1]))
    # return image_shown
    cv2.imshow('frame', image_shown)
    cv2.waitKey(1)


def detecte_thread_function(args):
  global thread_flag
  realsense = image_get()
  Number = mnist_detected()
  while (thread_flag):
    try:
      if realsense.state_check():
        image = Number.detecte_run(realsense.grab_image())
    except KeyboardInterrupt:
      break
  print("Turn off")

def listener(args):
  global thread_flag
  thread_flag = True
  rospy.init_node('image_get', anonymous=True)

  detecte_number_thd = threading.Thread(target=detecte_thread_function,name='detectd_thread',args=args)
  try:
    detecte_number_thd.start()
    rospy.spin()
  except KeyboardInterrupt:
    print("Close")
  print("Shutting down\n\n\n\n\n")
  thread_flag = False
  detecte_number_thd.join()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    listener(sys.argv)

