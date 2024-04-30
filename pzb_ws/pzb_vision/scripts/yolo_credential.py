#!/usr/bin/env python3

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO # Yolov8
from ultralytics.utils.plotting import Annotator
import os
import yaml
import numpy as np
import PIL

class YoloCredentialDetection(Node):
  def __init__(self):
    super().__init__('yolo_credential')
    pkg_share_directory = get_package_share_directory('pzb_vision')

    # PARAMETERS
    self.declare_parameter('model_file','yolov8n.pt') # Model file to use
    
    # TOPICS - SUBSCRIBERS
    self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10) # Frames from a video stream
    
    # TOPICS - PUBLISHERS
    self.publisher_video= self.create_publisher(Image, '/yolo_credential_video', 10)
    self.pub_flag = self.create_publisher(Int32, '/yolo_credentials_flag', 10)
    self.flag_detection = Int32()

    # YOLO MODEL
    # self.MODEL_FILE = self.get_parameter('model_file').get_parameter_value().string_value
    # self.MODEL_PATH= os.path.join(pkg_share_directory,self.MODEL_FILE)
    # self.MODEL = YOLO(self.MODEL_PATH)
    self.MODEL = YOLO('/home/max/irs/src/TurboPlusPlus_ImplementacionRobotica/pzb_ws/pzb_vision/scripts/runs/detect/train2/weights/best.pt')  # pretrained YOLOv8n model
    self.get_logger().info('Model loaded')
    self.get_logger().info('Detection Classes: ' + str(self.MODEL.model.names[0]))
    
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    # Option 1
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

    # Option 2
    # From a picture
    # pil_image = PIL.Image.open('/home/max/irs/src/TurboPlusPlus_ImplementacionRobotica/pzb_ws/pzb_vision/scripts/imag.jpg').convert('RGB')
    # current_frame = np.array(pil_image)
    # current_frame = current_frame[:, :, ::-1].copy()

    # YOLO predictions
    results = self.MODEL(current_frame)  # generator of Results objects
    annotator = Annotator(current_frame, line_width=2)
    # results = self.MODEL.predict(current_frame, save=True, imgsz=640, conf=0.5)
    for result in results:
      boxes_xyxy = result.boxes.xyxy.cpu()
      color_box = (0,184,79)
      text_color = (0,0,0)
      conf_val = 0
      if(result.boxes.conf.size()[0] != 0):
        conf_val = result.boxes.conf[0].item()
      for box_xyxy in boxes_xyxy:
        if conf_val > 0.5:
          annotator.box_label(box_xyxy, label=str(conf_val),color=color_box,txt_color=(text_color))

      self.publisher_video.publish(self.br.cv2_to_imgmsg(current_frame, encoding='bgr8'))
  
def main(args=None):
  rclpy.init(args=args)
  yolo_credential = YoloCredentialDetection()
  rclpy.spin(yolo_credential)
  yolo_credential.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()