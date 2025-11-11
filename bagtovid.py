#!/usr/bin/env python3
import rosbag
import cv2
from cv_bridge import CvBridge

bag = rosbag.Bag('test1.bag', 'r')
bridge = CvBridge()
out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))

for topic, msg, t in bag.read_messages(topics=['/camera/rgb/image_raw']):
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    out.write(cv_img)

bag.close()
out.release()
