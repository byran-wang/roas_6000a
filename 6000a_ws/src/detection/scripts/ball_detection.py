import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--IDE", type=bool)
parser.add_argument("--DEBUG_LOCAL_IMAGE", type=bool)
args = parser.parse_args()

if args.IDE:
    import sys

    path_list = ('~/Documents/project/roas_6000a/6000a_ws/devel/lib/detection',
                 '/opt/ros/noetic/lib/python3/dist-packages', '/usr/lib/python38.zip', '/usr/lib/python3.8',
                 '/usr/lib/python3.8/lib-dynload', '~/.local/lib/python3.8/site-packages',
                 '/usr/local/lib/python3.8/dist-packages', '/usr/lib/python3/dist-packages')
    for p in path_list:
        sys.path.append(p)


def log_d(str):
    if args.IDE:
        print(str)
    else:
        rospy.loginfo(str)

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from pathlib import Path

from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import math
import numpy as np
# import matplotlib.pyplot as plt

img_name = ''
class ImageReceiver:
    def __init__(self, save_dir='./imgs'):
        if not args.DEBUG_LOCAL_IMAGE:
            self.subscriber = rospy.Subscriber("/vrep/image", Image, self.call_back)
        # ROS to CV imgage convert
        self.bridge = CvBridge()
        self.save_dir = save_dir
        results_folder = Path(save_dir)
        results_folder.mkdir(parents=True, exist_ok=True)

    def call_back(self, img):
        global img_name
        try:
            if not args.DEBUG_LOCAL_IMAGE:
                log_d(f'recv img {img.header.seq}')
                img_seq =  img.header.seq
                img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            else:
                img_seq = img_name.split('/')[-1].split('.jpeg')[0].split('_')[-1]
            img = cv2.flip(img, 1)
            # convert to hsv space
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            #mask ball image
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            # find ball contour
            (_, contours, _) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) < 1:
                return

            # locate centroid of ball
            # largest_perimeter = 0.0
            largest_area = 0.0
            for c in contours:
                area = cv2.contourArea(c)
                # log_d(f'img {img_seq}, perimeter {cv2.arcLength(c, True)}, Area {area}')
                if (cv2.contourArea(c) > largest_area):
                    best_c = c
                    largest_area = area

            M = cv2.moments(best_c)
            cX = float(M["m10"] / M["m00"])
            cY = float(M["m01"] / M["m00"])
            radius = int(math.sqrt(largest_area / math.pi))
            log_d(f'detected ball in image {img_seq} with radius {radius}, centroid {cX:.2f},{cY:.2f}')
            # self._save_debug_contour_img(img, best_c, cX, cY, radius, img_seq)
        except Exception as e:
            print(e)
    def _save_debug_contour_img(self, img, contours, cX, cY, radius, img_seq):
        # Draw all contours on a copy of the original image
        contour_image = np.copy(img)
        # To draw all the contours in an image
        cv2.drawContours(contour_image, contours, -1, (0, 0, 255), 3)
        cv2.circle(contour_image, (int(cX), int(cY)), 3, (255, 0, 0), -1)
        cv2.circle(contour_image, (int(cX), int(cY)), int(radius), (255, 0, 0), 2)
        cv2.putText(contour_image, "centroid", (int(cX) - 25, int(cY) - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        # Convert color space for displaying the result with matplotlib
        # contour_image = cv2.cvtColor(contour_image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f'{self.save_dir}/contours_{img_seq}.jpg', contour_image)

rospy.init_node('img_detection', anonymous=True)
image_rcv = ImageReceiver()


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    if args.DEBUG_LOCAL_IMAGE:
        test_img_path = './record_imgs/imgs_ball'
        all_files = os.listdir(test_img_path)
        all_files.sort(key=lambda x: os.path.getmtime(os.path.join(test_img_path, x)))
        global img_name

        for img_file in all_files:
            img_name = os.path.join(test_img_path,img_file)
            img = cv2.imread(img_name)
            image_rcv.call_back(img)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()