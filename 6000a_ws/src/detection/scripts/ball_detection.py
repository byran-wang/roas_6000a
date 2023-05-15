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
        if not args.DEBUG_LOCAL_IMAGE:
            log_d(f'recv img {img.header.seq}')
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")

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