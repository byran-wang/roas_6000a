import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--IDE", type=bool)
args = parser.parse_args()

if args.IDE:
    import sys

    path_list = ('/home/simba/Documents/project/roas_6000a/6000a_ws/devel/lib/image_detection',
                 '/opt/ros/noetic/lib/python3/dist-packages', '/usr/lib/python38.zip', '/usr/lib/python3.8',
                 '/usr/lib/python3.8/lib-dynload', '/home/simba/.local/lib/python3.8/site-packages',
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


class MarkerPublisher:
    def __init__(self):
        self.marker = Marker()
        self.marker.header.frame_id = "camera_link"  # !!!!!!!!!!!!!! should check frame name.
        self.marker.type = self.marker.TEXT_VIEW_FACING
        self.marker.text = 'me'
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.a = 1.0  # Don't forget to set the alpha!
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 1
        self.marker.pose.position.y = 1
        self.marker.pose.position.z = 1
        # Create a new publisher. We publish messages of type Marker to the topic /visualization_marker
        self.publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    def send_a_marker(self):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x += 0.1
        self.publisher.publish(self.marker)


def img_recv_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.step)
    log_d(f'recv img {data.header.seq}')
    marker_pub.send_a_marker()
    image_rcv.save_img(data)
    laser_switch_pub.swith()

class ImageReceiver:
    def __init__(self, is_save=False, save_dir='./imgs'):
        rospy.Subscriber("/vrep/image", Image, img_recv_callback)
        self.is_save = is_save
        self.save_dir = save_dir
        if is_save:
            results_folder = Path(save_dir)
            results_folder.mkdir(parents=True, exist_ok=True)

    def save_img(self, img, format="bgr8"):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(img, format)
        except Exception as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            if self.is_save:
                cv2.imwrite(f'{self.save_dir}/camera_image_{img.header.seq}.jpeg', cv2_img)

class LaserSwithPublisher():
    def __init__(self, turn_off=False):
        # Create a publisher that publishes to the /vrep/laser_switch topic.
        self.pub = rospy.Publisher('/vrep/laser_switch', Bool, queue_size=10)
        self.turn_off = turn_off

    def swith(self):
        # Create a message to turn off the laser.
        msg = Bool()
        if (self.turn_off):
            msg.data = False
        else:
            msg.data = True
            # Publish the message.
        self.pub.publish(msg)


# send_a_marker(pub)
# Instantiate CvBridge
bridge = CvBridge()
marker_pub = MarkerPublisher()
image_rcv = ImageReceiver(is_save=True)
laser_switch_pub = LaserSwithPublisher(turn_off=False)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('img_detection', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
