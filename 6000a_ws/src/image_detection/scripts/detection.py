import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--IDE", type=bool)
args = parser.parse_args()

if args.IDE:
    import sys
    path_list = ('/home/simba/Documents/project/roas_6000a/6000a_ws/devel/lib/image_detection', '/opt/ros/noetic/lib/python3/dist-packages', '/usr/lib/python38.zip', '/usr/lib/python3.8', '/usr/lib/python3.8/lib-dynload', '/home/simba/.local/lib/python3.8/site-packages', '/usr/local/lib/python3.8/dist-packages', '/usr/lib/python3/dist-packages')
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


from visualization_msgs.msg import Marker

def send_a_marker(publisher):
    marker = Marker()
    marker.header.frame_id = "base_link"  #!!!!!!!!!!!!!! should check frame name.
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0  # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 1
    marker.pose.position.y = 1
    marker.pose.position.z = 1
    publisher.publish(marker)

# Create a new publisher. We publish messages of type Marker to the topic /visualization_marker
pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

# send_a_marker(pub)
# Instantiate CvBridge
bridge = CvBridge()
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.step)
    log_d(f'recv img {data.header.seq}')
    send_a_marker(pub)
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('camera_image.jpeg', cv2_img)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('img_detection', anonymous=True)

    rospy.Subscriber("/vrep/image", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()