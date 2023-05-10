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

import rospy
from sensor_msgs.msg import Image
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.step)
    log_d(f'recv img {data.header.seq}')

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