import argparse
import os

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
import math


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



img_name = ''
class ImageReceiver:
    def __init__(self, save_dir='./imgs'):
        # self.subscriber = rospy.Subscriber("/vrep/image", Image, self.call_back)
        self.publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        # ROS to CV imgage convert
        self.bridge = CvBridge()
        self.save_dir = save_dir
        results_folder = Path(save_dir)
        results_folder.mkdir(parents=True, exist_ok=True)
        ref_img_path = './imgs_ref/'
        pictures_name = ["pic001", "pic002", "pic003", "pic004", "pic005"]
        marker_name = ["obama", "green hair girl", "super brother", "yellow hair boy", "cartoon boy"]
        self.pictures = [cv2.imread(ref_img_path + name + ".jpg") for name in pictures_name]
        # image detection init
        self.match_cnt = [0] * len(self.pictures)
        self.square_cnt = [0] * len(self.pictures)
        self.marked = [False] * len(self.pictures)
        self.bg_det_cnt = 0

        self.bf = cv2.BFMatcher()
        # Initialize the ORB detector
        # self.orb = cv2.ORB_create()
        self.surf = cv2.xfeatures2d.SURF_create(400)
        self.ref_kps, self.ref_descs = [], []

        for i, picture in enumerate(self.pictures):
            picture = cv2.resize(picture, (400, 400), interpolation=cv2.INTER_AREA)
            # Compute the keypoints and descriptors for both images
            kp1, des1 = self.surf.detectAndCompute(picture, None)
            # kp1, des1 = self.orb.detectAndCompute(picture, None)
            picture = cv2.flip(picture, 1)
            kp1_f, des1_f = self.surf.detectAndCompute(picture, None)
            # kp1_f, des1_f = self.orb.detectAndCompute(picture, None)
            self.ref_kps.append((kp1, kp1_f))
            self.ref_descs.append((des1, des1_f))


        self.markers = []
        for i in range(len(pictures_name)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "camera_link"
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.text = marker_name[i]
            self.markers.append(marker)

    def call_back(self, img):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.step)
        # log_d(f'recv img {img.header.seq}')

        # img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        # for i in range(5):
        #     self.publisher.publish(self.markers[i])

        match_id = -1
        global img_name
        id, cnt = self._best_fit(img)
        if id < 0: # means error
            return

        if cnt < 35: # detect the background
            self.match_cnt = [x - 2 if x > 0 else x for x in self.match_cnt]
            self.match_cnt = [0 if x < 0 else x for x in self.match_cnt]
            self.bg_det_cnt += 1
        else:
            # rospy.loginfo(str(id))
            log_d(f'match cnt is {self.match_cnt}, id {id}, cnt {cnt}')
            self.bg_det_cnt = 0
            for i in range(len(self.match_cnt)):
                if i == id:
                    if (self.match_cnt[i] < 10):
                        self.match_cnt[i] += 1
                else:
                    self.match_cnt[i] -= 2

            self.match_cnt = [0 if x < 0 else x for x in self.match_cnt]

            if(self.match_cnt[id] >= 10 and not self.marked[id]):
                match_id = id
                self._show_mark(id, img)

        if match_id > -1:
        # if 1:
            log_d(f'img {img_name}, current match id {id} cnt {cnt}, continue match id {match_id}, cnt {self.match_cnt}')


        # marker_pub.send_a_marker()
        # image_rcv.save_img(data)
        # laser_switch_pub.swith()

    def _show_mark(self, id, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img_BW = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        _, contours, _ = cv2.findContours(img_BW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        ratio = float(w) / h

        if ratio > 1.1 and ratio < 0.5:
            self.square_cnt[id] = 0
        else:
            self.square_cnt[id] += 1

        if (self.square_cnt[id] >= 15):
            self.markers[id].pose.position.x = 1 / math.tan(math.pi / 8 * h / img.shape[1]) * 0.5
            self.markers[id].pose.position.y = (x + w / 2) / img.shape[1]
            self.markers[id].pose.position.z = 0

            self.publisher.publish(self.markers[id])
            self.marked[id] = True

    def _best_fit(self, img):
        try:
            best_id = -1
            best_cnt = -1
            keypoint, descriptor = self.surf.detectAndCompute(img, None)
            # keypoint, descriptor = self.orb.detectAndCompute(img, None)

            for i, ref_des in enumerate(self.ref_descs):
                cur_cnt = 0

                for ref_des_i in ref_des:
                    result = self.bf.knnMatch(ref_des_i, descriptor, k=2)
                    for m, n in result:
                        if m.distance < 0.5 *n.distance:
                            cur_cnt += 1

                if cur_cnt > best_cnt:
                    best_cnt = cur_cnt
                    best_id = i
            return best_id, best_cnt
        except Exception as e:
            return -1, -1

    def save_img(self, img, format="bgr8",is_save=False):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(img, format)
        except Exception as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            if is_save:
                cv2.imwrite(f'{self.save_dir}/camera_image_{img.header.seq}.jpeg', cv2_img)
    def det_img(self, img, format="bgr8"):

        return detect_id

class LaserSwithPublisher():
    def __init__(self):
        # Create a publisher that publishes to the /vrep/laser_switch topic.
        self.pub = rospy.Publisher('/vrep/laser_switch', Bool, queue_size=10)

    def swith(self, turn_off=False):
        # Create a message to turn off the laser.
        msg = Bool()
        if (turn_off):
            msg.data = False
        else:
            msg.data = True
            # Publish the message.
        self.pub.publish(msg)


# send_a_marker(pub)
# Instantiate CvBridge
# bridge = CvBridge()
# marker_pub = MarkerPublisher()
image_rcv = ImageReceiver()
laser_switch_pub = LaserSwithPublisher()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('img_detection', anonymous=True)
    test_img_path = './record_imgs/imgs_figures'
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
