import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("move_r1mini")
        rospy.Subscriber('/main_camera/image_raw/compressed', CompressedImage, self.camera_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.speed = Twist()
        self.speed.linear.x = 2.0
        self.speed.angular.z = 0.0
        self.image = None

    def camera_callback(self, data):
        try:    
            self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
            print("img!!!!!!!!!!")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
        
    def run(self):
        print("1111")
        while not rospy.is_shutdown():
            if self.image is not None:
                self.speed.angular.z = 0.0
                self.pub.publish(self.speed)
                print(self.speed)
            self.rate.sleep()

if __name__ == "__main__":
    move = Camera()
    move.run()
