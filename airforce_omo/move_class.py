import rospy
from geometry_msgs.msg import Twist
class Move:
    def __init__(self):
        rospy.init_node("move_r1mini")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.speed = Twist()
        self.speed.linear.x = 3.0 # + for forward, - for backward
        self.speed.angular.z = 0.0 # + for right, - for left
        self.rate = rospy.Rate(100)

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.speed)
            self.rate.sleep()

if __name__ == "__main__":
    move = Move()
    move.run()