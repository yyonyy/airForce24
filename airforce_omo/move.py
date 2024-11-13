import rospy
from geometry_msgs.msg import Twist

speed = Twist()
rospy.init_node("move")
rate = rospy.Rate(100)
pub = rospy.Publisher("/cmd_vel", Twist,queue_size=1)
speed.linear.x = 5.0
speed.angular.z = 0.0

while not rospy.is_shutdown():
    pub.publish(speed)
    rate.sleep()    