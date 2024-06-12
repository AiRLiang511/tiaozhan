import rospy
import random
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose, TwistStamped

def Point():
    point_x = random.randint(500, 700)
    point_y = random.randint(500, 700)
    return point_x, point_y


def Thunderstorm_Pub():
    x, y= Point()
    thunderstorm = rospy.Publisher('/zhihang/thunderstorm', Pose, queue_size=1)
    f = 10.0
    rate = rospy.Rate(f)
    thunderstorm_pub = Pose()
    thunderstorm_pub.position.x, thunderstorm_pub.position.y, thunderstorm_pub.position.z = x, y, 0.0
    thunderstorm_pub.orientation.x, thunderstorm_pub.orientation.y, thunderstorm_pub.orientation.z, thunderstorm_pub.orientation.w = 0.0, 0.0, 0.0, 0.0
    while not rospy.is_shutdown():
        try:
            thunderstorm.publish(thunderstorm_pub)
        except:
            continue
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('zhihang3')
    Thunderstorm_Pub()
    rospy.spin()