import random
import numpy as np
from scipy.interpolate import interp1d

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist, Quaternion
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetLinkState
import tf
import math



def interpolation():
    ix = []
    iy = []

    temp_x = np.array([0, random.randint(60,69), random.randint(100,109), random.randint(150,159), 200])
    temp_y = np.array([0, -20, 0, 20, 0])
    cubic_spline = interp1d(temp_x, temp_y, kind='cubic')
    waypoint_x_start = temp_x[0]
    waypoint_x_end = temp_x[-1]
    length = int(abs(waypoint_x_end - waypoint_x_start) / 0.01)
    ix = np.linspace(waypoint_x_start, waypoint_x_end, num=length)
    iy = cubic_spline(ix)
    ix_reversed = ix[::-1]
    iy_reversed = iy[::-1]
    return ix, iy, ix_reversed, iy_reversed

def pose_publisher():
    x_center=-1200
    y_center=-1200
    model_state_pub = rospy.Publisher('/gazebo/set_model_states', ModelStates, queue_size=1)
    rate = rospy.Rate(300)
    poses_msg = ModelStates()
    poses_msg.name = [None] * 9
    poses_msg.pose = [Pose() for i in range(9)]
    poses_msg.twist = [Twist() for i in range(9)]

    poses_msg.name[0] = 'target_true'
    poses_msg.name[1] = 'target_true_false1'
    poses_msg.name[2] = 'target_true_false2'
    poses_msg.name[3] = 'target_false1'
    poses_msg.name[4] = 'target_false2'
    poses_msg.name[5] = 'target_false3'
    poses_msg.name[6] = 'target_false4'
    poses_msg.name[7] = 'target_false5'
    poses_msg.name[8] = 'target_false6'


    poses_msg.pose[0].position.x = x_center - 200
    poses_msg.pose[0].position.y = y_center - 300
    poses_msg.pose[0].position.z = 3.5
    poses_msg.pose[1].position.x = x_center - 200
    poses_msg.pose[1].position.y = y_center + 300
    poses_msg.pose[1].position.z = 3.5
    poses_msg.pose[2].position.x = x_center + 200
    poses_msg.pose[2].position.y = y_center - 300
    poses_msg.pose[2].position.z = 3.5
    poses_msg.pose[3].position.x = x_center - 200
    poses_msg.pose[3].position.y = y_center
    poses_msg.pose[3].position.z = 3.5
    poses_msg.pose[4].position.x = x_center
    poses_msg.pose[4].position.y = y_center - 300
    poses_msg.pose[4].position.z = 3.5
    poses_msg.pose[5].position.x = x_center
    poses_msg.pose[5].position.y = y_center
    poses_msg.pose[5].position.z = 3.5
    poses_msg.pose[6].position.x = x_center
    poses_msg.pose[6].position.y = y_center + 300
    poses_msg.pose[6].position.z = 3.5
    poses_msg.pose[7].position.x = x_center + 200
    poses_msg.pose[7].position.y = y_center
    poses_msg.pose[7].position.z = 3.5
    poses_msg.pose[8].position.x = x_center + 200
    poses_msg.pose[8].position.y = y_center + 300
    poses_msg.pose[8].position.z = 3.5


    quat = tf.transformations.quaternion_from_euler(0, -1.57, 0)
    poses_msg.pose[0].orientation = Quaternion(*quat)
    poses_msg.pose[1].orientation = Quaternion(*quat)
    poses_msg.pose[2].orientation = Quaternion(*quat)
    poses_msg.pose[3].orientation = Quaternion(*quat)
    poses_msg.pose[4].orientation = Quaternion(*quat)
    poses_msg.pose[5].orientation = Quaternion(*quat)
    poses_msg.pose[6].orientation = Quaternion(*quat)
    poses_msg.pose[7].orientation = Quaternion(*quat)
    poses_msg.pose[8].orientation = Quaternion(*quat)

    try:
        while not rospy.is_shutdown():
            waypoint_x, waypoint_y, waypoint_x_reversed, waypoint_y_reversed = interpolation()
            initial_positions = [{'x': pose.position.x, 'y': pose.position.y} for pose in poses_msg.pose]
            for index in range(len(waypoint_x)):
                poses_msg.pose[0].position.x = initial_positions[0]['x'] + waypoint_y[index]
                poses_msg.pose[0].position.y = initial_positions[0]['y'] + waypoint_x[index]
                poses_msg.pose[1].position.x = initial_positions[1]['x'] + waypoint_y[index]
                poses_msg.pose[1].position.y = initial_positions[1]['y'] + waypoint_x[index]
                poses_msg.pose[2].position.x = initial_positions[2]['x'] + waypoint_y[index]
                poses_msg.pose[2].position.y = initial_positions[2]['y'] + waypoint_x[index]
                poses_msg.pose[3].position.x = initial_positions[3]['x'] + waypoint_y[index]
                poses_msg.pose[3].position.y = initial_positions[3]['y'] + waypoint_x[index]
                poses_msg.pose[4].position.x = initial_positions[4]['x'] + waypoint_y[index]
                poses_msg.pose[4].position.y = initial_positions[4]['y'] + waypoint_x[index]
                poses_msg.pose[5].position.x = initial_positions[5]['x'] + waypoint_y[index]
                poses_msg.pose[5].position.y = initial_positions[5]['y'] + waypoint_x[index]
                poses_msg.pose[6].position.x = initial_positions[6]['x'] + waypoint_y[index]
                poses_msg.pose[6].position.y = initial_positions[6]['y'] + waypoint_x[index]
                poses_msg.pose[7].position.x = initial_positions[7]['x'] + waypoint_y[index]
                poses_msg.pose[7].position.y = initial_positions[7]['y'] + waypoint_x[index]
                poses_msg.pose[8].position.x = initial_positions[8]['x'] + waypoint_y[index]
                poses_msg.pose[8].position.y = initial_positions[8]['y'] + waypoint_x[index]

                model_state_pub.publish(poses_msg)
                rate.sleep()
            for index in range(len(waypoint_x)):
                poses_msg.pose[0].position.x = initial_positions[0]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[0].position.y = initial_positions[0]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[1].position.x = initial_positions[1]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[1].position.y = initial_positions[1]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[2].position.x = initial_positions[2]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[2].position.y = initial_positions[2]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[3].position.x = initial_positions[3]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[3].position.y = initial_positions[3]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[4].position.x = initial_positions[4]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[4].position.y = initial_positions[4]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[5].position.x = initial_positions[5]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[5].position.y = initial_positions[5]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[6].position.x = initial_positions[6]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[6].position.y = initial_positions[6]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[7].position.x = initial_positions[7]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[7].position.y = initial_positions[7]['y'] + waypoint_x_reversed[index]
                poses_msg.pose[8].position.x = initial_positions[8]['x'] + waypoint_y_reversed[index]
                poses_msg.pose[8].position.y = initial_positions[8]['y'] + waypoint_x_reversed[index]

                model_state_pub.publish(poses_msg)
                rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    rospy.init_node('control_targets')
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    pose_publisher()
