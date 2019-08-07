#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from autorally_msgs.msg import wheelSpeeds

class AutorallyTwistTruth():
    def __init__(self):
        rospy.init_node("autorally_twist_truth_pub", anonymous=False)

        self.last_pose = None
        self.last_twist = None
        self.last_time = None
        self.last_spd_est = None

        self.last_speed = None

        self.alpha = 0.25
        self.twist_pub1 = rospy.Publisher('/AR_twist1', Twist, queue_size=10)
        self.twist_pub2 = rospy.Publisher('/AR_twist2', Twist, queue_size=10)
        self.smooth_spd_pub = rospy.Publisher('/smoothSpeed', Float64, queue_size=10)
        self.spd_pub = rospy.Publisher('/speed', Float64, queue_size=10)
        self.spd_err_pub = rospy.Publisher('/speed_err', Float64, queue_size=10)

        gazebo_model_state_sub = rospy.Subscriber('/autorally_platform/gazebo/model_states',
                ModelStates, self.model_states_cb)
        wheel_speed_sub = rospy.Subscriber('/wheelSpeeds', wheelSpeeds, self.wheel_speed_cb)
        bcn_speed_est_sub = rospy.Subscriber('/base_controller_nodelet/speedEst', Float64, self.bcn_speed_est_cb)


        rospy.spin()


    def model_states_cb(self, model_states_msg):
        ar_index = model_states_msg.name.index('autoRallyPlatform')
        ar_pose = model_states_msg.pose[ar_index]
        ar_twist = model_states_msg.twist[ar_index]

        current_time = rospy.get_time()
        if self.last_time is not None:
            delta_t = current_time - self.last_time
            if delta_t > 0 and self.last_pose is not None and self.last_twist is not None:

                linear_mvmt_est = Twist()
                linear_mvmt_est.linear.x = dist(self.last_pose.position, ar_pose.position) / delta_t
                linear_mvmt_est.linear.y = 0
                linear_mvmt_est.linear.z = 0
                linear_mvmt_est.angular.x = 0
                linear_mvmt_est.angular.y = 0
                linear_mvmt_est.angular.z = yaw_diff_from_xyquat(self.last_pose.orientation, ar_pose.orientation) / delta_t

                self.twist_pub1.publish(linear_mvmt_est)

                gazebo_twist_est = Twist()
                gazebo_twist_est.linear.x = (ar_twist.linear.x**2 + ar_twist.linear.y**2) ** 0.5
                gazebo_twist_est.linear.y = 0.0
                gazebo_twist_est.linear.z = 0.0
                gazebo_twist_est.angular.x = 0.0
                gazebo_twist_est.angular.y = 0.0
                gazebo_twist_est.angular.z = ar_twist.angular.z

                self.twist_pub2.publish(gazebo_twist_est)
                
                self.last_speed = gazebo_twist_est.linear.x

        self.last_pose = ar_pose
        self.last_twist = ar_twist
        self.last_time = current_time

    def wheel_speed_cb(self, wheel_spd_msg):
        current_spd = 0.5*(wheel_spd_msg.lfSpeed + wheel_spd_msg.rfSpeed)
        self.spd_pub.publish(current_spd)

        if self.last_spd_est is None:
            self.last_spd_est = current_spd
        else:
            self.last_spd_est = (1-self.alpha) * self.last_spd_est + self.alpha * current_spd

        self.smooth_spd_pub.publish(self.last_spd_est)

    def bcn_speed_est_cb(self, speed_msg):
        if self.last_speed is not None:
            err = Float64()
            err.data = speed_msg.data - self.last_speed
            self.spd_err_pub.publish(err)
        else:
            print('waiting for speed data...')



def dist(p1, p2):
    sqd_dist = 0.0
    
    sqd_dist += (p1.x - p2.x) ** 2
    sqd_dist += (p1.y - p2.y) ** 2
    sqd_dist += (p1.z - p2.z) ** 2
    
    return sqd_dist ** 0.5

def yaw_diff_from_xyquat(q1, q2):
    return yaw_from_xyquat(q1) - yaw_from_xyquat(q2)

def yaw_from_xyquat(q):
    sign = 1
    if q.z < 0:
        sign *= -1
    return 2*np.arccos(q.w)


if __name__ == '__main__':
    try:
        node = AutorallyTwistTruth()
    except rospy.ROSInterruptException:
        pass

