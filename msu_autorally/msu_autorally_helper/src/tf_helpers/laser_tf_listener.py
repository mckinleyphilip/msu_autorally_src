#!/usr/bin/env python
import roslib

import rospy
import tf

from sensor_msgs import LaserScan

# callback for laser listener
def laser_callback(data):
    transformed_scan = tl.transformPoint("base_link", data)
    # publish the transformed laser scan?

if __name__ == '__main__':
    rospy.init_node('laser_tf_listener')

    tl = tf.TransformListener()
    tl.waitForTransform("base_link",
                        "autorally_platform/hokuyo",
                        rospy.time.now(),
                        rospy.Duration(5.0))
    topic = ropsy.get_param('~topic', 'sensor_msgs/LaserSacn')
    rospy.Subscriber(topic, LaserScan, laser_callback)
