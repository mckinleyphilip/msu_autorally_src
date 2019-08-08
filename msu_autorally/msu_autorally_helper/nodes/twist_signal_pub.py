#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

class TwistCmdPub():
    def __init__(self):
        rospy.init_node("twist_signal_pub", anonymous=False)
        twist_cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        pub_rate = rospy.Rate(10);

        running = True
        start_time = 0
        while start_time == 0:
            start_time = rospy.get_time()

        print('Twist signal pub, start_time=%.2f, publishing...' % start_time)
        while running:
            current_time = rospy.get_time() - start_time
            twist_cmd, running = self.getTwistSignal_1(current_time)
            
            twist_cmd_pub.publish(twist_cmd)

            pub_rate.sleep()
        print('Twist signal pub finished.')

    def getTwistSignal_1(self, t):
        max_linear = 10.0
        max_angular = 1.0
        
        result = Twist()
                
        result.linear.x = 0
        result.linear.y = 0
        result.linear.z = 0
        
        result.angular.x = 0
        result.angular.y = 0
        result.angular.z = 0

        
        signal_live = t < 70
        if t >= 35:
            t -= 35
        if signal_live:
            if (0 <= t and t < 2):
                result.linear.x = (0.5*t)*max_linear
            elif (1 <= t and t < 5):
                result.linear.x = max_linear
            elif (5 <= t and t < 10):
                result.linear.x = (1.5-0.1*t)*max_linear
            elif (10 <= t and t < 15):
                result.linear.x = max_linear*0.5
                if (10 <= t and t < 11):
                    result.angular.z = (t-10)*max_angular
                elif (11 <= t < 14):
                    result.angular.z = max_angular
                else:
                    result.angular.z = (15-t)*max_angular
            elif (15 <= t and t < 16):
                result.linear.x = (0.5+0.5*(t-15))*max_linear
            elif (16 <= t and t < 20):
                result.linear.x = max_linear
            elif (20 <= t and t < 25):
                result.linear.x = (3-0.1*t)*max_linear
            elif (25 <= t and t < 30):
                result.linear.x = 0.5*max_linear
                if (25 <= t and t < 26):
                    result.angular.z = -(t-25)*max_angular
                elif (26 <= t and t < 29):
                    result.angular.z = -max_angular
                else:
                    result.angular.z = -(30-t)*max_angular
            elif (30 <= t and t < 35):
                result.linear.x = (3.5-0.1*t)*max_linear
        
        return result, signal_live

if __name__ == '__main__':
    try:
        node = TwistCmdPub()
    except rospy.ROSInterruptException:
        pass

