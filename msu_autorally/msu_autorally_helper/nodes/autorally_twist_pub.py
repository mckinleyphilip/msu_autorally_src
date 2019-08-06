#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

class AutorallyTwistCmd():
    def __init__(self):
        rospy.init_node("autorally_twist_pub", anonymous=False)
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

        
        signal_live = t < 60
        if t >= 30:
            t -= 30
        if signal_live:
            if (0 <= t and t < 1):
                result.linear.x = t*max_linear
            elif (1 <= t and t < 9):
                result.linear.x = max_linear
            elif (9 <= t and t < 10):
                result.linear.x = max_linear*(1-0.5*(t-9))
            elif (10 <= t and t < 15):
                result.linear.x = max_linear*0.5
                if (10 <= t and t < 11):
                    result.angular.z = (t-10)*max_angular
                elif (11 <= t < 14):
                    result.angular.z = max_angular
                else:
                    result.angular.z = (1-(t-14))*max_angular
            elif (15 <= t and t < 16):
                result.linear.x = max_linear*(1-0.5*(16-t))
            elif (16 <= t and t < 24):
                result.linear.x = max_linear
            elif (24 <= t and t < 25):
                result.linear.x = max_linear*(1-0.5*(t-24))
            elif (25 <= t and t < 30):
                result.linear.x = max_linear*0.5
                if (25 <= t and t < 26):
                    result.angular.z = -(t-25)*max_angular
                elif (26 <= t and t < 29):
                    result.angular.z = -max_angular
                else:
                    result.angular.z = -(30-t)*max_angular
                    result.linear.x = (30-t)*0.5*max_linear
        
        return result, signal_live

if __name__ == '__main__':
    try:
        node = AutorallyTwistCmd()
    except rospy.ROSInterruptException:
        pass

