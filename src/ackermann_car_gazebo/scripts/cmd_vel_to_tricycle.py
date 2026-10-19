#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist


class CmdVelToTricycle:
    def __init__(self):
        self.wheelbase = rospy.get_param("~wheelbase", 0.60)
        self.max_steering_angle = abs(rospy.get_param("~max_steering_angle", 0.60))
        input_topic = rospy.get_param("~input_topic", "cmd_vel")
        output_topic = rospy.get_param("~output_topic", "tricycle_cmd")

        self.publisher = rospy.Publisher(output_topic, Twist, queue_size=1)
        self.subscriber = rospy.Subscriber(input_topic, Twist, self.cmd_callback, queue_size=1)

    def cmd_callback(self, msg):
        speed = msg.linear.x
        yaw_rate = msg.angular.z

        out = Twist()
        out.linear.x = speed

        if abs(speed) < 1e-4:
            steering = 0.0 if abs(yaw_rate) < 1e-4 else math.copysign(self.max_steering_angle, yaw_rate)
        else:
            steering = math.atan(self.wheelbase * yaw_rate / speed)

        out.angular.z = max(-self.max_steering_angle, min(self.max_steering_angle, steering))
        self.publisher.publish(out)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_to_tricycle")
    CmdVelToTricycle()
    rospy.spin()
