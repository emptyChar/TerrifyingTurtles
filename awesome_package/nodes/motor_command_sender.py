#!/usr/bin/env python
import rospy
import math
from mavros_msgs.msg import MotorSetpoint


class MyFirstNode():
    def __init__(self):
        rospy.init_node("motor_command_sender")
        self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint",
                                            MotorSetpoint,
                                            queue_size=1)

    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            msg = MotorSetpoint()
            msg.header.stamp = rospy.Time.now()
            # since the bluerov has 8 motors, the setpoint list holds 8 values
            t = rospy.get_time()
            msg.setpoint[0] = 0.2 * math.sin(t)
            msg.setpoint[1] = -0.2 * math.sin(t)
            msg.setpoint[2] = 0.2 * math.cos(t)
            msg.setpoint[3] = -0.2 * math.cos(t)
            msg.setpoint[4] = 0.4 * math.sin(t)
            msg.setpoint[5] = -0.4 * math.sin(t)
            msg.setpoint[6] = 0.4 * math.cos(t)
            msg.setpoint[7] = -0.4 * math.cos(t)

            self.setpoint_pub.publish(msg)

            rate.sleep()


def main():
    node = MyFirstNode()
    node.run()


if __name__ == "__main__":
    main()
