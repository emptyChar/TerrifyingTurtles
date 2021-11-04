#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoint_publisher")

        self.start_time = rospy.get_time()

        # change these parameters to adjust setpoint(s)
        # either constant setpoint (setpoint_1), or jumping between two setpts
        self.constant = True  # constant or square wave?
        self.duration = 30.0  # in s
        self.setpoint_1 = -0.7
        self.setpoint_2 = -0.6

        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            Float64,
                                            queue_size=1)

    def get_setpoint(self):
        if self.constant:
            setpoint = self.setpoint_1
        else:  # square wave
            now = rospy.get_time()
            time = self.start_time - now

            i = time % (self.duration * 2)
            if i > (self.duration):
                setpoint = self.setpoint_1
            else:
                setpoint = self.setpoint_2

        self.publish_setpoint(setpoint)

    def publish_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        # check if the setpoint is within boundries
        if msg.data < -0.8:
            msg.data = 0.8
        elif msg.data > 0.1:
            msg.data = 0.1
        self.setpoint_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.get_setpoint()
            rate.sleep()


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
