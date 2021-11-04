#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


class ThrustControl ():
    def __init__(self):
        rospy.init_node("thrust_controller")

        # Subscribes to: depth_setpoint, depth
        self.depth_setpoint_sub = rospy.Subscriber("depth_setpoint",
                                                   Float64,
                                                   self.on_depth_setpoint,
                                                   queue_size=1)

        self.depth_sub = rospy.Subscriber("depth",
                                          Float64,
                                          self.on_depth,
                                          queue_size=1)

        # Publishes: vertical_thrust
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

        # variables
        # self.depth_setpoint = -0.6
        self.depth = 0.0

        self.d = self.depth_sub
        self.t = rospy.get_time()
        self.v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.integral = 0

    def on_depth_setpoint(self, msg):
        self.depth_setpoint = msg.data

    def on_depth(self, msg):
        self.depth = msg.data

    def run(self):
        global v
        v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            thrust = self.control()
            self.vertical_thrust_pub.publish(thrust)
            rate.sleep()

    def control(self):
        # get the data
        d = Float64()
        d.data = self.depth
        dt = Float64()
        dt.data = 0.02

        self.depth_setpoint_sub = rospy.Subscriber("depth_setpoint",
                                                   Float64,
                                                   self.on_depth_setpoint,
                                                   queue_size=1)

        self.depth_sub = rospy.Subscriber("depth",
                                          Float64,
                                          self.on_depth,
                                          queue_size=1)
        # calculate output from data
        # for derivative
        dx = Float64()
        dx.data = 1000*(100000*d.data - 100000*self.depth) / dt.data
        v.insert(0, dx.data)
        v.pop()
        dy = Float64()
        dy.data = sum(v) / len(v)
        thrust_d = 1000000*dy.data

        # proportional
        depth_delta = self.depth_setpoint - self.depth
        thrust_p = depth_delta * 1

        #integral
        self.integral += depth_delta
        thrust_i = self.integral * 0.01



        thrust = Float64()

        thrust.data = thrust_p

        if thrust.data > 1:
            thrust.data = 1
        elif thrust.data < -1:
            thrust.data = -1

        if self.depth < -0.8:
            thrust.data = 0
        elif self.depth < -0.78:
            thrust.data = depth_delta*5

        if self.depth > -0.1:
            thrust.data = 0
        elif self.depth > -0.12:
            thrust.data = depth_delta*5

        # return output
        return thrust


def main():
    controller = ThrustControl()
    controller.run()


if __name__ == "__main__":
    main()
