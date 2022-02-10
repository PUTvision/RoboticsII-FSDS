#!/usr/bin/env python3
import math
import rospy
from fs_msgs.msg import ControlCommand
from fs_msgs.msg import FinishedSignal


class SineSteering:
    def __init__(self, steering_period: int = 10, publishing_frequency: float = 5.0, throtle: float = 0.2):
        self.t = 0.0                                        # Steering timestamp.

        self.steering_period = steering_period              # The number of seconds to do a full steering cycle (neutral -> full right -> full left -> neutroal).
        self.publishing_frequency = publishing_frequency    # Hz at what the car setpoints are published.
        self.throtle = throtle                              # The throtle. This is just a static value.

        self.controll_publisher = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
        self.finished_publisher = rospy.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)

    def publish(self, x):
        # TODO: Implement sine signal steering value calculation
        steering = 0

        cc = ControlCommand()
        cc.header.stamp = rospy.Time.now()
        cc.throttle = self.throtle
        cc.steering = steering
        cc.brake = 0
        self.controll_publisher.publish(cc)

        # TODO: Update steering timestamp 
        self.t += 0.0

    def finish(self, x):
        self.finished_publisher.publish(FinishedSignal())


if __name__ == '__main__':
    rospy.init_node('sine_steering')

    sine_steering = SineSteering()

    rospy.Timer(rospy.Duration(0.1), sine_steering.publish)
    rospy.Timer(rospy.Duration(30), sine_steering.finish)

    rospy.spin()
