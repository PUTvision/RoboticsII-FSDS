#!/usr/bin/env python3

import math

import rospy as rp

from fs_msgs.msg import ControlCommand
from fs_msgs.msg import FinishedSignal


class SineSteering:
    def __init__(self, steering_period: int = 10, publishing_frequency: float = 5.0, throtle: float = 0.2):
        # Steering timestamp.
        self.t = 0.0

        # The number of seconds to do a full steering cycle (neutral -> full right -> full left -> neutral).
        self.steering_period = steering_period

        # Hz at what the car setpoints are published.
        self.publishing_frequency = publishing_frequency
        
        # The throtle. This is just a static value.
        self.throtle = throtle

        self.controll_publisher = rp.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
        self.finished_publisher = rp.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)


    def publish(self, *args):
        # TODO: Implement sine signal steering value calculation
        steering = 0

        cc = ControlCommand()
        cc.header.stamp = rp.Time.now()
        cc.throttle = self.throtle
        cc.steering = steering
        cc.brake = 0
        self.controll_publisher.publish(cc)

        # TODO: Update steering timestamp 
        self.t += 0.0


    def finish(self, *args):
        self.finished_publisher.publish(FinishedSignal())


if __name__ == '__main__':
    rp.init_node('sine_steering')

    sine_steering = SineSteering()

    rp.Timer(rp.Duration(0.1), sine_steering.publish)
    rp.Timer(rp.Duration(30), sine_steering.finish)

    rp.spin()
