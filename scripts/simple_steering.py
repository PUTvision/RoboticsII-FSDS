#!/usr/bin/env python3

from typing import List

import numpy as np
import rospy as rp

from geometry_msgs.msg import TwistStamped, PoseArray, Pose
from fs_msgs.msg import ControlCommand, FinishedSignal, GoSignal


class SimpleSteering:
    """Simplex steering node for racecar which use only nformation about cones position."""

    def __init__(self, max_throttle: float, target_speed: float, max_steering: float, cones_range_cutoff: float):
        self.max_throttle = max_throttle
        self.target_speed = target_speed
        self.max_steering = max_steering
        self.cones_range_cutoff = cones_range_cutoff

        rp.Subscriber("/fsds/gss", TwistStamped, self.gss_signal_callback)

        rp.Subscriber('/fsds_roboticsII/cones_poses', PoseArray, self.cones_poses_callback)

        self.control_publisher = rp.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        self.finished_publisher = rp.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)
        self.go_publisher = rp.Publisher('/fsds/signal/go', GoSignal, queue_size=1)

        self.velocity = 0.0
        self.brake = 1 # float64 0..1
        self.go = False


    def start(self):
        """Start simulation, release the brake and send GO signal for race evaluator."""
        gs = GoSignal()
        gs.mission = 'Trackdrive'
        self.go_publisher.publish(gs)
        
        self.go = True
        self.brake = 0


    def gss_signal_callback(self, data: TwistStamped):
        """Racecar velocity calculation with use of speed sensor measurements.

        Parameters
        ----------
        data : TwistStamped
            Expresses velocity in free space broken into its linear and angular parts
        """
        v_x = data.twist.linear.x
        v_y = data.twist.linear.y

        self.velocity = np.sqrt(np.power(v_x, 2) + np.power(v_y, 2))


    def cones_poses_callback(self, cones_poses_data: PoseArray):
        """Cones poses data callback.

        Parameters
        ----------
        cones_poses_data : PoseArray
            Detected cones data.
        """
        cones_poses = cones_poses_data.poses

        steering = self.calculate_steering(cones_poses)

        throttle = self.calculate_throttle()

        self.publish_control_command(throttle=throttle, steering=steering)


    def calculate_steering(self, cones_poses: List[Pose]) -> float:
        """Calculate steering axis position based on detected cones position.

        Parameters
        ----------
        cones_poses : List[Pose]
            Array with detected cones poses.
        Returns
        -------
        float
            Steering axis position.
        """
        cones_average_y = sum([cone.position.y for cone in cones_poses]) / len(cones_poses)

        if cones_average_y > 0:
            return -max_steering
        else:
            return max_steering


    def calculate_throttle(self) -> float:
        """Calculate throttle opening.

        Returns
        -------
        float
            Throttle opening in range 0-1.
        """
        # The lower the velocity, the more throttle, up to self.max_throttle
        throttle = self.max_throttle * max(1 - self.velocity / self.target_speed, 0)

        if self.brake == 1:
            throttle = 0

        return throttle


    def publish_control_command(self, throttle: float, steering: float):
        """Publish racecar steering command for simulator.

        Parameters
        ----------
        throttle : float
            Racecar throttle opening in range 0..1.
        steering : float
            Racecar steering in range -1..1.
        """
        if self.go:
            cc = ControlCommand()

            cc.header.stamp = rp.Time.now()
            cc.throttle = throttle
            cc.steering = steering
            cc.brake = self.brake

            self.control_publisher.publish(cc)


    def emergency_brake(self):
        """Emergency brake for emergency state."""
        self.publish_finish_signal()


    def publish_finish_signal(self, *args):
        """Stop vehicle and publish finish signal (shutdown ros connection with simulator)."""
        self.brake = 1
        while self.velocity > 0:
            self.publish_control_command(throttle=0, steering=0)

        self.finished_publisher.publish(FinishedSignal())


if __name__ == '__main__':
    rp.init_node('simple_steering', log_level=rp.DEBUG)

    max_throttle = rp.get_param('/vehicle/max_throttle')
    target_speed = rp.get_param('/vehicle/target_speed')
    max_steering = rp.get_param('/vehicle/max_steering')
    cones_range_cutoff = rp.get_param('cones_range_cutoff')

    AS = SimpleSteering(
        max_throttle=max_throttle,
        target_speed=target_speed,
        max_steering=max_steering,
        cones_range_cutoff=cones_range_cutoff
    )
    
    # sleep for 5 seconds before all systems will be ready
    rp.sleep(5)
    AS.start()

    rp.spin()
