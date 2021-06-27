#!/usr/bin/env python

import numpy as np
from libs.normalise_angle import normalise_angle

class KinematicBicycleModel():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0, L=1.0, dt=0.05):
        """
        2D Kinematic Bicycle Model

        :param x:           (float) vehicle's x-coordinate [m]
        :param y:           (float) vehicle's y-coordinate [m]
        :param yaw:         (float) vehicle's heading [rad]
        :param v:           (float) vehicle's velocity in the x-axis [m/s]
        :param delta:       (float) vehicle's steering angle [rad]
        :param L:           (float) vehicle's wheelbase [m]
        :param dt:          (float) discrete time period [s]

        :return x:          (float) vehicle's x-coordinate [m]
        :return y:          (float) vehicle's y-coordinate [m]
        :return yaw:        (float) vehicle's heading [rad]
        """

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = delta
        self.dt = dt
        self.L = L

    def kinematic_model(self):

        if self.delta == 0.0:
            omega = 0.0

        else:
            R = self.L / np.tan(self.delta)
            omega = self.v / R

        # Compute the state change rate
        x_dot = self.v * np.cos(self.yaw)
        y_dot = self.v * np.sin(self.yaw)

        # Compute the final state using the discrete time model
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.yaw += omega * self.dt
        self.yaw = normalise_angle(self.yaw)
        
        return self.x, self.y, self.yaw

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()
