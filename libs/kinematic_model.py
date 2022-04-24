#!/usr/bin/env python

from numpy import cos, sin, tan
from libs import normalise_angle

class KinematicBicycleModel():

    def __init__(self, L=1.0, dt=0.05):

        """
        2D Kinematic Bicycle Model

        At initialisation
        :param L:           (float) vehicle's wheelbase [m]
        :param dt:          (float) discrete time period [s]

        At every time step
        :param x:           (float) vehicle's x-coordinate [m]
        :param y:           (float) vehicle's y-coordinate [m]
        :param yaw:         (float) vehicle's heading [rad]
        :param v:           (float) vehicle's velocity in the x-axis [m/s]
        :param delta:       (float) vehicle's steering angle [rad]

        :return x:          (float) vehicle's x-coordinate [m]
        :return y:          (float) vehicle's y-coordinate [m]
        :return yaw:        (float) vehicle's heading [rad]
        """

        self.dt = dt
        self.L = L

    def kinematic_model(self, x, y, yaw, v, delta):

        # Compute the state change rate
        x_dot = v * cos(yaw)
        y_dot = v * sin(yaw)
        omega = v * tan(delta) / self.L

        # Compute the final state using the discrete time model
        x += x_dot * self.dt
        y += y_dot * self.dt
        yaw += omega * self.dt
        yaw = normalise_angle(yaw)
        
        return x, y, yaw