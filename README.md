# Stanley Controller

<div align="center">
	<img src="resources/animation.gif" />
</div>

## Abstract

The Stanley controller is a non-linear controller for real-time autonomous automobile trajectory tracking. This repository contains a complete python abstraction of Stanford's [Stanley controller](http://robotics.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf).

#### Stanley Controller

```yaml
At initialisation
:param control_gain:                (float) time constant [1/s]
:param softening_gain:              (float) softening gain [m/s]
:param yaw_rate_gain:               (float) yaw rate gain [rad]
:param steering_damp_gain:          (float) steering damp gain
:param max_steer:                   (float) vehicle's steering limits [rad]
:param wheelbase:                   (float) vehicle's wheelbase [m]
:param path_x:                      (ArrayLike) list of x-coordinates along the path
:param path_y:                      (ArrayLike) list of y-coordinates along the path
:param path_yaw:                    (ArrayLike) list of discrete yaw values along the path
:param dt:                          (float) discrete time period [s]

At every time step
:param x:                           (float) vehicle's x-coordinate [m]
:param y:                           (float) vehicle's y-coordinate [m]
:param yaw:                         (float) vehicle's heading [rad]
:param target_velocity:             (float) vehicle's velocity [m/s]
:param steering_angle:              (float) vehicle's steering angle [rad]

:return limited_steering_angle:     (float) steering angle after imposing steering limits [rad]
:return target_index:               (int) closest path index
:return crosstrack_error:           (float) distance from closest path index [m]
```

#### Stanley Controller Piecewise

For the sake of performance, it is recommended to pass NumPy arrays to the piecewise function to remove any conversion overheads in every frame. See [here](https://github.com/winstxnhdw/python-bench/blob/master/src/array_conversion.ipynb) to understand the performance impact of array conversions.

```yaml
At initialisation
:param control_gain:                (float) time constant [1/s]
:param softening_gain:              (float) softening gain [m/s]
:param yaw_rate_gain:               (float) yaw rate gain [rad]
:param steering_damp_gain:          (float) steering damp gain
:param max_steer:                   (float) vehicle's steering limits [rad]
:param wheelbase:                   (float) vehicle's wheelbase [m]
:param dt:                          (float) discrete time period [s]

Every frame
:param x:                           (float) vehicle's x-coordinate [m]
:param y:                           (float) vehicle's y-coordinate [m]
:param yaw:                         (float) vehicle's heading [rad]
:param target_velocity:             (float) vehicle's velocity [m/s]
:param steering_angle:              (float) vehicle's steering angle [rad]
:param path_x:                      (ndarray) piecewise list of x-coordinates along the path
:param path_y:                      (ndarray) piecewise list of y-coordinates along the path
:param path_yaw:                    (ndarray) piecewise list of discrete yaw values along the path

:return limited_steering_angle:     (float) steering angle after imposing steering limits [rad]
:return target_index:               (int) closest path index
:return crosstrack_error:           (float) distance from closest path index [m]
```

## Useful Information

- This abstraction along with the original Stanley controller is based on forward driving. Thus, most computations are done relative to the vehicle's front axle.
- If you are passing path information piecewise, you have the advantage of increasing your control gain even further to minimise crosstrack error.

## Requirements

```bash
pip install numpy
```

## Demo

Recursively git clone the repository

```bash
git clone --recursive https://github.com/winstxnhdw/FullStanleyController.git
```

Install requirements.txt

```bash
pip install -r requirements.txt
```

Play the animation

```
python animation.py
```
