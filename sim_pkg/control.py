# -*- coding: utf-8 -*-

"""This module exposes functions for robot positional control."""

from __future__ import absolute_import

from typing import Tuple, Optional
from time import time

import numpy as np

# TODO: Incorrect import
try:
    from .math_utils import angle_between, clamp_angle, distance
    from . import motor
except (ImportError, ValueError):
    from math_utils import angle_between, clamp_angle, distance
    import motor


def _sat(dis, s, k1, k2):
    """
    Theoretically, this function is supposed to return motor powers based off a
    PID controller model (P-only)?

    I'm not sure.

    Todo:
        Understand what the purpose of this function is.
    """
    # pylint: disable-all
    MIN_P = 30
    l = 0
    r = 0
    vel = 30 / dis / k1
    if abs(s * k2 * dis) > 10:
        if s > 0:
            l = dis * k1 * vel - 10
            r = dis * k1 * vel + 10
        if s < 0:
            l = dis * k1 * vel + 10
            r = dis * k1 * vel - 10
    else:
        l = dis * k1 * vel - s * k2 * dis
        r = dis * k1 * vel + s * k2 * dis
    dif = r - l
    if dif > 0:
        if r <= MIN_P or l <= MIN_P:
            l = MIN_P
            r = MIN_P + dif
            return [l, r]
        if l >= 100 or r >= 100:
            r = 100
            l = r - dif
            return [l, r]
    else:
        if l <= MIN_P or r <= MIN_P:
            r = MIN_P
            l = r - dif
            return [l, r]
        if l >= 100 or r >= 100:
            l = 100
            r = 100 + dif
            return [l, r]
    return [l, r]

def drive_robot(pos, target, motor_msg, status):
    """
    This function drives a robot towards some position.

    Parameters:
        pos (tuple[float, float, float]): The current position (x, y, theta)
        target (tuple[float, float]): The target position (x, y)
        motor_msg (multiprocessing.Array): The motor message mp.Array.
        status: (list): A list of status flags. It is not currently understood
            what the purpose of this flag is since it is 0 always where it is
            used. Note that this function sets the first element of status to
            something.

    Returns:
        A status code? Unknown, as of now.

    Todo:
        Refactor and understand.
    """
    # pylint: disable-all
    target_theta = angle_between(pos, target)
    theta = pos[2]

    # The maximum distance error from the target.
    # These values control the maximum error from the target.
    MAX_DIST_ERR = 0.1
    MAX_THETA_ERR = 0.4
    MAX_THETA_ERR_2 = 0.7

    p = 20
    theta = clamp_angle(theta, '+-pi')
    dist_to_target = distance(pos, target)

    if dist_to_target > MAX_DIST_ERR:
        theta_err = clamp_angle(target_theta - theta, '+-pi')
        ss = clamp_angle(target_theta - theta - np.pi, '+-pi')

        if abs(theta_err) >= abs(ss):
            if status[0] == 0:
                if abs(ss) > MAX_THETA_ERR:
                    motor.set_pow(motor_msg,
                                  -50 * np.sign(ss), 50 * np.sign(ss))
                    status[0] = 0
                    return 0

                temp = _sat(dist_to_target, -ss, p, 30)
                motor.set_pow(motor_msg, -temp[0], -temp[1])

                status[0] = 1
                return 1

            if status[0] == 1:
                if abs(ss) > MAX_THETA_ERR_2:
                    motor.set_pow(motor_msg,
                                  -50 * np.sign(ss), 50 * np.sign(ss))
                    status[0] = 0
                    return 10

                temp = _sat(dist_to_target, -ss, p, 30)
                motor.set_pow(motor_msg, -temp[0], -temp[1])
                status[0] = 1
                return 11
        else:
            if status[0] == 0:
                if abs(theta_err) > MAX_THETA_ERR:
                    motor.set_pow(motor_msg,
                                  -50 * np.sign(theta_err),
                                  50 * np.sign(theta_err))
                    status[0] = 0
                    return 0

                temp = _sat(dist_to_target, theta_err, p, 30)
                motor.set_pow(motor_msg, temp[0], temp[1])

                status[0] = 1
                return 1

            if status[0] == 1:
                if abs(theta_err) > MAX_THETA_ERR_2:
                    motor.set_pow(motor_msg,
                                  -50 * np.sign(theta_err),
                                  50 * np.sign(theta_err))
                    status[0] = 0
                    return 10

                temp = _sat(dist_to_target, theta_err, p, 30)
                motor.set_pow(motor_msg, temp[0], temp[1])
                status[0] = 1
                return 11
    else:
        motor.set_pow(motor_msg, 0, 0)
        return 2


class PIDController:
    """This class represents a simple, 1-dimensional PID controller.

    When using the PID controller, you must reset it before use.

    Example:

    .. code-block:: python

       ctl = PIDController(coeffs, current, target)
       ctl.reset()
       while abs(ctl.last_error) > max_error:
           # Measure input
           drive = ctl.step(current)
           do_drive(drive)

    Parameters:
        coeffs (Tuple[float, float, float]): The coeffs Kp, Ki, Kd
        initial_value (float): The starting y(t)
        set_point (float): The target set point.
        circle_max (Optional[float]): If not None, then the PID controller will
            use circular error rather than Euclidean error. Set the value to
            the maximum value in the circle. The circle values must range from
            ``[-circle_max, circle_max]``. Useful for creating an angle PID
            controller as it resolves the pi-crossing errors.
    """
    def __init__(self, coeffs, initial_value, set_point, circle_max=None):
        # type: (Tuple[float, float, float], float, float, Optional[float]) -> None
        self._Kp = coeffs[0]  # pylint: disable=invalid-name
        self._Ki = coeffs[1]  # pylint: disable=invalid-name
        self._Kd = coeffs[2]  # pylint: disable=invalid-name
        self._max_integral = None
        self._max_val = None
        self._error_calculator = PIDController.error_euclidean \
            if not circle_max is not None \
            else lambda c, t: PIDController.error_circular(c, t, circle_max)

        self.reset(initial_value, set_point)

    @property
    def max_integral_val(self):
        # type: () -> Optional[float]
        """The maximum integral component value. You can use this to limit
        windup. Set to None to remove anti-windup."""
        return self._max_integral

    @max_integral_val.setter
    def max_integral_val(self, value):
        # type: (Optional[float]) -> None
        self._max_integral = value

    @property
    def max_value(self):
        # type: () -> Optional[float]
        """The maximum value. Use this to limit the output of the PID
        controller."""
        return self._max_val

    @max_value.setter
    def max_value(self, value):
        # type: (Optional[float]) -> None
        self._max_val = value

    def reset(self, initial_value, set_point):
        # type: (float, float) -> None
        """Resets the PID controller to an initial state."""
        self.integral = 0.0
        self.derivative = 0.0

        self.set_point = set_point
        self.last_error = self.calculate_error(initial_value)

        self.last_time = time()

    def calculate_error(self, point):
        # type: (float) -> float
        """Calculates the error in the controller."""
        return self._error_calculator(point, self.set_point)

    def _time_since_last_step(self):
        # type: () -> float
        sample_time = time()
        sample_period = sample_time - self.last_time
        self.last_time = sample_time
        return sample_period

    def step(self, data_point, sample_period=None, set_point=None):
        # type: (float, Optional[float], float) -> float
        """Given a data_point, the controller calculates the drive signal.

        If you do not provide a sample period, the controller will
        automatically measure the time since the last step.

        Parameters:
            data_point (float): The current plant output.
            sample_period (Optional[float]): The period between sampling.
            set_point (Optional[float]): An optional set_point to track.

        Returns:
            (float): The control signal u(t)
        """
        if set_point is not None:
            self.set_point = set_point

        period = sample_period if sample_period is not None \
            else self._time_since_last_step()

        error = self.calculate_error(data_point)

        new_integral = self.integral \
            + period * (error + self.last_error) / 2.0
        self.integral = min(self.max_integral_val, new_integral) \
            if self.max_integral_val is not None else new_integral

        self.derivative = (error - self.last_error) / period

        self.last_error = error

        u_t = (self._Kp * error +
               self._Ki * self.integral +
               self._Kd * self.derivative)

        return min(u_t, self.max_value) if self.max_value is not None else u_t

    @staticmethod
    def error_euclidean(current, target):
        # type: (float, float) -> float
        """Calculates the error using the trivial formula:
            err = target - current.

        Parameters:
            current (float): The current value
            target (float): The target value

        Returns:
            The error.
        """
        return target - current

    @staticmethod
    def error_circular(current, target, max_val):
        # type: (float, float, float) -> float
        """Calculates the circular error. Note that on a circle you will likely
        have a zero-crossing somewhere, so Euclidean error will not work.

        Parameters:
            current (float): The current value
            target (float): The target value
            max_val (float): The maximum value in the circle. Note that the
                minimum value in the circle must be equal to -max_val. For
                regular geometric circles, set this to pi.

        Returns:
            The error.
        """
        return min((target - current + 2 * max_val,
                    target - current,
                    target - current - 2 * max_val),
                   key=abs)


class MotorController:
    @staticmethod
    def power_from_relative_angle_speed(angle, speed):
        # type: (float, float) -> Tuple[float, float]
        """Given an angle and a speed, this function converts this into motor
        powers.

        Parameters:
            angle (float): The angle you wish to rotate to.
            speed (float): The speed you wish to move by.

        Todo:
            Document this function a bit better. Use actual units rather than
            these relative values.
        """
        motor_left = (1.0 if angle < 0 else 1.0 - 2.0 / np.pi * angle) * speed
        motor_right = (1.0 if angle >= 0 else 2.0 / np.pi * angle + 1) * speed
        return (motor_left, motor_right)