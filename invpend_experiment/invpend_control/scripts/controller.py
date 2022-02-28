#!/usr/bin/env python

import math

class controller():
    def __init__(self, delta_t, MAX, MIN, Kp, Ki, Kd, filter_gain):
        self.delta_t = delta_t
        self.output_max = MAX
        self.output_min = MIN
        self.kp = Kp
        self.kd = Kd
        self.ki = Ki
        self.filter_gain = filter_gain
        self.integral_state = 0
        self.derivative_state = 0
        self.pid_out =0

    def step(self, error):
        filter_coefficient = (error - self.derivative_state) * self.filter_gain

        integral_gain = (error + self.integral_state) + filter_coefficient

        output_signal = self.kp * error + self.kd * filter_coefficient + self.ki * integral_gain

        if output_signal > self.output_max:
            self.pid_out = self.output_max
        elif output_signal < self.output_min:
            self.pid_out = self.output_min
        else:
            self.pid_out = output_signal

        self.integral_state = self.integral_state + (self.delta_t * integral_gain)

        self.derivative_state = self.derivative_state + (self.delta_t * filter_coefficient)

        return self.pid_out
