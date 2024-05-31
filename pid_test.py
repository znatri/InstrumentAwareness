"""
Author: Raghavasimhan Sankaranarayanan
Date Created: 5/16/24
"""
import time

import matplotlib.pyplot as plt
import numpy as np
from udpHandler import UDPHandler


def lpf(_x: np.ndarray, alpha):
    y = np.zeros_like(_x)
    y[0] = (1 - alpha) * x[0]
    for i in range(1, len(_x)):
        y[i] = (1 - alpha) * x[i] + alpha * y[i-1]
    return y


def avg(_x: np.ndarray, M):
    y = _x.copy()
    for i in range(M, len(_x)):
        y[i] = np.mean(_x[i - M: i])

    return y


class PID:
    def __init__(self, kp, kd, ki, time_period_ms=10):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.prev_e = 0
        self.integral = 0
        self.tp = time_period_ms * 0.001

    def step(self, measurement, set_point, offset=0):
        e = set_point - measurement
        P = self.kp * e
        self.integral += self.ki * e * self.tp
        D = self.kd * (e - self.prev_e) / self.tp
        self.prev_e = e
        return offset + P + self.integral + D


if __name__ == "__main__":
    x = np.zeros(300, dtype=float)
    x[:100] = 0
    x[100:200] = 5.5
    x[200:300] = 10.2

    udp = UDPHandler("10.2.1.177", 8888)
    for i in range(len(x)):
        udp.send(x[i])
        time.sleep(0.001 * 16.67)   # corresponds to 60 fps
    # pid = PID(kp=0.01, kd=1e-3, ki=50)
    # out = np.zeros_like(x)
    # for i in range(1, len(out)):
    #     out[i] = pid.step(out[i-1], x[i])
    # plt.plot(x)
    # plt.plot(out)
    # plt.show()
