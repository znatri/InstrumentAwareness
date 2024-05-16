"""
Author: Raghavasimhan Sankaranarayanan
Date Created: 5/16/24
"""
import matplotlib.pyplot as plt
import numpy as np


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


if __name__ == "__main__":
    x = np.zeros(100)
    x[:10] = 0
    x[10:20] = 5
    x[20:30] = 10
    x[30:40] = 15
    x[40:50] = 10
    x[50:60] = 15
    x[60:70] = 10
    x[70:80] = 15
    x[80:90] = 5
    x[90:] = 0

    out = avg(x, 15)
    # out = lpf(out, 0.8)

    plt.plot(x)
    plt.plot(out)
    plt.show()
