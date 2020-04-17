import numpy as np
import matplotlib.pyplot as plt

def plotLine(alpha, radius, interval):
    dataLine = []
    if isinstance(alpha, list):
        for i, a in enumerate(alpha):
            r = radius[i]
            # _input = np.linspace(interval[0], interval[1])
            x = r*np.cos(a) - np.array(interval) * np.sin(a)
            y = r*np.sin(a) + np.array(interval) * np.cos(a)
            dataLine.append(f'a: {a}, r: {r}')
            plt.plot(x, y)

    else:
        _input = np.linspace(interval[0], interval[1])
        x = radius * np.cos(alpha) - np.array(interval) * np.sin(alpha)
        y = radius * np.sin(alpha) - np.array(interval) * np.cos(alpha)
        dataLine.append(f'a: {alpha}, r: {radius}')
        plt.plot(x, y)

    plt.xlim([-2, 2])
    plt.legend(dataLine)
    plt.show()


if __name__ == '__main__':
    # alpha = [1.57, 0.17, 0.79, 0.79, 0.79]
    # r = [1.50603, 0.45, 0.45, 0.32, 0.32]
    alpha = [-0.762224, 1.571262, -0.003589, 1.570190, -0.004958]
    r = [0.078177, 1.599398, 1.594018, 1.400392, 1.892890]
    # alpha = [0.01, 0.00, 0.00, 0.00, 0.00]
    # r = [0.54, 0.54, 0.54, 0.54, 0.54]
    # alpha = 0.79
    # r = 2.59
    plotLine(alpha, r, [-3, 3])