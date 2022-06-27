import matplotlib.pyplot as plt

path = [(0.1707, 0.2702), (0.141, 0.139), (0.212, 0.208), (0.283, 0.275)]

X = []
Y = []

for item in path:
    X.append(item[0])
    Y.append(item[1])

plt.ion() # turn interactive mode on
animated_plot = plt.plot(X, Y, 'ro')[0]

for i in range(len(X)):
    animated_plot.set_xdata(X[0:i])
    animated_plot.set_ydata(Y[0:i])
    plt.draw()
    plt.pause(0.1)