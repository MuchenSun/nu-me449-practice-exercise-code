import numpy as np 
# import modern_robotics as mr 
from IKinBodyIterates2 import IKinBodyIterates
from math import *
from matplotlib import pyplot as plt

Blist = np.array([[0, 0, 1,-2, 0, 0],
                  [0, 0, 1,-1, 0, 0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1]]).T 
M = np.array([[1, 0, 0, 0],
              [0, 1, 0, 2],
              [0, 0, 1, 1],
              [0, 0, 0, 1]])
T = np.array([[ 0, 1, 0,-1],
              [-1, 0, 0, 0],
              [ 0, 0, 1, 0],
              [ 0, 0, 0, 1]])
eomg = 0.001
ev = 0.0001

sol1 = np.array([pi/6, 2*pi/3, 2*pi/3, -1])
sol2 = np.array([5*pi/6, 4*pi/3, 4*pi/3, -1])
step = (sol2-sol1) / 100
print(step)

thetalist_vec = []
iteration_vec = []
for i in range(-50, 150, 1):
    thetalist0 = sol1 + i * step
    (thetalistAns, successAns, iteration) = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
    print("(",i,"): ", thetalist0, thetalistAns, iteration)
    iteration_vec.append(iteration)
    if thetalistAns[0] < 1:
        thetalist_vec.append(1)
    else:
        thetalist_vec.append(2)

print("Sol1: ", sol1)
print("Sol2: ", sol2)

tvec = np.linspace(-50, 150, 200)
plt.plot(tvec, iteration_vec)
plt.scatter(tvec, thetalist_vec, c='red')
plt.show()
