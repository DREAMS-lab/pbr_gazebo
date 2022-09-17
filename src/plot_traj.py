#! /usr/bin/env python
"""
plot_traj.py
Zhiang Chen, Sept 2022
"""

from numpy import genfromtxt
import rospkg

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('pbr_gazebo')
my_data = genfromtxt(pkg_path + '/topple_log/60.0_1.0_0.733333333333.csv', delimiter=',')
print(my_data.shape)

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d')


x = my_data[:, 0]
y = my_data[:, 1]
z = my_data[:, 2]

v_x = my_data[:, 7]
v_y = my_data[:, 8]
v_z = my_data[:, 9]

v = np.sqrt(v_x**2 + v_y**2 + v_z**2 )

ax.plot3D(x, y, z, 'gray')
p = ax.scatter3D(x, y, z, c=v, cmap='Greens');

fig.colorbar(p, ax=ax)

plt.show()