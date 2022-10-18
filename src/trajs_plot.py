from numpy import genfromtxt
import os
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib



files = [f for f in os.listdir('./topple_log/') if f.startswith('0.0')]


font = {'family' : 'sans-serif',
        'size'   : 14}

matplotlib.rc('font', **font)
matplotlib.rc('axes', labelsize=14)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-8, 8)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xticks([-15, 0, 15])
ax.set_yticks([-15, 0, 15])
ax.set_zticks([-5, 0, 5])

n = 0
for f in files:
	my_data = genfromtxt('./topple_log/'+f, delimiter=',')
	if len(my_data.shape)==1:
		continue
	x = my_data[:, 0]
	y = my_data[:, 1]
	z = my_data[:, 2]
	if (z.max() - z.min()) < 0.5:
		continue
	v_x = my_data[:, 7]
	v_y = my_data[:, 8]
	v_z = my_data[:, 9]
	v = np.sqrt(v_x**2 + v_y**2 + v_z**2 )
	x_site = my_data[:, 13]
	y_site = my_data[:, 14]
	z_site = my_data[:, 15]
	x = np.subtract(x, x_site)
	y = np.subtract(y, y_site)
	z = np.subtract(z, z_site)
	p = ax.scatter3D(x, y, z, c=v, cmap='plasma', s=1**2)
	n += 1

print("toppled trajectories #: ", n)
fig.colorbar(p, ax=ax, label='velocity (m/s)', shrink=0.7)
ax.view_init(elev=15, azim=-78)
plt.show()


