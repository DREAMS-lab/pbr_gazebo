#! /usr/bin/env python
"""
plot_csv.py
Zhiang Chen, June 2021
"""

import os
import numpy as np
import matplotlib.pyplot as plt

files = [f for f in os.listdir('../csv_data/.') if f.endswith('.csv')]
for f in files:
    data = np.genfromtxt('../csv_data/' + f, delimiter=',')
    Xs_1 = []
    Ys_1 = []
    S_1 = []
    Xs_0 = []
    Ys_0 = []
    S_0 = []
    size = 3
    plt.figure()
    for PGA_g, PGV_2_PGA, state in data:
        # print(PGA_g, PGV_2_PGA, state)
        if state == 1:
            Xs_1.append(PGA_g)
            Ys_1.append(PGV_2_PGA)
            S_1.append(size**2)
        else:
            Xs_0.append(PGA_g)
            Ys_0.append(PGV_2_PGA)
            S_0.append(size**2)

    plt.title(' '.join(f.split('.')[0].split('_')))
    plt.scatter(Xs_1, Ys_1, s=S_1, c='r')
    plt.scatter(Xs_0, Ys_0, s=S_0, c='b', marker="v")
    plt.axis([0, 0.55, 0, 0.55])
    plt.xlabel('PGA (g)')
    plt.ylabel('PGV/PGA (s)')
    plt.savefig('../csv_data/' + f.split('.')[0] + '.png')
    plt.close()