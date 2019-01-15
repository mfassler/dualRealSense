#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt


DATA_PORT = 3125

data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(("0.0.0.0", DATA_PORT))



depth_scale = 0.001

left_coords = np.empty((848, 2))
right_coords = np.empty((848, 2))

#left_xrange = (np.arange(848) - rgb0_intrinsics.ppx) / rgb0_intrinsics.fx  # this doesn't change
#right_xrange = (np.arange(848) - rgb1_intrinsics.ppx) / rgb1_intrinsics.fx  # this doesn't change
left_xrange = (np.arange(848) - 424) / 615.0
right_xrange = (np.arange(848) - 424) / 615.0

theta = np.radians(69.3/ 2.0)

rotate_left = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]])

rotate_right = np.array([
    [np.cos(-theta), -np.sin(-theta)],
    [np.sin(-theta), np.cos(-theta)]])


plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
plt.axis('equal')
plt.ylim([0, 8.0])
plt.xlim([-5.0, 5.0])

line_range = np.arange(848*2)
lineA, = ax.plot(left_xrange, np.zeros(848), 'r.')
lineB, = ax.plot(right_xrange, np.zeros(848), 'b.')

fig.canvas.draw()
left_coords = np.empty((848, 2))
right_coords = np.empty((848, 2))



while True:
    inputs, outputs, errors = select.select([data_sock], [], [])
    for oneInput in inputs:
        if oneInput == data_sock:
            data, addr = data_sock.recvfrom(65535)
            print(len(data))
            scanline = np.fromstring(data, dtype='<u2')
            left_scan_line = scanline[:848] * depth_scale
            left_coords[:,1] = left_scan_line
            left_coords[:,0] = left_xrange * left_scan_line

            right_scan_line = scanline[848:] * depth_scale
            right_coords[:,1] = right_scan_line
            right_coords[:,0] = right_xrange * right_scan_line

            aa = np.dot(rotate_left, left_coords.T).T
            bb = np.dot(rotate_right, right_coords.T).T

            lineA.set_data(aa[:,0], aa[:,1])
            lineB.set_data(bb[:,0], bb[:,1])

            fig.canvas.draw()



