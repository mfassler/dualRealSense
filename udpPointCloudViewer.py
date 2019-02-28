#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


import select
import socket
import struct
import numpy as np
import open3d

from rigid_transform import rigid_transform_3D


DATA_PORT = 9221
data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(("0.0.0.0", DATA_PORT))

DATA_PORT2 = 9222
data_sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock2.bind(("0.0.0.0", DATA_PORT2))

np.set_printoptions(precision=16)
notAddedYet = True
numPts = 0

vis = open3d.Visualizer()
vis.create_window(width=800, height=600, left=1100, right=50)  # "right" is "top"

pcd = open3d.PointCloud()
prev_pcd = open3d.PointCloud()


cur_points = np.empty((0, 3))
prev_points = np.empty((0, 3))
while True:
    inputs, outputs, errors = select.select([data_sock, data_sock2], [], [])
    for oneInput in inputs:
        if oneInput == data_sock:
            data, addr = data_sock.recvfrom(32000)

            if len(data) % 12 != 0:
                print("Error:  data should be divisible by 12")
                assert False
            else:
                numPts = int(len(data) / 12)
                print('a', numPts)

                cur_points = np.empty((numPts, 3))
                cur_colors = np.empty((numPts, 3))

                for i in range(numPts):
                    iStart = i*12
                    iStop = iStart + 12
                    x,y,z = struct.unpack('fff', data[iStart:iStop])
                    cur_points[i] = (x,y,z)
                    cur_colors[i] = 0, 0.5, 0

                pcd.points = open3d.Vector3dVector(cur_points)
                pcd.colors = open3d.Vector3dVector(cur_colors)

        if oneInput == data_sock2:
            data, addr = data_sock2.recvfrom(32000)

            if len(data) % 12 != 0:
                print("Error:  data should be divisible by 12")
                assert False
            else:
                numPts = int(len(data) / 12)
                print('b', numPts)

                prev_points = np.empty((numPts, 3))
                prev_colors = np.empty((numPts, 3))

                for i in range(numPts):
                    iStart = i*12
                    iStop = iStart + 12
                    x,y,z = struct.unpack('fff', data[iStart:iStop])
                    prev_points[i] = (x,y,z)
                    prev_colors[i] = 0.5, 0, 0

                prev_pcd.points = open3d.Vector3dVector(prev_points)
                prev_pcd.colors = open3d.Vector3dVector(prev_colors)

    if len(prev_points) > 5:
        if len(prev_points) == len(cur_points):
            try:
                R, tt = rigid_transform_3D(prev_points, cur_points)
            except np.linalg.linalg.LinAlgError as e:
                # sometimes the SVD doesn't converge.  If rare, this isn't a problem
                print("  ******* WARNING:  ", e)
                print("  ******* WARNING:  ", e)
                print("  ******* WARNING:  ", e)
                print("  ******* WARNING:  ", e)
            else:
                print(R, tt)

    if notAddedYet and numPts > 50:
        vis.add_geometry(pcd)
        vis.add_geometry(prev_pcd)
        notAddedYet = False

    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()


