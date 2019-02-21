#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct
import numpy as np
import cv2 as cv


DATA_PORT = 3125
QR_CODE_PORT = 3126

data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(("0.0.0.0", DATA_PORT))

qr_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
qr_sock.bind(("0.0.0.0", QR_CODE_PORT))


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

left_coords = np.empty((848, 2))
right_coords = np.empty((848, 2))


qrCodes = {}

#avoidance_areas = np.ones((800, 800, 3), np.uint8) * 255
#avoidance_areas = cv.imread('avoidance_areas/avoidance_areas.png')
avoidance_areas = cv.imread('avoidance_areas/very_large.png')

amap = np.copy(avoidance_areas)
cv.imshow('a map', amap)
cv.moveWindow('a map', 560, 0)
cv.waitKey(1)

while True:
    inputs, outputs, errors = select.select([data_sock, qr_sock], [], [])
    for oneInput in inputs:
        if oneInput == data_sock:

            data = None

            # empty out the recv buffer (in case fig.canvas.draw is slower than udp packets)
            data_sock.setblocking(0)
            cont=True
            while cont:
                try:
                    tmpData, addr = data_sock.recvfrom(65535)
                except Exception as ee:
                    #print(ee)
                    cont=False
                else:
                    if tmpData:
                        if data is not None:
                            print('throwing away a packet (GUI is too slow)')
                        data = tmpData
                    else:
                        cont=False
            data_sock.setblocking(1)

            #print(len(data))
            scanline = np.fromstring(data, dtype='<u2')
            left_scan_line = scanline[:848] * depth_scale
            left_coords[:,1] = left_scan_line
            left_coords[:,0] = left_xrange * left_scan_line

            right_scan_line = scanline[848:] * depth_scale
            right_coords[:,1] = right_scan_line
            right_coords[:,0] = right_xrange * right_scan_line

            aa = np.dot(rotate_left, left_coords.T).T
            bb = np.dot(rotate_right, right_coords.T).T

        elif oneInput == qr_sock:
            qrData = None
            qr_sock.setblocking(0)
            cont = True
            while cont:
                try:
                    tmpQrData, addr = qr_sock.recvfrom(256)
                except Exception as ee:
                    #print(ee)
                    cont = False
                else:
                    if tmpQrData:
                        if qrData is not None:
                            print('throwing away QR packet (GUI is too slow)')
                        qrData = tmpQrData
                    else:
                        cont = False
            qr_sock.setblocking(1)
            print(len(qrData))
            try:
                camNo, qrId, x, y, z = struct.unpack('iifff', qrData)
            except:
                print('failed to parse QR data')
            else:
                print(camNo, qrId, x, y , z)
                if (camNo == 0):
                    qrAa = np.dot(rotate_left, [x, z]).T
                    qrCodes[qrId] = (qrAa, time.time())
                    #qrMarkersA.set_data(qrAa[0], qrAa[1])
                else:
                    qrBb = np.dot(rotate_right, [x, z]).T
                    qrCodes[qrId] = (qrBb, time.time())
                    #qrMarkersB.set_data(qrBb[0], qrBb[1])

        # Draw a real-time map on a white background
        #  (each pixel represents 1cm x 1cm in real space)
        amap = np.copy(avoidance_areas)

        for pt in aa:
            x = int(round(pt[0]*100) + 400) # graphical coords, in cm
            y = int(800 - round(pt[1]*100)) # graphical coords, in cm
            if (5 <= x < 795) and (5 <= y < 795):
                amap[y, x] = [255, 0, 0]

        for pt in bb:
            x = int(round(pt[0]*100) + 400) # graphical coords, in cm
            y = int(800 - round(pt[1]*100)) # graphical coords, in cm
            if (5 <= x < 795) and (5 <= y < 795):
                amap[y, x] = [255, 0, 0]

        for key, val in qrCodes.items():
            pt = val[0]
            ts = val[1]
            if time.time() - ts < 1.0:
                x = int(round(pt[0]*100) + 400) # graphical coords, in cm
                y = int(800 - round(pt[1]*100)) # graphical coords, in cm
                if (5 <= x < 795) and (5 <= y < 795):
                    font = cv.FONT_HERSHEY_SIMPLEX
                    cv.putText(amap, str(key), (x,y), font, 1, [0,0,0], 2)
                    cv.circle(amap, (x,y), 3, [0,0,255], -1)

        cv.imshow('a map', amap)
        cv.waitKey(1)






