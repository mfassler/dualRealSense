#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


import select
import socket
import struct
import numpy as np
import cv2



START_MAGIC = b"__HylPnaJY_START_JPG "
STOP_MAGIC = b"_g1nC_EOF"

DATA_PORT = 3101
IMAGE_PORT = 3123

data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(("0.0.0.0", DATA_PORT))

image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
image_sock.bind(("0.0.0.0", IMAGE_PORT))



def showImage(data):
    aa = np.frombuffer(data, np.uint8)
    print(len(aa))
    im = cv2.imdecode(aa, cv2.IMREAD_UNCHANGED)
    for i in range(nboxes):
        if distances[i] < 1.0:
            cimg2 = cv2.rectangle(im, (bboxes[i][0], bboxes[i][2]), (bboxes[i][1], bboxes[i][3]),
                (0,0,255), 2)
        elif distances[i] < 2.0:
            cimg2 = cv2.rectangle(im, (bboxes[i][0], bboxes[i][2]), (bboxes[i][1], bboxes[i][3]),
                (0,255,255), 2)
    cv2.imshow('asdf', im)
    cv2.waitKey(1)


jpgDataSize = 0
jpgPackets = []
inBand = False

nboxes = 0
distances = np.empty(16, np.float)
bboxes = np.empty((16,4), np.int32)

while True:
    inputs, outputs, errors = select.select([data_sock, image_sock], [], [])
    for oneInput in inputs:
        if oneInput == image_sock:
            data, addr = image_sock.recvfrom(65535)

            if inBand:
                if data == STOP_MAGIC:
                    inBand = False
                    if len(jpgPackets) > 1:
                        jpgData = b''.join(jpgPackets)
                        if jpgDataSize == len(jpgData):
                            showImage(jpgData)
                        else:
                            print('image size doesn\'t match')
                    else:
                        print('no data')
                else:
                    jpgPackets.append(data)
        
            else:
                if data.startswith(START_MAGIC):
                    jpgDataSize = int(data[-10:], 10)
                    jpgPackets = []
                    inBand = True

        if oneInput == data_sock:
            data, addr = data_sock.recvfrom(65535)
            #numPackets, = struct.unpack('!i', data[:4])
            nboxes = data[3]
            #print("%d, %d" % (nboxes, len(data)-4))
            for i in range(nboxes):
                iStart = i*20 + 4
                iStop = iStart + 20
                distances[i], bboxes[i][0], bboxes[i][1], bboxes[i][2], bboxes[i][3] = \
                    struct.unpack('fiiii', data[iStart:iStop])
                #print("%d  %f, %d %d %d %d" % (i, distances[i], bboxes[i][0], bboxes[i][1], bboxes[i][2], bboxes[i][3] ))
            print(distances[:nboxes])


