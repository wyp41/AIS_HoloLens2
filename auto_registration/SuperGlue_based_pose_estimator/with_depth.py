from pathlib import Path
import argparse
import cv2
import matplotlib.cm as cm
import torch
import numpy as np
import math
import time
import socket
import threading
import struct
import abc
from datetime import datetime, timedelta
from collections import namedtuple, deque
from enum import Enum
from scipy.spatial.transform import Rotation as R

from models.matching import Matching
from models.utils import (AverageTimer, VideoStreamer,
                          make_matching_plot_fast, frame2tensor)
import os
metric3d_dir = os.path.dirname(__file__)
import torch
from mmengine import Config, DictAction
from mono.model.monodepth_model import get_configured_monodepth_model
import numpy as np



# np.warnings.filterwarnings('ignore')

# Definitions
# Protocol Header Format
# see https://docs.python.org/2/library/struct.html#format-characters
VIDEO_STREAM_HEADER_FORMAT = "@qIIII18f"

VIDEO_FRAME_STREAM_HEADER = namedtuple(
    'SensorFrameStreamHeader',
    'Timestamp ImageWidth ImageHeight PixelStride RowStride fx fy '
    'PVtoWorldtransformM11 PVtoWorldtransformM12 PVtoWorldtransformM13 PVtoWorldtransformM14 '
    'PVtoWorldtransformM21 PVtoWorldtransformM22 PVtoWorldtransformM23 PVtoWorldtransformM24 '
    'PVtoWorldtransformM31 PVtoWorldtransformM32 PVtoWorldtransformM33 PVtoWorldtransformM34 '
    'PVtoWorldtransformM41 PVtoWorldtransformM42 PVtoWorldtransformM43 PVtoWorldtransformM44 '
)

RM_STREAM_HEADER_FORMAT = "@qIIII16f"

RM_FRAME_STREAM_HEADER = namedtuple(
    'SensorFrameStreamHeader',
    'Timestamp ImageWidth ImageHeight PixelStride RowStride '
    'rig2worldTransformM11 rig2worldTransformM12 rig2worldTransformM13 rig2worldTransformM14 '
    'rig2worldTransformM21 rig2worldTransformM22 rig2worldTransformM23 rig2worldTransformM24 '
    'rig2worldTransformM31 rig2worldTransformM32 rig2worldTransformM33 rig2worldTransformM34 '
    'rig2worldTransformM41 rig2worldTransformM42 rig2worldTransformM43 rig2worldTransformM44 '
)

# Each port corresponds to a single stream type
VIDEO_STREAM_PORT = 23940
AHAT_STREAM_PORT = 23941

HOST = '192.168.0.105'
# HOST = '192.168.1.132'
# HOST = '127.0.0.1'


HundredsOfNsToMilliseconds = 1e-4
MillisecondsToSeconds = 1e-3

torch.set_grad_enabled(False)

class pnp():
    def __init__(self):

        self.cmd = "get"
        self.key = "get"
        self.dep = []
        self.calibPos = [0, 0, 0, 0, 1, 0, 0]
        # self.calibPos = [0, 0, 0, -0.707, 0, 0, 0.707]
        self.camPos = [0, 0, 0, 0, 0, 0, 1.0]
        self.corPos = [0, 0, 0, 0, 0, 0, 1.0]
        self.new_cmd = False
        self.calibed = False
        self.pv_trans = []
        self.pv_rot = []
        self.err = 10000
        self.mean_err = 0
        self.mean_std = 0
        self.c = 0
        self.c1 = 0

        self.origin = [[0.0, 0, 0],  [0, -0.02, 0], [0.08, 0, 0], [0.045, 0.04, 0]]
        # self.origin = [[0.0, 0, 0], [0, 0.01, 0], [0, -0.01, 0], [0.02, 0.025, 0], [0.03, 0, 0]], [0, 0.02, 0]
        # self.origin = [[0.0, 0, 0], [0, 0.02, 0], [0, -0.02, 0], [0.08, 0, 0], [0.045, 0.04, 0], [0.04, -0.04, 0], [0.06, -0.04, 0]]
        self.marker = []

        # self.imgs = [[280, 223], [280, 192], [281, 257], [429, 221], [359, 158]]
        # self.imgs = [[360, 147], [392, 143], [325, 146], [451, 193], [366, 233]]
        # self.imgs = [[251,246],[252,203],[251,290],[328,144],[389,240]], [329, 78]
        self.imgs = [[330,132],  [328,183], [525,142], [444, 28]]

        # self.imgs = [[213,180], [213,133], [216,225], [295,67], [360,168]]

        # self.imgs = [[326, 152], [328, 89], [326, 204], [577, 156], [466, 27]]
        # self.imgs = [[327, 199], [331, 158], [322, 241], [538, 208], [442, 125], [422, 279], [482, 277]]
        self.new_imgs = []
        

        self.trans = [0, 0, 0]
        self.rot = [[1,0,0], 
                    [0,1,0], 
                    [0,0,1]]
        
        self.std = 0
        
        self.cameraMatrix = np.float32([ 6.5664077162555463e+02, 0., 3.2841613945979151e+02, 0.,
       6.5705173539981251e+02, 2.2792930298550786e+02, 0., 0., 1. ]).reshape((3,3))
        self.distCoeffs = np.float32([0, 0, 0, 0, 0])
        
        t1 = threading.Thread(target=self.calib_comm)
        t2 = threading.Thread(target=self.keyboard_read)
        t3 = threading.Thread(target=self.cam_comm)
        # t1.start()
        t2.start()
        # t3.start()

    # def SVD_ICP(self, posA, posB):
    #     N = len(posA)
    #     A = np.array(posA)
    #     B = np.array(posB)
    #     centroid_A = np.mean(A, axis=0)
    #     centroid_B = np.mean(B, axis=0)
    #     A_normalized = (A-np.tile(centroid_A, (N, 1)))
    #     B_normalized = (B-np.tile(centroid_B, (N, 1)))
    #     H = np.matmul(A_normalized.T, B_normalized)
    #     U, S, VT = np.linalg.svd(H)
    #     V = VT.T
    #     R = np.matmul(V, U.T)
    #     if np.linalg.det(R) < 0:
    #         print("reflection detected")
    #         V[:,2] = -1*V[:,2]
    #         R = np.matmul(V, U.T)
    #     t = -1*np.dot(R, centroid_A) + centroid_B
    #     self.estimated_points = self.transform(R, t, self.originA)
    #     return(R, t)
    
    # def transform(self, R, t, p_list):
    #     p_list = np.array(p_list).T
    #     n_list = np.matmul(R, p_list)
    #     n_list[0, :] = n_list[0, :] + t[0]
    #     n_list[1, :] = n_list[1, :] + t[1]
    #     n_list[2, :] = n_list[2, :] + t[2]
    #     n_list = n_list.T
    #     return n_list
    
    def set_cameraMatrix(self, mat):
        self.cameraMatrix = np.float32(mat).reshape((3,3))
    
    def set_campose(self, trans, rot):
        self.pv_trans = trans
        self.pv_rot = rot
        # print(self.pv_trans, self.pv_rot, self.camPos, self.cameraMatrix)

    def solve(self):
        self.marker = np.float32(self.marker)
        self.new_imgs = np.float32(self.new_imgs)
        if len(self.new_imgs) > 3:
            res = cv2.solvePnPRansac(self.marker, self.new_imgs, self.cameraMatrix, self.distCoeffs)
            # print(res)
            if res[0]:
                self.trans = res[2]
                self.rot = cv2.Rodrigues(res[1])[0]
                self.corPos = self.camPos
                # pv_pos = list(self.pv_trans) + list((R.from_matrix(self.pv_rot)).as_quat())
                # self.corPos = pv_pos
                if len(self.dep) != 0:
                    self.depth_cal()
                # print(self.trans, n_r)
                # self.key = "c"
                # self.new_cmd = True
        # self.rot_params_rv(res[1])
    
    def depth_cal(self):
        self.marker = np.float32(self.marker)
        self.new_imgs = np.float32(self.new_imgs)
        self.dep = np.float32(self.dep)
        # print(self.dep)
        p = np.array([0,0,1])
        zc = []
        zc_mean = []
        dep_map = []
        for i in range(len(self.new_imgs)):
            p[0] = self.new_imgs[i][0]
            p[1] = self.new_imgs[i][1]
            div = np.matmul(np.matmul(np.linalg.inv(self.rot), np.linalg.inv(self.cameraMatrix)),p)
            # print(np.float32(self.marker[i]))
            divd = np.matmul(np.linalg.inv(self.rot), self.trans).T + np.float32(self.marker[i])
            # print(divd[0])
            # print(div)
            zc.append(divd[0] / div)
            # print(type(self.dep))
            dep_map.append(self.dep[int(p[1]), int(p[0])])
        for i in zc:
            zc_mean.append(np.mean(i))
        
        # print(self.dep)
        z1 = np.float32(zc_mean)-zc_mean[0]
        z2 = np.float32(dep_map)-dep_map[0]
        z2 = (z2/z2[1])*z1[1]
        # print("zc", z1)
        # print("dep", z2)
        # print(np.std(z1), np.std(z2), np.std(z1) - np.std(z2))
        self.std = abs(np.mean(z1) - np.mean(z2))
        print(self.std)

        self.mean_std = (self.std + self.mean_std*self.c1)/(self.c1 + 1)
        self.c1 += 1

        # self.std = np.std(zc_mean)
        # print(self.mean_std)

    def draw_bbox(self, frame):
        l = 0.2
        h = 0.2
        w = 0.01
        x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
        y_corners = [h/2, h/2, h/2, h/2, -h/2, -h/2, -h/2, -h/2]
        z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
        for p in self.marker:
            x_corners.append(p[0])
            y_corners.append(p[1])
            z_corners.append(p[2])
        corners_3d = np.matmul(self.rot, np.vstack([x_corners, y_corners, z_corners]))
        # print(corners_3d)
        corners_3d[0,:] = corners_3d[0,:] + self.trans[0]
        corners_3d[1,:] = corners_3d[1,:] + self.trans[1]
        corners_3d[2,:] = corners_3d[2,:] + self.trans[2]
        # print(corners_3d)
        corners_2d = np.matmul(self.cameraMatrix, corners_3d).T
        corners_2d = corners_2d[:,0:2]/corners_2d[:,2:3]
        corners_2d = corners_2d[:,0:2].astype(np.int32)
        # print(corners_2d)

        img_np = np.asarray(frame)
        color = (0, 0, 0) # BGR

        for k in range(4):
            i, j = k, (k+1) % 4
            cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 5)
            i, j = k + 4, (k + 1) % 4 + 4
            cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 5)
            i, j = k, k + 4
            cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 5)
        
        error = 0
        # print(corners_2d, self.new_imgs)
        for k in range(len(self.new_imgs)):
            pred_pos = []
            pred_pos.append(corners_2d[k+8,0])
            pred_pos.append(corners_2d[k+8,1])
            # print(pred_pos)
            
            error += self.get_dis(pred_pos, self.new_imgs[k])
            cv2.circle(frame, pred_pos, 5, color, -1)
        
        error = error/len(self.new_imgs)

        # self.key = "c"
        # self.new_cmd = True

        print(error)
        if error < 100:
            self.mean_err = (error + self.mean_err*self.c) / (self.c + 1)
            self.c += 1

        if error < self.err and len(self.new_imgs) > 3:
            self.err = error
             
            # print("mean_error", self.mean_err)
            if error > 2:
                self.key = "c"
                self.new_cmd = True
                print("calibed")
            n_r = R.from_matrix(self.rot)
            n_r = list(n_r.as_quat())
            print(error)
            print(self.std)
            print(self.trans, n_r)

        return img_np
        

    def comp_calib(self):
        n_t = np.float32(self.trans).reshape((1,3))
        r = R.from_quat([0, 1, 0, 0])
        r = np.float32(r.as_matrix())
        n_r = np.matmul(self.rot, r)
        n_r = R.from_matrix(n_r)
        n_r = n_r.as_quat()
        return list(n_t[0]), list(n_r)
    
    def mul(self, trans1, rot1, trans2, rot2):
        trans2 = np.float32(trans2)
        n_r = np.matmul(rot1, rot2)
        n_t = np.matmul(rot1, trans2).reshape((1,3)) + np.float32(trans1)
        n_t = list(n_t[0])
        return n_t, n_r

    def comp_obj(self):
        o_trans = self.calibPos[:3]
        o_rot = self.calibPos[3:]
        cam_trans = self.corPos[:3]
        cam_rot = self.corPos[3:]
        cam_trans[2] = -cam_trans[2]
        cam_rot[0] = -cam_rot[0]
        cam_rot[1] = -cam_rot[1]

        rec_trans = [0.002, 0.002, -0.058]
        rec_rot = [-1, 0.00, -0.000, 0.009]

        # rec_trans = [0, -0.085, 0.072]
        # rec_rot = [1, 0, 0, 0]

        # print("cam_trans:", cam_trans, "pv_trans:", self.pv_trans)

        r = R.from_quat(o_rot)
        r = np.float32(r.as_matrix())
        cr = R.from_quat(cam_rot)
        cr = np.float32(cr.as_matrix())
        rr = R.from_quat(rec_rot)
        rr = np.float32(rr.as_matrix())
        # mr = R.from_quat(mrr)
        # mr = np.float32(mr.as_matrix())

        target_trans = self.trans
        tr = self.rot

        # target_trans = [0, 0, 0.2]
        # tr = rr

        ct = np.float32(cam_trans)
        # ct = np.float32(self.pv_trans) + np.float32(cam_trans)
        # ct = ct / 2.0
        # print(ct, self.pv_trans)

        n_t, n_r = self.mul(ct, cr, rec_trans, rr)
        n_t, n_r = self.mul(n_t, n_r, target_trans, tr)
        n_t, n_r = self.mul(n_t, n_r, o_trans, r)
        # n_t, n_r = self.mul(n_t, n_r, mrt, mr)
        n_r = R.from_matrix(n_r)
        n_r = list(n_r.as_quat())

        return n_t, n_r
        
    
    def add_image_point(self, x, y):
        self.imgs.append([x,y])
    
    def get_dis(self, a, b):
        m = 0
        for i in range(len(a)):
            m += (a[i] - b[i])**2
        m = math.sqrt(m)
        return m
    
    def get_depth(self, depth):
        self.dep = depth
        # print("dep")

    def fetch_corr(self, mkpt0, mkpt1, conf):
        dis = 10000
        ind = -1
        self.new_imgs = []
        self.marker = []
        for i in range(len(self.imgs)):
            for j in range(len(mkpt0)):
                d = self.get_dis(self.imgs[i], mkpt0[j])
                if d < dis:
                    dis = d
                    ind = j
            if dis < 10:
                self.new_imgs.append(mkpt1[ind])
                self.marker.append(self.origin[i])
            ind = -1
            dis = 10000
        # for i in self.new_imgs:
        #     print(self.dep.shape)
        #     print(self.dep[int(i[1]), int(i[0])])

        # print(len(self.new_imgs))

        return len(self.new_imgs)
    
    def calib_comm(self):
        self.calib_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.calib_sock.connect((HOST, 25003))
        # self.calib_sock.connect(("127.0.0.1", 25003))
        while True:
            time.sleep(0.5)
            if self.new_cmd:
                self.cmd = self.key
                self.new_cmd = False
            else:
                self.cmd = "get"
            if self.cmd == "q":
                break
                # print("calibed:", self.cmd)
            # time.sleep(0.5) #sleep 0.5 sec
            if self.cmd == "c":
                trans, rot = self.comp_obj()
                # trans[1] += 1.6
                trans[2] = -trans[2]
                rot[0] = -rot[0]
                rot[1] = -rot[1]
                self.cmd = "e" + str(tuple(trans + rot))
                print(self.cmd)
            # print(self.cmd)
            self.calib_sock.sendall(self.cmd.encode("UTF-8")) #Converting string to Byte, and sending it to C#
            receivedPos = self.calib_sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
            # print("objPos:", receivedPos)
            # locPos = receivedPos[1:-1].split(',')
            # for i in range(len(locPos)):
            #     locPos[i] = float(locPos[i])
            # self.camPos = locPos

    def cam_comm(self):
        self.cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cam_sock.connect((HOST, 25004))
        # self.calib_sock.connect(("127.0.0.1", 25003))
        while True:
            time.sleep(0.5)
            if self.cmd == "q":
                break
            cmd = "get"
            self.cam_sock.sendall(cmd.encode("UTF-8")) #Converting string to Byte, and sending it to C#
            receivedPos = self.cam_sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
            # print("camPos:", receivedPos)
            locPos = receivedPos[1:-1].split(',')
            for i in range(len(locPos)):
                locPos[i] = float(locPos[i])
            self.camPos = locPos

    def keyboard_read(self):
        while True:
            self.key = str(input("ctrl:"))
            self.new_cmd = True
            if self.key == "q":
                print(self.mean_err, self.mean_std)
                break
            elif self.key == "c":
                self.err = 10000

class SensorType(Enum):
    VIDEO = 1
    AHAT = 2
    LONG_THROW_DEPTH = 3
    LF_VLC = 4
    RF_VLC = 5

class FrameReceiverThread(threading.Thread):
    def __init__(self, host, port, header_format, header_data):
        super(FrameReceiverThread, self).__init__()
        self.header_size = struct.calcsize(header_format)
        self.header_format = header_format
        self.header_data = header_data
        self.host = host
        self.port = port
        self.latest_frame = None
        self.latest_header = None
        self.socket = None
        self.K1 = []
        self.trans = []
        self.rot = []

    def get_data_from_socket(self):
        # read the header in chunks
        reply = self.recvall(self.header_size)

        if not reply:
            print('ERROR: Failed to receive data from stream.')
            return

        data = struct.unpack(self.header_format, reply)
        header = self.header_data(*data)

        # read the image in chunks
        image_size_bytes = header.ImageHeight * header.RowStride
        image_data = self.recvall(image_size_bytes)

        return header, image_data

    def recvall(self, size):
        msg = bytes()
        while len(msg) < size:
            part = self.socket.recv(size - len(msg))
            if part == '':
                break  # the connection is closed
            msg += part
        return msg

    def start_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        # send_message(self.socket, b'socket connected at ')
        print('INFO: Socket connected to ' + self.host + ' on port ' + str(self.port))

    def start_listen(self):
        t = threading.Thread(target=self.listen)
        t.start()

    @abc.abstractmethod
    def listen(self):
        return

    @abc.abstractmethod
    def get_mat_from_header(self, header):
        return


class VideoReceiverThread(FrameReceiverThread):
    def __init__(self, host):
        super().__init__(host, VIDEO_STREAM_PORT, VIDEO_STREAM_HEADER_FORMAT,
                         VIDEO_FRAME_STREAM_HEADER)
        self.running = True

    def listen(self):
        while self.running:
            self.latest_header, image_data = self.get_data_from_socket()
            self.latest_frame = np.frombuffer(image_data, dtype=np.uint8).reshape((self.latest_header.ImageHeight,
                                                                                   self.latest_header.ImageWidth,
                                                                                   self.latest_header.PixelStride))
            K2 = self.get_mat_from_header(self.latest_header)
            self.trans = K2[:3, 3]
            self.rot = K2[:3, :3]
            # print("pvtoworld:", self.trans, self.rot)
            fx = float(self.latest_header[5])
            fy = float(self.latest_header[6])
            u0 = self.latest_header.ImageWidth/2.0
            v0 = self.latest_header.ImageHeight/2.0
            # print(self.latest_header.ImageWidth, self.latest_header.ImageHeight)
            self.K1 = np.matrix([[fx, 0, u0],
                           [0, fy, v0],
                           [0, 0, 1]])
            

    def get_mat_from_header(self, header):
        pv_to_world_transform = np.array(header[7:24]).reshape((4, 4)).T
        return pv_to_world_transform




class AhatReceiverThread(FrameReceiverThread):
    def __init__(self, host):
        super().__init__(host,
                         AHAT_STREAM_PORT, RM_STREAM_HEADER_FORMAT, RM_FRAME_STREAM_HEADER)
        self.running = True

    def listen(self):
        while self.running:
            self.latest_header, image_data = self.get_data_from_socket()
            self.latest_frame = np.frombuffer(image_data, dtype=np.uint16).reshape((self.latest_header.ImageHeight,
                                                                                    self.latest_header.ImageWidth))
            # print(self.get_mat_from_header(self.latest_header))

    def get_mat_from_header(self, header):
        rig_to_world_transform = np.array(header[5:22]).reshape((4, 4)).T
        return rig_to_world_transform

class depth_estimator():
    def __init__(self, model, img, intrinsic=[6.5664077162555463e+02, 6.5705173539981251e+02, 3.2841613945979151e+02, 2.2792930298550786e+02]):
        self.model = model
        self.running = True
        self.depth = []
        self.intrinsic = intrinsic
        self.color_frame = img
        t = threading.Thread(target=self.estimate)
        t.start()

    def fetch_image(self, img, intrinsic=[6.5664077162555463e+02, 6.5705173539981251e+02, 3.2841613945979151e+02, 2.2792930298550786e+02]):
        self.color_frame = img
        # print("fetch", self.color_frame)
        self.intrinsic = intrinsic

    def estimate(self):
        while self.running:
            input_size = (616, 1064) # for vit model
            padding = [123.675, 116.28, 103.53]
            # rgb_origin = rgb_file[:, :, ::-1]
            rgb_origin = self.color_frame[:, :, ::-1]

            # print("rgb", rgb_origin)

            #### ajust input size to fit pretrained model
            # keep ratio resize
            # input_size = (544, 1216) # for convnext model
            h, w = rgb_origin.shape[:2]
            scale = min(input_size[0] / h, input_size[1] / w)
            rgb = cv2.resize(rgb_origin, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)
            # remember to scale intrinsic, hold depth
            intrinsic = [self.intrinsic[0] * scale, self.intrinsic[1] * scale, self.intrinsic[2] * scale, self.intrinsic[3] * scale]
            # padding to input_size
            h, w = rgb.shape[:2]
            pad_h = input_size[0] - h
            pad_w = input_size[1] - w
            pad_h_half = pad_h // 2
            pad_w_half = pad_w // 2
            rgb = cv2.copyMakeBorder(rgb, pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half, cv2.BORDER_CONSTANT, value=padding)
            pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]

            #### normalize
            mean = torch.tensor([123.675, 116.28, 103.53]).float()[:, None, None]
            std = torch.tensor([58.395, 57.12, 57.375]).float()[:, None, None]
            rgb = torch.from_numpy(rgb.transpose((2, 0, 1))).float()
            rgb = torch.div((rgb - mean), std)
            rgb = rgb[None, :, :, :].cuda()

            ###################### canonical camera space ######################
            # inference
            with torch.no_grad():
                pred_depth, confidence, output_dict = self.model.inference({'input': rgb})

            # un pad
            pred_depth = pred_depth.squeeze()
            pred_depth = pred_depth[pad_info[0] : pred_depth.shape[0] - pad_info[1], pad_info[2] : pred_depth.shape[1] - pad_info[3]]

            # upsample to original size
            pred_depth = torch.nn.functional.interpolate(pred_depth[None, None, :, :], rgb_origin.shape[:2], mode='bilinear').squeeze()
            ###################### canonical camera space ######################

            #### de-canonical transform
            canonical_to_real_scale = intrinsic[0] / 1000.0 # 1000.0 is the focal length of canonical camera
            pred_depth = pred_depth * canonical_to_real_scale # now the depth is metric
            pred_depth = torch.clamp(pred_depth, 0, 300)

            self.depth = pred_depth.cpu().numpy()

            # print(self.depth)

            # print("depth get")

if __name__ == "__main__":
    pose_solver = pnp()
    dependencies = ['torch', 'torchvision']
    model = torch.hub.load('yvanyin/metric3d', 'metric3d_vit_small', pretrain=True)
    model.cuda().eval()
    
    parser = argparse.ArgumentParser(
        description='SuperGlue demo',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--input', type=str, default='0',
        help='ID of a USB webcam, URL of an IP camera, '
             'or path to an image directory or movie file')
    parser.add_argument(
        '--output_dir', type=str, default=None,
        help='Directory where to write output frames (If None, no output)')

    parser.add_argument(
        '--image_glob', type=str, nargs='+', default=['*.png', '*.jpg', '*.jpeg'],
        help='Glob if a directory of images is specified')
    parser.add_argument(
        '--skip', type=int, default=1,
        help='Images to skip if input is a movie or directory')
    parser.add_argument(
        '--max_length', type=int, default=1000000,
        help='Maximum length if input is a movie or directory')
    parser.add_argument(
        '--resize', type=int, nargs='+', default=[640, 360],
        help='Resize the input image before running inference. If two numbers, '
             'resize to the exact dimensions, if one number, resize the max '
             'dimension, if -1, do not resize')

    parser.add_argument(
        '--superglue', choices={'indoor', 'outdoor'}, default='indoor',
        help='SuperGlue weights')
    parser.add_argument(
        '--max_keypoints', type=int, default=-1,
        help='Maximum number of keypoints detected by Superpoint'
             ' (\'-1\' keeps all keypoints)')
    parser.add_argument(
        '--keypoint_threshold', type=float, default=0.005,
        help='SuperPoint keypoint detector confidence threshold')
    parser.add_argument(
        '--nms_radius', type=int, default=4,
        help='SuperPoint Non Maximum Suppression (NMS) radius'
        ' (Must be positive)')
    parser.add_argument(
        '--sinkhorn_iterations', type=int, default=20,
        help='Number of Sinkhorn iterations performed by SuperGlue')
    parser.add_argument(
        '--match_threshold', type=float, default=0.2,
        help='SuperGlue match threshold')

    parser.add_argument(
        '--show_keypoints', action='store_true',
        help='Show the detected keypoints')
    parser.add_argument(
        '--no_display', action='store_true',
        help='Do not display images to screen. Useful if running remotely')
    parser.add_argument(
        '--force_cpu', action='store_true',
        help='Force pytorch to run in CPU mode.')

    opt = parser.parse_args()
    # print(opt)

    if len(opt.resize) == 2 and opt.resize[1] == -1:
        opt.resize = opt.resize[0:1]
    if len(opt.resize) == 2:
        print('Will resize to {}x{} (WxH)'.format(
            opt.resize[0], opt.resize[1]))
    elif len(opt.resize) == 1 and opt.resize[0] > 0:
        print('Will resize max dimension to {}'.format(opt.resize[0]))
    elif len(opt.resize) == 1:
        print('Will not resize images')
    else:
        raise ValueError('Cannot specify more than two integers for --resize')

    device = 'cuda' if torch.cuda.is_available() and not opt.force_cpu else 'cpu'
    print('Running inference on device \"{}\"'.format(device))
    config = {
        'superpoint': {
            'nms_radius': opt.nms_radius,
            'keypoint_threshold': opt.keypoint_threshold,
            'max_keypoints': opt.max_keypoints
        },
        'superglue': {
            'weights': opt.superglue,
            'sinkhorn_iterations': opt.sinkhorn_iterations,
            'match_threshold': opt.match_threshold,
        }
    }
    matching = Matching(config).eval().to(device)
    keys = ['keypoints', 'scores', 'descriptors']


    # video_receiver = VideoReceiverThread(HOST)
    # video_receiver.start_socket()

    # video_receiver.start_listen()

    vs = VideoStreamer(opt.input, opt.resize, opt.skip,
                       opt.image_glob, opt.max_length)
    frame, ret = vs.next_frame()
    assert ret, 'Error when reading the first frame (try different --input?)'

    # print(frame)
    # frame = cv2.imread("tissue.png", cv2.IMREAD_GRAYSCALE)
    frame = cv2.imread("init.png", cv2.IMREAD_GRAYSCALE)
    # frame = cv2.imread("pic/pic1.png", cv2.IMREAD_GRAYSCALE)
    frame = cv2.resize(frame, (640, 360))
    # print(frame)
    frame_tensor = frame2tensor(frame, device)
    last_data = matching.superpoint({'image': frame_tensor})
    last_data = {k+'0': last_data[k] for k in keys}
    last_data['image0'] = frame_tensor
    last_frame = frame
    last_image_id = 0
    leng = 0

    color_frame = cv2.imread("init.png")
    # color_frame = cv2.imread("tissue.png")
    depth_solve = depth_estimator(model, color_frame)

    if opt.output_dir is not None:
        print('==> Will write outputs to {}'.format(opt.output_dir))
        Path(opt.output_dir).mkdir(exist_ok=True)

    # Create a window to display the demo.
    if not opt.no_display:
        cv2.namedWindow('SuperGlue matches', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('SuperGlue matches', 640*2, 360)
    else:
        print('Skipping visualization, will not show a GUI.')

    # Print the keyboard help menu.
    print('==> Keyboard control:\n'
          '\tn: select the current frame as the anchor\n'
          '\te/r: increase/decrease the keypoint confidence threshold\n'
          '\td/f: increase/decrease the match filtering threshold\n'
          '\tk: toggle the visualization of keypoints\n'
          '\tq: quit')

    timer = AverageTimer()
    count = 0
    import copy
    while True:
        # print(count)
        # if np.any(video_receiver.latest_frame):
        #     # print(count)
        #     frame = video_receiver.latest_frame
        #     color_frame = copy.deepcopy(frame)
        #     frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
        #     frame = cv2.resize(frame, (640, 360))
        #     camera_matrix = video_receiver.K1
        #     pose_solver.set_cameraMatrix(camera_matrix)
        #     count += 1
        # else:
        #     continue

        frame, ret = vs.next_frame()
        color_frame = copy.deepcopy(frame)
        frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
        count += 1
        if not ret:
            print('Finished demo_superglue.py')
            break
        
        # intrinsic = [camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2]]
        # print(intrinsic)
        
        # color_frame = copy.deepcopy(cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR))
        # depth_solve.fetch_image(color_frame, intrinsic)
        depth_solve.fetch_image(color_frame)
        depth_solve.fetch_image(color_frame)
        pose_solver.get_depth(depth_solve.depth)
        
        timer.update('data')
        stem0, stem1 = last_image_id, count

        frame_tensor = frame2tensor(frame, device)
        pred = matching({**last_data, 'image1': frame_tensor})
        kpts0 = last_data['keypoints0'][0].cpu().numpy()
        kpts1 = pred['keypoints1'][0].cpu().numpy()
        matches = pred['matches0'][0].cpu().numpy()
        confidence = pred['matching_scores0'][0].cpu().numpy()
        # timer.update('forward')

        valid = matches > -1
        mkpts0 = kpts0[valid]
        mkpts1 = kpts1[matches[valid]]
        conf = confidence[valid]
        if leng < 4:
            leng = pose_solver.fetch_corr(mkpts0, mkpts1, confidence[valid])
            print(leng)
            if leng > 3:
                # pose_solver.set_campose(video_receiver.trans, video_receiver.rot)
                pose_solver.solve()
                if pose_solver.std < 0.005:
                    frame = pose_solver.draw_bbox(frame)
                # frame = pose_solver.draw_bbox(frame)
                leng = 0
            # pose_solver.solve()

        color = cm.jet(confidence[valid])
        text = [
            'SuperGlue',
            'Keypoints: {}:{}'.format(len(kpts0), len(kpts1)),
            'Matches: {}'.format(len(mkpts0))
        ]
        k_thresh = matching.superpoint.config['keypoint_threshold']
        m_thresh = matching.superglue.config['match_threshold']
        small_text = [
            'Keypoint Threshold: {:.4f}'.format(k_thresh),
            'Match Threshold: {:.2f}'.format(m_thresh),
            'Image Pair: {:06}:{:06}'.format(stem0, stem1),
        ]
        out = make_matching_plot_fast(
            last_frame, frame, kpts0, kpts1, mkpts0, mkpts1, color, text,
            path=None, show_keypoints=opt.show_keypoints, small_text=small_text)
        if not opt.no_display:
            cv2.imshow('SuperGlue matches', out)
            key = chr(cv2.waitKey(1) & 0xFF)
            if key == 'q':
                vs.cleanup()
                # video_receiver.running = False
                print('Exiting (via q) demo_superglue.py')
                break
            elif key == 'n':  # set the current frame as anchor
                cv2.imwrite("init.png", frame)
                last_data = {k+'0': pred[k+'1'] for k in keys}
                last_data['image0'] = frame_tensor
                last_frame = frame
                last_image_id = (count - 1)
            elif key in ['e', 'r']:
                # Increase/decrease keypoint threshold by 10% each keypress.
                d = 0.1 * (-1 if key == 'e' else 1)
                matching.superpoint.config['keypoint_threshold'] = min(max(
                    0.0001, matching.superpoint.config['keypoint_threshold']*(1+d)), 1)
                print('\nChanged the keypoint threshold to {:.4f}'.format(
                    matching.superpoint.config['keypoint_threshold']))
            elif key in ['d', 'f']:
                # Increase/decrease match threshold by 0.05 each keypress.
                d = 0.05 * (-1 if key == 'd' else 1)
                matching.superglue.config['match_threshold'] = min(max(
                    0.05, matching.superglue.config['match_threshold']+d), .95)
                print('\nChanged the match threshold to {:.2f}'.format(
                    matching.superglue.config['match_threshold']))
            elif key == 'k':
                opt.show_keypoints = not opt.show_keypoints
            elif key == "p":
                # pose_solver.set_campose(video_receiver.trans, video_receiver.rot)
                pose_solver.solve()
                leng = 0

        timer.update('viz')
        # timer.print()

        if opt.output_dir is not None:
            #stem = 'matches_{:06}_{:06}'.format(last_image_id, vs.i-1)
            stem = 'matches_{:06}_{:06}'.format(stem0, stem1)
            out_file = str(Path(opt.output_dir, stem + '.png'))
            print('\nWriting image to {}'.format(out_file))
            cv2.imwrite(out_file, out)

    cv2.destroyAllWindows()
    # video_receiver.running = False
    depth_solve.running = False
    vs.cleanup()

