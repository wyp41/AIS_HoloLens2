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

# HOST = '192.168.122.157'
# HOST = '192.168.0.105'
HOST = '127.0.0.1'
# HOST = '192.168.0.113'


HundredsOfNsToMilliseconds = 1e-4
MillisecondsToSeconds = 1e-3

torch.set_grad_enabled(False)

class pnp():
    def __init__(self):

        self.cmd = "get"
        self.key = "get"
        self.calibPos = [0, 0, 0, 0, 1, 0, 0]
        # self.calibPos = [0, 0, 0, -0.707, 0, 0, 0.707]
        self.camPos = [0, 0, 0, 0, 0, 0, 1.0]
        self.corPos = [0, 0, 0, 0, 0, 0, 1.0]
        self.new_cmd = False
        self.calibed = False
        self.pv_trans = []
        self.pv_rot = []
        self.err = 10000

        # self.origin = [[0.0, 0, 0], [0, 0.02, 0], [0, -0.02, 0], [0.08, 0, 0], [0.045, 0.04, 0]]
        # self.origin = [[0.0, 0, 0], [0, 0.01, 0], [0, -0.01, 0], [0.02, 0.025, 0], [0.03, 0, 0]]
        self.origin = [[0.0, 0, 0],  [0, -0.02, 0], [0.08, 0, 0], [0.045, 0.04, 0]]

        self.marker = []

        # self.imgs = [[326, 152], [328, 89], [326, 204], [577, 156], [466, 27]]
        # self.imgs = [[280, 223], [280, 192], [281, 257], [429, 221], [359, 158]]
        # self.imgs = [[251,246],[252,203],[251,290],[328,144],[389,240]]
        # self.imgs =[[218,212], [216,144], [219,268], [315,76], [395,196]]
        self.imgs = [[330,132],  [328,183], [525,142], [444, 28]]
        # self.imgs = [[330, 208], [334, 158], [327, 254], [560, 242], [470, 113]]
        # self.imgs = [[319, 240], [320, 257], [320, 224], [250, 240], [280, 273]]
        self.new_imgs = []

        self.trans = [0, 0, 0]
        self.rot = [[1,0,0], 
                    [0,1,0], 
                    [0,0,1]]

        
        self.cameraMatrix = np.float32([ 6.5664077162555463e+02, 0., 3.2841613945979151e+02, 0.,
       6.5705173539981251e+02, 2.2792930298550786e+02, 0., 0., 1. ]).reshape((3,3))
        self.distCoeffs = np.float32([0, 0, 0, 0, 0])
        
        t1 = threading.Thread(target=self.calib_comm)
        t2 = threading.Thread(target=self.keyboard_read)
        t3 = threading.Thread(target=self.cam_comm)
        # t1.start()
        t2.start()
        # t3.start()
    
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
        # print(error)

        if error < self.err and len(self.new_imgs) > 3:
            self.err = error
            if error > 2:
                self.key = "c"
                self.new_cmd = True
            # print("calibed")
            n_r = R.from_matrix(self.rot)
            n_r = list(n_r.as_quat())
            print(error)
            print(self.trans, n_r)

        return img_np
    
    def mul(self, trans1, rot1, trans2, rot2):
        trans2 = np.float32(trans2)
        n_r = np.matmul(rot1, rot2)
        n_t = np.matmul(rot1, trans2).reshape((1,3)) + np.float32(trans1)
        n_t = list(n_t[0])
        return n_t, n_r

    def comp_obj(self):
        o_trans = self.calibPos[:3]
        o_rot = self.calibPos[3:]
        # print(self.camPos)
        cam_trans = self.corPos[:3]
        cam_rot = self.corPos[3:]
        # cam_trans = self.camPos[:3]
        # cam_rot = self.camPos[3:]
        # cam_trans[2] = -cam_trans[2]
        # cam_rot[0] = -cam_rot[0]
        # cam_rot[1] = -cam_rot[1]
        rec_trans = [0, -0.085, 0.072]
        # rec_trans = [-0.02, -0.065, 0.1]
        rec_rot = [1, 0, 0, 0]

        # rec_trans = [-0.04, 0.006, -0.08]
        # rec_rot = [-0.996, -0.035, -0.087, -0.003]


        r = R.from_quat(o_rot)
        r = np.float32(r.as_matrix())
        cr = R.from_quat(cam_rot)
        cr = np.float32(cr.as_matrix())
        rr = R.from_quat(rec_rot)
        rr = np.float32(rr.as_matrix())

        target_trans = self.trans
        tr = self.rot

        ct = np.float32(cam_trans)

        # n_t, n_r = self.mul(ct, cr, rec_trans, rr)

        # mt, mr = self.mul(self.pv_trans, np.float32((R.from_matrix(self.pv_rot)).as_matrix()), [0,0,0], rr)

        # print("cam_trans:", n_t, (R.from_matrix(n_r)).as_quat(), "pv_trans:", mt, (R.from_matrix(mr)).as_quat())

        # target_trans = [0, 0, 0.2]
        # tr = rr

        # ct = np.float32(self.pv_trans) + np.float32(cam_trans)
        # ct = ct / 2.0
        # print(ct, self.pv_trans)

        n_t, n_r = self.mul(ct, cr, rec_trans, rr)
        n_t, n_r = self.mul(n_t, n_r, target_trans, tr)
        n_t, n_r = self.mul(n_t, n_r, o_trans, r)
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


if __name__ == "__main__":
    pose_solver = pnp()
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
    print(opt)

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


    video_receiver = VideoReceiverThread(HOST)
    video_receiver.start_socket()

    video_receiver.start_listen()

    # vs = VideoStreamer(opt.input, opt.resize, opt.skip,
    #                    opt.image_glob, opt.max_length)
    # frame, ret = vs.next_frame()
    # assert ret, 'Error when reading the first frame (try different --input?)'

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
    while True:
        # print(count)
        if np.any(video_receiver.latest_frame):
            frame = video_receiver.latest_frame
            frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
            frame = cv2.resize(frame, (640, 360))
            camera_matrix = video_receiver.K1
            pose_solver.set_cameraMatrix(camera_matrix)
            count += 1
        else:
            # print('Finished demo_superglue.py')
            # break
            continue
        

        # frame, ret = vs.next_frame()
        # if not ret:
        #     print('Finished demo_superglue.py')
        #     break
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
            if leng > 3:
                # pose_solver.set_campose(video_receiver.trans, video_receiver.rot)
                pose_solver.solve()
                frame = pose_solver.draw_bbox(frame)
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
                # vs.cleanup()
                video_receiver.running = False
                print('Exiting (via q) demo_superglue.py')
                break
            elif key == 'n':  # set the current frame as anchor
                # cv2.imwrite("tissue.png", frame)
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
        timer.print()

        if opt.output_dir is not None:
            #stem = 'matches_{:06}_{:06}'.format(last_image_id, vs.i-1)
            stem = 'matches_{:06}_{:06}'.format(stem0, stem1)
            out_file = str(Path(opt.output_dir, stem + '.png'))
            print('\nWriting image to {}'.format(out_file))
            cv2.imwrite(out_file, out)

    cv2.destroyAllWindows()
    video_receiver.running = False
    # vs.cleanup()
