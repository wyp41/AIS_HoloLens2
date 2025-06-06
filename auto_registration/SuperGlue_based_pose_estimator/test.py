import numpy as np
import cv2

img1=cv2.imread('init.png')
img1 = cv2.resize(img1, (640, 360))

#像素点坐标初定义
pro_x = []
pro_y = []

#定义鼠标点击事件并将点击坐标输入数组
def mouse_img_cod(event, cod_x, cod_y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (cod_x,cod_y)
        cv2.circle(img1, (cod_x,cod_y), 1, (255, 0, 0), thickness = -1)
        cv2.putText(img1, xy, (cod_x,cod_y), cv2.FONT_HERSHEY_PLAIN,1.0, (0,0,0), thickness = 1) #将坐标值放在图片内
        cv2.imshow("image", img1)
        pro_x.append(cod_x)
        pro_y.append(cod_y)
        
cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE) #创建一个名为image的窗口
cv2.setMouseCallback("image", mouse_img_cod) #鼠标事件回调
cv2.imshow('image',img1) #显示图片
cv2.waitKey(0) #按下任意键退出窗口
cv2.destroyAllWindows()

print(pro_x,pro_y) #打印坐标值 

# import cv2
# import numpy as np

# cap = cv2.VideoCapture(0)
# n = 0
# while(1):
#     # get a frame
#     ret, frame = cap.read()
#     frame = cv2.resize(frame, (640, 480))
#     # show a frame
#     n += 1
#     if n >= 100:
#         cv2.imwrite("init.png", frame)
#         print("write!")
#         n = -10000
#     cv2.imshow("capture", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         # cv2.imwrite("init.png", frame)
#         break
# cap.release()
# cv2.destroyAllWindows() 

# import numpy as np
# import cv2 as cv
 
# img = cv.imread('red.png')
# img = cv.resize(img, (640, 640))

# point_size = 5
# point_color = (255, 255, 255) # BGR
# thickness = -1 # 可以为 0 、4、8

# # 要画的点的坐标

# points_list = [(320, 320), (320, 384), (320, 256), (64, 320), (176, 448)]

# for point in points_list:
# 	cv.circle(img, point, point_size, point_color, thickness)

# cv.namedWindow("image")
# cv.imshow('image', img)
# cv.imwrite('m.png', img)
# cv.waitKey (10000) # 显示 10000 ms 即 10s 后消失
# cv.destroyAllWindows()

# rot = np.float32([[1,0,0],[0,1,0],[0,0,1]])
# trans = np.float32([0,0,1])
# cameraMatrix = np.float32([ 6.5664077162555463e+02, 0., 3.2841613945979151e+02, 0.,
#        6.5705173539981251e+02, 2.2792930298550786e+02, 0., 0., 1. ]).reshape((3,3))
# l = 0.2
# h = 0.2
# w = 0.2
# x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
# y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
# z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
# corners_3d = np.matmul(rot, np.vstack([x_corners, y_corners, z_corners]))
# print(corners_3d)
# corners_3d[0,:] = corners_3d[0,:] + trans[0]
# corners_3d[1,:] = corners_3d[1,:] + trans[1]
# corners_3d[2,:] = corners_3d[2,:] + trans[2]
# print(corners_3d)
# corners_2d = np.matmul(cameraMatrix, corners_3d).T
# print(corners_2d)
# corners_2d = corners_2d[:,0:2]/corners_2d[:,2:3]
# corners_2d = corners_2d[:,0:2].astype(np.int32)
# print(corners_2d)

# img1=cv2.imread('init.png')
# img_np = np.asarray(img1)
# color = (0, 0, 0) # BGR

# for k in range(4):
#     i, j = k, (k+1) % 4
#     cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 1)
#     i, j = k + 4, (k + 1) % 4 + 4
#     cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 1)
#     i, j = k, k + 4
#     cv2.line(img_np, (corners_2d[i, 0], corners_2d[i, 1]), (corners_2d[j, 0], corners_2d[j, 1]), color, 1)

# cv2.namedWindow("image")
# cv2.imshow('image', img1)
# cv2.waitKey (5000) # 显示 10000 ms 即 10s 后消失
# cv2.destroyAllWindows()


