import cv2

# 抓取视频流
cap1 = cv2.VideoCapture("https://hololens:hololens@192.168.0.105/api/holographic/stream/live_high.mp4?holo=true&pv=true&mic=true&loopback=true")

ret1, frame1 = cap1.read()

cv2.namedWindow("hololens", 0)
cv2.resizeWindow("hololens", 640, 360)
# 使用，展示
while ret1:
    ret1, frame1 = cap1.read()
    cv2.imshow("hololens", frame1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite("init.png", frame1)
        break
# 退出时释放窗口和内存
cv2.destroyAllWindows()
cap1.release()
