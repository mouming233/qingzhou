# -*- coding:utf-8 _*-
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2, time, socket, json
import multiprocessing as mp

# cameraMatrix = np.array([[306.5654,0,314.5789],[0,306.0117,269.9443],[0,0,1]])
# distCoeffs = np.array([-0.2687,0.0518,0.0023,0.0031])
# newcameraMatrix,useless = cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(640,480),0,(640,480))
# map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, newcameraMatrix, (640, 480), 5)

cameraMatrix = np.array([[304.666,0,322.807],[0,302.009,268.436],[0,0,1]])
distCoeffs = np.array([-0.2507,0.0425,0.0023,0.0031])
newcameraMatrix,useless = cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(640,480),0,(640,480))
map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, newcameraMatrix, (640, 480), 5)

def gstreamer_pipeline(
    capture_width=3264,
    capture_height=2464,
    display_width=640,
    display_height=480,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


#从摄像头读取数据
cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
out = cv2.VideoWriter("out1.avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 60, (640,480))
if not cam.isOpened():
    print("Unable to open camera")
else:
	print('Open camera success!')
	while True:
		ret, Img = cam.read()
		result = cv2.remap(Img, map1, map2, cv2.INTER_LINEAR)
		out.write(result)
cam.release()