# -*- coding:utf-8 _*-
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2, time, socket, json
import multiprocessing as mp

cameraMatrix = np.array([[306.5654,0,0],[0,306.0117,0],[314.5789,269.9443,1]])
distCoeffs = np.array([-0.2687,0.0518,0.0023,0.0031])
newcameraMatrix,useless = cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(640,480),0,(640,480))
map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, newcameraMatrix, (640, 480), cv2.CV_32FC1)

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


def ImgRead(ImgQueue):
    # %% 从摄像头读取数据
    # cam = cv2.VideoCapture(0)
	cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
	print(cam.get(6))
	if not cam.isOpened():
		print("Unable to open camera")
	else:
		print('Open camera success!')
		while True:
			ret, Img = cam.read()
			if not ret:
				break
			while not ImgQueue.empty():
				ImgQueue.get()
		    ImgQueue.put(Img)
            Img = cv2.remap(Img, map1, map2, cv2.INTER_LINEAR)
            Img = Img.astype(np.uint8)
            cv2.imshow('ImgRead', Img)
			key = cv2.waitKey(5)
			if key == 27:
				break
		cam.release()

def vision():
	Frame = 0
	ImgQueue = mp.Queue()  # 先进先出队列，实现不同进程数据交互
	Mps = []
	Mps.append(mp.Process(target=ImgRead, args=(ImgQueue,)))
	[Mp.start() for Mp in Mps]
	# Mps[0].join()
	while ImgQueue.empty():
		pass
	while True:
		Key = input('Press s or S to save image:')
		if Key == 's' or Key == 'S':
			Img = ImgQueue.get()
			cv2.imwrite('%04d.jpg' % Frame, Img)
			print('Save image %04d.jpg success!' % Frame)
			Frame = Frame + 1
		elif Key == 'Q' or Key == 'q':
			break
	[Mp.terminate() for Mp in Mps]

if __name__ == '__main__':
	vision()

