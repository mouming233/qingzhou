#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>	

using namespace cv;
using namespace std;

//函数
float cacu_distance(Point2f target);
float cacu_distance(Point2f a, Point2f b);
vector<Point2f> arrange(vector<Point2f>const& targets);
float angle(Point a, float r);
float time(float speed, float angle, float r);
Point2f circle_center(Point2f a, Point2f b);
string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
        to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
        "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
        to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

//一个像素点对应物理世界中的长度的转换系数(像素到米)
float one = 0.00078;

//车头在透视变换后图片中的坐标
Point2f car(320,485);

int main(int argc, char** argv)
{
    //视频参数
    int capture_width = 640;
    int capture_height = 480;
    int display_width = 640;
    int display_height = 480;
    int framerate = 60;
    int flip_method = 0;
    //创建管道
    string pipeline = gstreamer_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method);
    //管道与视频流绑定
    VideoCapture cap(pipeline, CAP_GSTREAMER); 
    if(!cap.isOpened())
    {
        std::cout<<"打开摄像头失败."<<std::endl;
        return 0;
    }

	// 初始化ROS节点
	ros::init(argc, argv, "s_road");
	ros::NodeHandle nh;

    //cmd发布者
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/qingzhou/s_road/cmd_vel", 1);
    //图片发布者
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("s_image", 1);
//----------------------------------------------------------------------------------------图像处理--------------------------------------------------------------------------------------------------------
	Mat frame;
	while(1){
		cap >> frame;

		//获取当前时刻
		float t1 = getTickCount();

		//定义变量
		Mat imgroi, switch_matrix, imgswi, imghsv, imgbin, imgdil1, imgero1, imgcan;
		Mat kernel = getStructuringElement(MORPH_CROSS, Size(5, 5));
		Mat imgero2, imgdil2;

		//提取绿色参数
		int hmin = 35, smin = 29, vmin = 0;
		int hmax = 85, smax = 255, vmax = 255;

		//掩膜变量
		vector<vector<Point>> contours;
		vector<Vec4i>hierarchy;

		//提取ROI区域并进行透视变换
		Mat mask = Mat::zeros(frame.size(), CV_8UC3);
		vector<vector<Point>> contour;
		vector<Point> pts;
		pts.push_back(Point(191, 335));
		pts.push_back(Point(0, 462));
		pts.push_back(Point(640, 480));
		pts.push_back(Point(429, 342));
		contour.push_back(pts);
		drawContours(mask, contour, 0, Scalar::all(255), -1);
		frame.copyTo(imgroi, mask);
		Point2f past[4] = { {191, 335},{429, 342},{0, 462},{640,480} };
		Point2f now[4] = { {0,0},{640,0},{0,480},{640,480} };
		switch_matrix = getPerspectiveTransform(past, now);
		warpPerspective(imgroi, imgswi, switch_matrix, Size(640, 480));

		//转换到hsv空间并通过颜色筛选色块
		cvtColor(imgswi, imghsv, COLOR_BGR2HSV);
		inRange(imghsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), imgbin);

		//进行一次开运算消除边缘连接
		erode(imgbin, imgero1, kernel);
		dilate(imgero1, imgdil1, kernel);

		erode(imgdil1, imgero2, kernel);
		dilate(imgero2, imgdil2, kernel);

		dilate(imgdil2, imgdil2, kernel);

		//进行边缘检测
		Canny(imgdil2, imgcan, 25, 75);
		dilate(imgcan, imgcan, kernel);

		//获取轮廓
		findContours(imgcan, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		//找出boundingrect并绘制它、标明文字
		vector<vector<Point>> conpoly(contours.size());
		vector<Rect> boundrect(contours.size());

		//轮廓的面积、周长信息
		float peri;
		int area;

		//储存轮廓点
		vector<Point2f> points;

		for (int i = 0; i < contours.size(); ++i)
		{
			peri = arcLength(contours[i], true);
			area = contourArea(contours[i]);
			//面积筛选
			if (area > 1000 && area < 5500) {
				approxPolyDP(contours[i], conpoly[i], 0.02 * peri, true);
				boundrect[i] = boundingRect(conpoly[i]);
				if (boundrect[i].height < 150)
				{
					points.push_back(Point2f(boundrect[i].x + boundrect[i].width / 2, boundrect[i].y + boundrect[i].height / 2));
					rectangle(imgswi, boundrect[i], Scalar(0, 255, 0), 2);
				}
			}
		}
		//连通性判断（只保留中心车道线）同时也是排序函数
		vector<Point2f> uf_points;
		if (points.size() > 0)
			uf_points = arrange(points);
//----------------------------------------------------------------------------------------图像处理--------------------------------------------------------------------------------------------------------

//-------------处理好后只有uf_points（存放检测到的车道块的信息）和imgswi----------

        //标记中心车道块顺序
		for (int i = 0; i < uf_points.size(); ++i)
		{
			string f = to_string(i);
			putText(imgswi, f, Point(uf_points[i].x, uf_points[i].y), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3);
		}

        //---------所需的物理量---------
        //所需运动的时间
        float time = 0;
        //角速度
        float angular = 0;
        //
        float r = 0;
		//行驶速度
		float speed = 0.1;
        //只检测到一个块的情况
		if (uf_points.size() ==  1)
		{
                line(imgswi,uf_points.back(),Point(car.x,car.y),Scalar(0, 0, 255));
                //以10ms向唯一点转向
                time = 0.01;
                angular = (uf_points.back().x - car.x) / abs(uf_points.back().x - car.x) * 0.1;
		}
        //检测到两个点及以上
		else if (uf_points.size() >= 2)
		{
            //计算拟合园的中心点坐标
			Point2f center = circle_center(uf_points.back(), *(uf_points.end() - 2));
            //计算半径
			r = cacu_distance(center);
			circle(imgswi, center, r, Scalar(0, 0, 255), 3);
            //计算运动时间和角速度
			time = asin(cacu_distance(uf_points.back()) / 2 / r) * r * 2 / speed * one;
			angular = speed / r;
		}
        //没检测到点则停下
        else
        {
            speed = 0;
            angular = 0;
        }

        //获取当前时刻并计算帧率
		float t2 = getTickCount();
		float spendTime = (t2 - t1) / (getTickFrequency());
		float FPS = 1 / spendTime;
		string fps = to_string(int(FPS));

		//------------------------------发布线速度与角速度
        geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = speed;
		cmd_vel.angular.z = angular;
		cmd_vel_pub.publish(cmd_vel);
        
        //计算运动所需的时间
        float duration = time - spendTime;

        //打印部分信息
        printf("r: %f  time: %f  spendTime: %f angular: %f  speed: %f duration: %f \n", r * one, time, spendTime,angular,speed,duration);

        //对图像做最后的处理
		putText(imgswi, "FPS:", Point(20, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);
		putText(imgswi, fps, Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);

        //------------------------------发布图像
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgswi).toImageMsg();
        image_pub.publish(msg);

        //倒头睡觉
        sleep(duration);
	}
    //释放摄像头
    cap.release();
	return 0;
}

//距离计算函数
float cacu_distance(Point2f target)
{
	return pow(pow(target.x - car.x, 2) + pow(target.y - car.y, 2), 0.5);
}

float cacu_distance(Point2f a, Point2f b)
{
	return pow(pow(a.x - b.x, 2) + pow(a.y - b.y, 2), 0.5);
}

//点排序函数
vector<Point2f> arrange(vector<Point2f>const& targets)
{
	printf("%d\n", targets.size());
	//假设检测到的点不超过十个
	float distance_list[10] = { 0 };
	int n = targets.size();
	//计算所有点到车的距离
	if (n)
	{
		for (int i = 0; i < n; ++i)
		{
			distance_list[i] = cacu_distance(targets[i]);
		}
	}
	//首先确定离车最近的点
	float min = distance_list[0];
	float y_min = targets[0].y;
	int count[10] = { 0 };
	//记录最近点的标号
	int min_cnt = 0;
	//记录联通点的个数
	int add = 1;
	if (n >= 2)
		for (int i = 1; i < n; ++i)
			if (min > distance_list[i] && y_min < targets[i].y)
			{
				min = distance_list[i];
				y_min = targets[i].y;
				min_cnt = i;
			}
	//对联通顺序进行记录
	count[min_cnt] = 1;
	//进行连通性判断
	for (int i = 0; i < n; ++i)
	{
		if (count[i] == 0)
		{
			//判定连通性时为了保证从下到上储存，要保证后来的点y更小
			if (cacu_distance(targets[i], targets[min_cnt]) < 230 && cacu_distance(targets[i], targets[min_cnt]) > 80 && targets[i].y < targets[min_cnt].y)
			{
				count[i] = ++add;
				min_cnt = i;
				//一旦找到了相邻点,再从头遍历
				i = -1;
			}
		}
	}
	//根据记录连通性顺序的count数组构造一个按顺序储存的点向量
	vector<Point2f> uf_points;
	for (int i = 1; i <= add; i++)
	{
		for (int j = 0; j < n; ++j)
			if (count[j] == i)
				uf_points.push_back(targets[j]);
	}
	return uf_points;
}

float angle(Point a, float r)
{
	if (a.x > car.x)
		return asin(abs(a.y - car.y) / abs(r));
	else
		return -asin(abs(a.y - car.y) / abs(r));
}

float time(float speed, float angle, float r)
{
	return abs(angle) * abs(r) / speed;
}

Point2f circle_center(Point2f a, Point2f b)
{
	float x2 = a.x;
	float x3 = b.x;
	float y2 = a.y;
	float y3 = b.y;
	Point2f result;
	result.x = ((x2 - car.x) / (car.y - y2) * (car.x + x2) / 2 - (x3 - x2) / (y2 - y3) * (x2 + x3) / 2 + (y3 - car.y) / 2) / ((x2 - car.x) / (car.y - y2) - (x3 - x2) / (y2 - y3));
	result.y = (x3 - x2) / (y2 - y3) * (result.x - (x2 + x3) / 2) + (y2 + y3) / 2;
	return result;
}
