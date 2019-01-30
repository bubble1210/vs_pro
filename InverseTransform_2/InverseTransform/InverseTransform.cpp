#include <iostream>  
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>

using namespace std;
using namespace cv;

int board_w = 6;
int board_h = 4;
double view = 23;	//adjust the view of bird

Rect select;
bool select_flag = false;
Point origin;
Mat bird_img;

Mat Calculation_H(Mat &src_img);
void func_adjust_view(VideoCapture cap, Mat mapx, Mat mapy);
void func_bird_view(VideoCapture cap, Mat mapx, Mat mapy);
void func_adjust_bird_view(VideoCapture cap, Mat mapx, Mat mapy);
void gamma(Mat &input, Mat &output);

int func_1();
int func_2();



void onMouse(int event, int x, int y, int, void*);



int main()
{
	func_1();
	//func_2();

	return 0;
}

//摄像头位置--鱼眼矫正--侧转俯
int func_1()
{
	VideoCapture cap(1);
	if (!cap.isOpened()) {
		cout << "Open videocapture failed!" << endl;
		return 0;
	}
	//cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap.set(CV_CAP_PROP_FPS, 30);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	FileStorage fs("./camera1_fish_720_2.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs;
	fs["intrinsic_matrix"] >> intrinsic_matrix;
	fs["distortion_coeffs"] >> distortion_coeffs;
	fs.release();

	Mat distort_img, undistort_img;
	for (int i = 0; i < 10; ++i) {
		cap >> distort_img;
	}

	Mat mapx = Mat(distort_img.size(), CV_32FC1);
	Mat mapy = Mat(distort_img.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);

	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, distort_img.size(), CV_32FC1, mapx, mapy);   //measure the mapx and mapy by the "test.xml" and image

	cout << "Test start" << endl;

	func_adjust_view(cap, mapx, mapy);

	//func_adjust_bird_view(cap, mapx, mapy);

	cout << "Test over" << endl;
	waitKey(0);
	return 0;

}

//融合双摄像头
int func_2()
{
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		cout << "Open videocapture failed!" << endl;
		return 0;
	}
	//cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap.set(CV_CAP_PROP_FPS, 30);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	VideoCapture cap1(1);
	if (!cap1.isOpened()) {
		cout << "Open videocapture failed!" << endl;
		return 0;
	}
	//cap1.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap1.set(CV_CAP_PROP_FPS, 30);
	cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);




	FileStorage fs("./camera1_fish_480_2.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs;
	fs["intrinsic_matrix"] >> intrinsic_matrix;
	fs["distortion_coeffs"] >> distortion_coeffs;
	fs.release();

	fs.open("./bird_H_1.xml", FileStorage::READ);
	Mat H;
	Rect scala;
	fs["H"] >> H;
	fs["Rect"] >> scala;
	fs.release();

	Mat distort_img, undistort_img;
	for (int i = 0; i < 10; ++i) {
		cap >> distort_img;
	}

	Mat mapx = Mat(distort_img.size(), CV_32FC1);
	Mat mapy = Mat(distort_img.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);

	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, distort_img.size(), CV_32FC1, mapx, mapy);





	fs.open("./camera2_fish_480_2.xml", FileStorage::READ);
	Mat intrinsic_matrix1, distortion_coeffs1;
	fs["intrinsic_matrix"] >> intrinsic_matrix1;
	fs["distortion_coeffs"] >> distortion_coeffs1;
	fs.release();

	fs.open("./bird_H_2.xml", FileStorage::READ);
	Mat H1;
	Rect scala1;
	fs["H"] >> H1;
	fs["Rect"] >> scala1;
	fs.release();

	Mat distort_img1, undistort_img1;
	for (int i = 0; i < 10; ++i) {
		cap >> distort_img1;
	}

	Mat mapx1 = Mat(distort_img1.size(), CV_32FC1);
	Mat mapy1 = Mat(distort_img1.size(), CV_32FC1);
	Mat R1 = Mat::eye(3, 3, CV_32F);

	initUndistortRectifyMap(intrinsic_matrix1, distortion_coeffs1, R1, intrinsic_matrix1, distort_img1.size(), CV_32FC1, mapx1, mapy1);
	

	namedWindow("mosaic");
	setMouseCallback("mosaic", onMouse, 0);

	Mat mosaic_img = Mat(distort_img.rows, distort_img.cols*2.5, CV_8UC3, Scalar(0, 0, 0));
	int index_x = 50;
	Mat bird_img, bird_img1;

	char c;

	Mat temp_out,temp_out1;
	Mat t_bird_img,t_bird_img1;
	while (1){
		cap >> distort_img;
		cap1 >> distort_img1;
		if (distort_img.empty()) {
			cout << "video ending......" << endl;
			return 0;
		}
		if (distort_img1.empty()) {
			cout << "video ending......" << endl;
			return 0;
		}

		remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);    //undistorting
		warpPerspective(undistort_img, t_bird_img, H, undistort_img.size(), WARP_INVERSE_MAP + INTER_LINEAR);
		temp_out = t_bird_img(scala);
		resize(temp_out, undistort_img, undistort_img.size());
		
		remap(distort_img1, undistort_img1, mapx1, mapy1, INTER_LINEAR);    //undistorting
		warpPerspective(undistort_img1, t_bird_img1, H1, undistort_img1.size(), WARP_INVERSE_MAP + INTER_LINEAR);
		temp_out1 = t_bird_img1(scala1);
		resize(temp_out1, undistort_img1, undistort_img1.size());
		

		imshow("left", undistort_img);
		imshow("right", undistort_img1);

		undistort_img.copyTo(mosaic_img(Rect(0, 0, undistort_img.cols, undistort_img.rows) ) );
		undistort_img1(Rect(index_x, 0, undistort_img.cols - index_x, undistort_img.rows)).copyTo(mosaic_img(Rect(undistort_img.cols, 0, undistort_img.cols - index_x, undistort_img.rows)));

		blur(mosaic_img(Rect(undistort_img.cols - 20, 0, 40, undistort_img.rows)), mosaic_img(Rect(undistort_img.cols - 20, 0, 40, undistort_img.rows)), Size(5, 5));

		imshow("mosaic", mosaic_img);


		c = (char)waitKey(10);
		if (c == 32)
			index_x += 5;
		if (c == 13){
			index_x -= 5;
			if (index_x < 0)
				index_x = 0;
		}
		if (27 == c)//ENTER key
			break;

		cout << index_x << endl;
	}
	return 1;
}

Mat Calculation_H(Mat &src_img)
{
	int board_n = board_w*board_h;
	Size board_sz = Size(board_w, board_h);

	Mat srcImage = src_img.clone();

	Mat grayImage;
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);

	vector<Point2f> corners2;
	bool found = findChessboardCorners(srcImage, board_sz, corners2);
	if (found == 1)
	{
		cornerSubPix(grayImage, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
	else cout << "findChessboardCorners failed" << endl;

	Point2f imagePoints[4], objectPoints[4];
	objectPoints[0].x = 0;  objectPoints[0].y = 0;
	objectPoints[1].x = board_w - 1;    objectPoints[1].y = 0;
	objectPoints[2].x = 0;  objectPoints[2].y = board_h - 1;
	objectPoints[3].x = board_w - 1;  objectPoints[3].y = board_h - 1;
	imagePoints[0] = corners2.at(0);
	imagePoints[1] = corners2.at(board_w - 1);
	imagePoints[2] = corners2.at(board_w*(board_h - 1));
	imagePoints[3] = corners2.at(board_n - 1);
	circle(srcImage, imagePoints[0], 5, Scalar(0, 0, 255), -1);
	circle(srcImage, imagePoints[1], 5, Scalar(0, 255, 0), -1);
	circle(srcImage, imagePoints[2], 5, Scalar(255, 0, 0), -1);
	circle(srcImage, imagePoints[3], 5, Scalar(255, 255, 255), -1);

	Mat H;
	H = getPerspectiveTransform(objectPoints, imagePoints);

	H.at<double>(2, 2) = view;

	FileStorage fsout("bird_H.xml", FileStorage::WRITE);
	fsout << "H" << H;
	fsout.release();

	return H;
}



void func_adjust_view(VideoCapture cap, Mat mapx, Mat mapy)
{
	char c;

	Mat distort_img, undistort_img;
	Mat gamma_img;


	namedWindow("undistort video");
	setMouseCallback("undistort video", onMouse, 0);

	while (1){
		cap >> distort_img;
		if (distort_img.empty()) {
			cout << "video ending......" << endl;
			return;
		}

		imshow("distort_img video", distort_img);

		remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);    //undistorting 

		gamma(undistort_img, gamma_img);

		//if (undistort_img.cols==640)
			//resize(undistort_img, undistort_img, Size(undistort_img.cols * 2, undistort_img.rows * 2), 0, 0, INTER_LINEAR);

		line(gamma_img, Point(0, undistort_img.rows / 3), Point(undistort_img.cols, undistort_img.rows / 3), Scalar(0, 0, 255), 1, CV_AA);// the first level line
		line(gamma_img, Point(0, 2 * undistort_img.rows / 3), Point(undistort_img.cols, 2 * undistort_img.rows / 3), Scalar(0, 0, 255), 1, CV_AA);// the second level line
		line(gamma_img, Point(undistort_img.cols / 2, 0), Point(undistort_img.cols / 2, undistort_img.rows), Scalar(0, 0, 255), 1, CV_AA);// the mid verticlal line



		imshow("undistort video", gamma_img);
		c = (char)waitKey(10);
		if (13 == c)//ENTER key
			break;
	}

	namedWindow("bird_video");
	setMouseCallback("bird_video", onMouse, 0);

	//cap >> distort_img;
	//remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);
	Mat H_bird = Calculation_H(gamma_img);
	H_bird.at<double>(2, 2) = view;

	while (1){
		cap >> distort_img;
		if (distort_img.empty()) {
			cout << "video ending......" << endl;
			return;
		}

		imshow("distort_img video", distort_img);

		remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);    //undistorting 

		imshow("undistort video", undistort_img);

		warpPerspective(undistort_img, bird_img, H_bird, distort_img.size(), WARP_INVERSE_MAP + INTER_LINEAR);

		rectangle(bird_img, select, Scalar(0, 0, 255), 1, 8, 0);

		cout << select << endl;
		cout << "640/480= " << 640.0 / 480.0 << "	1280/720= "<<1280/720.0<<endl;
		cout << "the rect :" << (double)select.width / select.height << endl << endl;

		imshow("bird_video", bird_img);

		c = (char)waitKey(10);
		if (c == 32){	//space mean +
			view += 0.1;
			H_bird.at<double>(2, 2) = view;
		}
		if (c == 13){	//enter mean -
			view -= 0.1;
			H_bird.at<double>(2, 2) = view;
		}
		if (27 == c)//esc
			break;
	}

	FileStorage fsout("bird_H.xml", FileStorage::WRITE);
	fsout << "H" << H_bird;
	//fsout << "View_scale" << view;
	fsout << "Rect" << select;
	fsout.release();
}

void func_adjust_bird_view(VideoCapture cap, Mat mapx, Mat mapy)
{
	Mat distort_img, undistort_img;
	Mat H_bird;
	FileStorage fs("./bird_H.xml", FileStorage::READ);
	fs["H"] >> H_bird;

	char c;

	cout << H_bird << endl;
	//H_bird.at<double>(0, 0) = 1;
	//H_bird.at<double>(0, 1) = 0;
	//H_bird.at<double>(0, 2) = 300;

	//H_bird.at<double>(1, 0) = 0;
	//H_bird.at<double>(1, 1) = 1;
	//H_bird.at<double>(1, 2) = 500;

	//H_bird.at<double>(2, 0) = 0;
	//H_bird.at<double>(2, 1) = 0;
	//H_bird.at<double>(2, 2) = 23;

	cout << H_bird.at<double>(2, 0) << endl;

	while (1){
		cap >> distort_img;
		if (distort_img.empty()) {
			cout << "video ending......" << endl;
			return;
		}

		imshow("distort_img video", distort_img);

		remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);    //undistorting 
		imshow("undistort_img video", undistort_img);

		warpPerspective(undistort_img, bird_img, H_bird, distort_img.size(), WARP_INVERSE_MAP + INTER_LINEAR);

		rectangle(bird_img, select, Scalar(0, 0, 255), 1, 8, 0);
		cout << select << endl;
		cout << "640/480= " << 640.0 / 480.0 << endl;
		cout << "the rect :" << (double)select.width / select.height << endl << endl;

		imshow("bird_video", bird_img);

		c = (char)waitKey(10);
		if (27 == c)//ESC键
			break;
	}
}

void onMouse(int event, int x, int y, int, void*)
{
//Point origin;//不能在这个地方进行定义，因为这是基于消息响应的函数，执行完后origin就释放了，所以达不到效果。
	if (select_flag)
	{
		select.x = MIN(origin.x, x);//不一定要等鼠标弹起才计算矩形框，而应该在鼠标按下开始到弹起这段时间实时计算所选矩形框
		select.y = MIN(origin.y, y);
		select.width = abs(x - origin.x);//算矩形宽度和高度
		select.height = abs(y - origin.y);
		select &= Rect(0, 0, bird_img.cols, bird_img.rows);//保证所选矩形框在视频显示区域之内
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		select_flag = true;//鼠标按下的标志赋真值
		origin = Point(x, y);//保存下来单击是捕捉到的点
		select = Rect(x, y, 0, 0);//这里一定要初始化，宽和高为(0,0)是因为在opencv中Rect矩形框类内的点是包含左上角那个点的，但是不含右下角那个点
	}
	else if (event == CV_EVENT_LBUTTONUP){
		select_flag = false;
	}
}


void gamma(Mat &input, Mat &output)
{
	output = Mat::zeros(input.size(), CV_32FC3);
	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			output.at<cv::Vec3f>(i, j)[0] = (input.at<cv::Vec3b>(i, j)[0])*(input.at<cv::Vec3b>(i, j)[0]);
			output.at<cv::Vec3f>(i, j)[1] = (input.at<cv::Vec3b>(i, j)[1])*(input.at<cv::Vec3b>(i, j)[1]);
			output.at<cv::Vec3f>(i, j)[2] = (input.at<cv::Vec3b>(i, j)[2])*(input.at<cv::Vec3b>(i, j)[2]);
		}
	}
	//归一化到0~255    
	normalize(output, output, 0, 255, CV_MINMAX);
	//转换成8bit图像显示    
	convertScaleAbs(output, output);
}
