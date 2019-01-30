#include <iostream>  
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>

using namespace std;
using namespace cv;

int board_w = 6;
int board_h = 4;

void func_adjust_view(VideoCapture cap,Mat mapx,Mat mapy);
void func_bird_view(VideoCapture cap,Mat mapx,Mat mapy,Mat H_bird);

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
		cornerSubPix(grayImage, corners2, Size(11, 11), Size(-1, -1),  TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
	else cout<<"findChessboardCorners failed"<<endl;
	
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
	
    H.at<double>(2, 2) = 24.5;
    
	FileStorage fsout("bird_H.xml", FileStorage::WRITE);
	fsout << "H" << H;
	fsout.release();

	return H;
}

int main()
{
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		cout << "Open videocapture failed!" << endl;
		return 0;
	}
	//cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	//cap.set(CV_CAP_PROP_FPS, 30);
	//cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	cout << "test mapx and mapy..." << endl;

	FileStorage fs("../xml/camera2_fish_480.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs, H;
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
	

	remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);
	Mat H_bird = Calculation_H(undistort_img);

	cout << "test is over. start video..." << endl;


	//func_adjust_view(cap, mapx, mapy);
	func_bird_view(cap,mapx,mapy,H_bird);
	
	
	waitKey(0);
	return 0;
}


void func_adjust_view(VideoCapture cap,Mat mapx,Mat mapy)
{
    Mat distort_img, undistort_img;
    cap >> distort_img;

    while(1){
    cap >> distort_img;
   	if (distort_img.empty()) {
		cout << "video ending......" << endl;			
		return;
	}

	imshow("distort_img video", distort_img);

	remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);    //undistorting 
	
line(undistort_img,Point(0, distort_img.rows/3),Point(distort_img.cols,distort_img.rows/3),Scalar(0,0,255),2,CV_AA);// the first level line
line(undistort_img,Point(0, 2*distort_img.rows/3),Point(distort_img.cols, 2*distort_img.rows/3),Scalar(0,0,255),2,CV_AA);// the second level line
line(undistort_img,Point(distort_img.cols/2, 0),Point(distort_img.cols/2, distort_img.rows),Scalar(0,0,255),2,CV_AA);// the mid verticlal line

	imshow("calibrated video", undistort_img);
	waitKey(1);
	}
}

void func_bird_view(VideoCapture cap,Mat mapx,Mat mapy,Mat H_bird)
{
	Mat distort_img, undistort_img;
	while(1){
    cap >> distort_img;
   	if (distort_img.empty()) {
		cout << "video ending......" << endl;			
		return;
	}

	imshow("distort_img video", distort_img);
	
	remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);    //undistorting 
	imshow("undistort_img video", undistort_img);
	
    warpPerspective(undistort_img, undistort_img, H_bird, distort_img.size(), WARP_INVERSE_MAP + INTER_LINEAR);
    
	imshow("bird video", undistort_img);
	
	waitKey(1); 
	}   
}

