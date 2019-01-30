
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;
using namespace cv;

int main()
{
	/**************************************/
	cout << "test mapx and mapy..." << endl;

	FileStorage fs("test.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs, H;
	fs["intrinsic_matrix"] >> intrinsic_matrix;
	fs["distortion_coeffs"] >> distortion_coeffs;
	fs["H"] >> H;
	fs.release();

	Mat distort_img = imread("1.jpg");
	Mat mapx = Mat(distort_img.size(), CV_32FC1);
	Mat mapy = Mat(distort_img.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);


	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, distort_img.size(), CV_32FC1, mapx, mapy);   //measure the mapx and mapy by the "test.xml" and image

	H.at<double>(2, 2) = 45;  //filed size. the num bigger, the view smaller.


	/**************************************/
	cout << "test is over. start video..." << endl;

	VideoCapture capture("2.WMV");

	if (!capture.isOpened())
	{
		cerr << "can not open video\n" << endl;
		return -1;
	}

	Mat distortImg, birdImg, undistortImg;
	while (1)
	{
		capture >> distortImg;
		if (distortImg.empty())
		{
			cout << "video ending......" << endl;
			break;
		}
		double start = getTickCount();     //measure the runtime of codes
		remap(distortImg, undistortImg, mapx, mapy, INTER_LINEAR);    //undistorting 
		cout << (getTickCount() - start) / getTickFrequency() << endl;
		warpPerspective(undistortImg, birdImg, H, undistortImg.size(), WARP_INVERSE_MAP + INTER_LINEAR);   //being birdseye


		imshow("calibrated video", birdImg);
		waitKey(1);
	}
	waitKey(0);
	return 0;
}
