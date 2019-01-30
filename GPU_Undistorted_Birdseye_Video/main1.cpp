
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpumat.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;
using namespace cv;

int main()
{
	gpu::GpuMat mapx_g,mapy_g,H_g;


	cout << "TestImage ..." << endl;
	Mat distort_img = imread("1.jpg", 1);
	
	Mat undistort_img, bird_img, H;

	
	FileStorage fs("test.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs;
	fs  ["intrinsic_matrix"] >> intrinsic_matrix;
	fs  ["distortion_coeffs"] >> distortion_coeffs;
	fs["H"] >> H;
        fs.release();

	H_g.upload(H);
	
	Mat mapx = Mat(distort_img.size(), CV_32FC1);
	Mat mapy = Mat(distort_img.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
		
	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, distort_img.size(), CV_32FC1, mapx, mapy);

	mapx_g.upload(mapx);
	mapy_g.upload(mapy);
	//remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);

	//imshow("undistorted image", undistort_img);

	H.at<double>(2, 2) = 45;
	//warpPerspective(undistort_img, bird_img, H, undistort_img.size(), WARP_INVERSE_MAP + INTER_LINEAR);
	//imshow("birdseye image", bird_img);

	//imwrite("test2.jpg", bird_img);



	VideoCapture capture("2.WMV");

	if (!capture.isOpened())
	{
		cerr << "can not open video\n" << endl;
		return -1;
	}

	Mat distortImg, birdImg, undistortImg;
	gpu::GpuMat distortImg_g, birdImg_g, undistortImg_g;

	while (1)
	{
		capture >> distortImg;

		if (distortImg.empty())
		{
			cout << "video ending......" << endl;
			break;
		}
		//H.at<double>(2, 2) = 25;//控制图像大小参数
		
		double start=getTickCount();
		Size aa(undistortImg.rows,undistortImg.cols);
		distortImg_g.upload(distortImg);
		gpu::remap(distortImg_g, undistortImg_g, mapx_g, mapy_g, INTER_LINEAR);
		gpu::warpPerspective(undistortImg_g, birdImg_g, H, aa, WARP_INVERSE_MAP + INTER_LINEAR);
		birdImg_g.download(birdImg);
		cout<<(getTickCount()-start)/getTickFrequency()<<endl;

		imshow("calibrated video", birdImg);
		waitKey(20);
	}
	waitKey(0);
	return 0;
}
