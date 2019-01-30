#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
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
	fs  ["intrinsic_matrix"] >> intrinsic_matrix;
	fs  ["distortion_coeffs"] >> distortion_coeffs;
	fs["H"] >> H;
	fs.release();

	Mat distort_img=imread("1.jpg");
	Mat mapx = Mat(distort_img.size(), CV_32FC1);
	Mat mapy = Mat(distort_img.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	

	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, distort_img.size(), CV_32FC1, mapx, mapy);   //measure the mapx and mapy by the "test.xml" and image
	
	H.at<double>(2, 2) = 55;  //filed size. the num bigger, the view smaller.
	H.at<double>(0, 2) += 2000;     //x  +l   -r
        H.at<double>(1, 2) += 800;    //y  +up  -down

	gpu::GpuMat mapx_g,mapy_g,H_g;
	mapx_g.upload(mapx);
	mapy_g.upload(mapy);

	/**************************************/
	cout<<"test GPU..."<<endl;
	int num_devices = gpu::getCudaEnabledDeviceCount();
	cout<<num_devices<<endl;






	/*************************************/
	cout<< "test is over. start video..."<<endl;
	
	VideoCapture capture(0);
	if (!capture.isOpened())
	{
		cerr << "can not open video\n" << endl;
		return -1;
	}

	Mat distortImg, birdImg, undistortImg;
	gpu::GpuMat distortImg_g, birdImg_g, undistortImg_g;
	double start,t_ave, t_sum=0,t_num=0;
	
	while (1)
	{
		capture >> distortImg;
		if (distortImg.empty())
		{
			cout << "video ending......" << endl;
			break;
		}
		
		start=getTickCount();     //measure the runtime of codes

		distortImg_g.upload(distortImg);
		gpu::remap(distortImg_g, undistortImg_g, mapx_g, mapy_g, INTER_LINEAR);  //undistorting  INTER_NEAREST  INTER_LINEAR
		gpu::warpPerspective(undistortImg_g, birdImg_g, H, undistortImg_g.size(), INTER_LINEAR + WARP_INVERSE_MAP);   //being birdseye
      		birdImg_g.download(birdImg);
		t_ave=(getTickCount()-start)/getTickFrequency();
		cout<<t_ave<<endl;
		

		t_sum += t_ave;
		t_num++;
		//cout<<t_sum<<endl;
		cout<<t_num<<endl;
		cout<<t_sum/t_num<<endl;

		imshow("calibrated video", birdImg);
		waitKey(30);
	}
	cout<<"the average time of a image processing:"<<t_sum/t_num<<endl;
	waitKey(0);
	return 0;
}
