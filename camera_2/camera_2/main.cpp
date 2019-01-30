#include <iostream>
#include <opencv2\opencv.hpp>
#include <conio.h>
#include <math.h>
using namespace std;
using namespace cv;

void Key(char &ch, int &X, int &Y, double &h_33, double &angle,int &roixy)
{
	if (72 == ch) //向上移动
	{
		Y-=1;
		cout << "Y=" << Y << endl;
		if (Y == 0) Y = 0;
	}
	else if (80 == ch) //向下移动
	{
		Y+=1;
		cout << "Y=" << Y << endl;
		if (Y == 1023) Y = 1023;
	}
	else if (75 == ch) //向左移动
	{
		X-=1;
		cout << "X=" << X << endl;
		if (X == 0) X = 0;
	}
	else if (77 == ch) //向右移动
	{
		X+=1;
		cout << "X=" << X << endl;
		if (X == 1023) X = 1023;
	}
	else if ('+' == ch) //h33++
	{
		h_33+=0.1;
		cout << "h33=" << h_33 << endl;
	}
	else if ('-' == ch) //h33--
	{
		h_33-=0.1;
		cout << "h33=" << h_33 << endl;
		if (h_33 == 0) h_33 = 0;
	}
	else if ('o' == ch) //顺时针旋转
	{
		angle-=0.1;
		cout << "set_angle=" << angle << endl;
	}
	else if ('p' == ch) //逆时针旋转
	{
		angle+=0.1;
		cout << "set_angle=" << angle << endl;
	}
	else if ('w' == ch) //逆时针旋转
	{
		roixy++;
		cout << "roixy=" << roixy << endl;
	}
	else if ('e' == ch) //逆时针旋转
	{
		roixy--;
		cout << "roixy=" << roixy << endl;
	}

}
void process(Mat &H,Mat &src,Mat &dst,double &camera_h33, double &camera_angle,Mat mapx,Mat mapy,int &roi_x,int &roi_y)
{
	Mat birdImage, distort_img, dst_1;
	H.at<double>(2, 2) = camera_h33;//控制图像大小参数
	remap(src, distort_img, mapx, mapy, INTER_LINEAR);//鱼眼矫正
	//birdImage = distort_img.clone();
	warpPerspective(distort_img, birdImage, H, src.size(), WARP_INVERSE_MAP + INTER_LINEAR, 0, Scalar(0, 0, 0));//透视变换
	CvSize size = cvSize(roi_x, roi_y);//区域大小
	Mat imageROI = birdImage(Rect(0, 0, size.width, size.height));//设置源图像ROI
	imshow("ROI", imageROI);
	//旋转
	double angle = -camera_angle;
	cv::Point2f center(imageROI.cols / 2, imageROI.rows / 2);
	cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1);
	cv::Rect bbox = cv::RotatedRect(center, imageROI.size(), angle).boundingRect();

	rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
	rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;

	cv::warpAffine(imageROI, dst_1, rot, bbox.size(), 1, 0, Scalar(0, 0, 0));
	Mat dst_2(dst_1.rows, dst_1.cols, CV_8UC3, Scalar(0, 0, 0));
	Mat mask1, mask;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3)); //获取自定义核
	Mat imageROI1 = dst_2(Rect(0, 0, dst_1.cols, dst_1.rows));
	cvtColor(dst_1, mask, COLOR_RGB2GRAY); //灰度化获取掩膜
	erode(mask, mask1, element); //腐蚀
	dst_1.copyTo(imageROI1, mask1);
	dst = dst_2.clone();
}
void findOverlap(Mat &img1,Mat &img2,int &img2_x,int &img2_y)
{
	int coordinata_row;
	int coordinata_col;
	int coordinata_row_max = 0;
	int coordinata_col_max = 0;
	int coordinata_row_min = img1.rows;
	int coordinata_col_min = img1.cols;
	for (int i = img2_y; i < img2.rows; i++)
	{
		uchar* p1 = img1.ptr<uchar>(i); //获取第i行首地址
		uchar* p2 = img2.ptr<uchar>(i - img2_y);
		for (int j = img2_x; j < img2.cols; j++)
		{
			if (p1[j * 3] != 0 && p1[j * 3 + 1] != 0 && p1[j * 3 + 2] != 0 && p2[(j - img2_x) * 3] != 0 && p2[(j - img2_x) * 3 + 1] != 0 && p2[(j - img2_x) * 3 + 2] != 0)
			{
				if (i > coordinata_row_max) coordinata_row_max = i;
				else if (j > coordinata_col_max) coordinata_col_max = j;
				else if (i < coordinata_row_min) coordinata_row_min = i;
				else if (j < coordinata_col_min) coordinata_col_min = j;
			}
		}
	}
	//rectangle(img1, cvPoint(coordinata_col_min, coordinata_row_min), cvPoint(coordinata_col_max, coordinata_row_max), Scalar(0, 0, 255), 1, 8, 0);//画矩形 
	//cout << coordinata_row_min << " "<< coordinata_col_min << " " << coordinata_row_max << " " << coordinata_col_max << " " << endl;
}
//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat &img1, Mat& img2, Mat &big,int &img2_x,int &img2_y)
{
	int start_col = img2_x; //开始位置，即重叠区域的左边界  
	int start_row = img2_y;
	double processWidth = img2.cols - img2_x;//重叠区域的宽度  
	int rows = img2.rows;
	int cols = img2.cols; //注意，是列数*通道数
	double alpha = 1;//img1中像素的权重  
	int coordinata_row_max = 0;
	int coordinata_col_max = 0;
	int coordinata_row_min = img1.rows;
	int coordinata_col_min = img1.cols;
	for (int i = img2_y; i < img1.rows; i++)
	{
		uchar* p1 = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* p2 = img2.ptr<uchar>(i - img2_y);
		for (int j = img2_x; j < img1.cols; j++) //374
		{
			if (p1[j * 3] != 0 && p1[j * 3 + 1] != 0 && p1[j * 3 + 2] != 0 && p2[(j - img2_x) * 3] != 0 && p2[(j - img2_x) * 3 + 1] != 0 && p2[(j - img2_x) * 3 + 2] != 0)
			{
				if (i > coordinata_row_max) coordinata_row_max = i;
				else if (j > coordinata_col_max) coordinata_col_max = j;
				else if (i < coordinata_row_min) coordinata_row_min = i;
				else if (j < coordinata_col_min) coordinata_col_min = j;
			}

		}
	}
	processWidth = coordinata_col_max - coordinata_col_min;//重叠区域的宽度  
	//渐入渐出
	for (int i = img2_y; i < img1.rows; i++)
	{
		uchar* p1 = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* p2 = img2.ptr<uchar>(i - img2_y);
		uchar* p3 = big.ptr<uchar>(i);
		for (int j = img2_x; j < img1.cols; j++) //374
		{
			if (p1[j * 3] != 0 && p1[j * 3 + 1] != 0 && p1[j * 3 + 2] != 0 && p2[(j - img2_x) * 3] != 0 && p2[(j - img2_x) * 3 + 1] != 0 && p2[(j - img2_x) * 3 + 2] != 0)
			{
				//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比
				alpha = (processWidth - (j - coordinata_col_min)) / processWidth;
				p3[j * 3] = p1[j * 3] * alpha + p2[(j - img2_x) * 3] * (1 - alpha);
				p3[j * 3 + 1] = p1[j * 3 + 1] * alpha + p2[(j - img2_x) * 3 + 1] * (1 - alpha);
				p3[j * 3 + 2] = p1[j * 3 + 2] * alpha + p2[(j - img2_x) * 3 + 2] * (1 - alpha);
			}

		}
	}
	//rectangle(big, cvPoint(coordinata_col_min, coordinata_row_min), cvPoint(coordinata_col_max, coordinata_row_max), Scalar(0, 0, 255), 1, 8, 0);//画矩形 
}
//亮度调整
void rejust_light(Mat &img1, Mat &img2,int &img2_x,int &img2_y)
{
	//Mat new_image = img2.clone();
	Mat img1_gray;// (waittoberejustimg.size(), CV_8UC1, Scalar(0));
	cvtColor(img1, img1_gray, CV_RGB2GRAY);
	Mat img2_gray;// (destinateimg.size(), CV_8UC1, Scalar(0));
	cvtColor(img2, img2_gray, CV_BGR2GRAY);
	int img1_sum = 0, img2_sum = 0;
	double alpha = 0; //控制对比度
	int beta = 0;  //控制亮度
	for (int i = img2_y; i < img1.rows; i++)
	{
		uchar* p1 = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* p2 = img2.ptr<uchar>(i - img2_y);
		uchar* p3 = img1_gray.ptr<uchar>(i);
		uchar* p4 = img2_gray.ptr<uchar>(i - img2_y);
		for (int j = img2_x; j < img1.cols; j++) //374
		{
			if (p1[j * 3] != 0 && p1[j * 3 + 1] != 0 && p1[j * 3 + 2] != 0 && p2[(j - img2_x) * 3] != 0 && p2[(j - img2_x) * 3 + 1] != 0 && p2[(j - img2_x) * 3 + 2] != 0)
			{
				img1_sum += p3[j];
				img2_sum += p4[j - img2_x];
			}

		}
	}
	alpha = (double)img1_sum / img2_sum;
	if (alpha <= 1)
	{
		for (int x = 0; x < img2.rows; x++)
		{
			for (int y = 0; y < img2.cols; y++)
			{
				for (int c = 0; c < 3; c++)
				{
					img2.at<Vec3b>(x, y)[c] = saturate_cast<uchar>(alpha*(img2.at<Vec3b>(x, y)[c]) + beta);
				}
			}
		}
	}
	else
	{
		for (int x = 0; x < img1.rows; x++)
		{
			for (int y = 0; y < img1.cols; y++)
			{
				for (int c = 0; c < 3; c++)
				{
					img1.at<Vec3b>(x, y)[c] = saturate_cast<uchar>(alpha*(img1.at<Vec3b>(x, y)[c]) + beta);
				}
			}
		}	
	}

}
int main()
{
	char ch2;
	int model = 1;
	VideoCapture capture1(1);
	VideoCapture capture2(0);
	capture1.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture1.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	capture2.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture2.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	Mat frame1,frame2;
	Mat birdImage1, birdImage2;
	Mat dst1, dst2;
	Mat camera1_H, camera2_H;
	//写入鱼眼矫正参数
	FileStorage f1("camera1_fish.xml", FileStorage::READ);
	Mat camera1_intrinsic_matrix, camera1_distortion_coeffs;
	f1["intrinsic_matrix"] >> camera1_intrinsic_matrix;
	f1["distortion_coeffs"] >> camera1_distortion_coeffs;
	f1.release();
	FileStorage f2("camera2_fish.xml", FileStorage::READ);
	Mat camera2_intrinsic_matrix, camera2_distortion_coeffs;
	f2["intrinsic_matrix"] >> camera2_intrinsic_matrix;
	f2["distortion_coeffs"] >> camera2_distortion_coeffs;
	f2.release();
	//写入透视变换参数
	FileStorage f3("camera1_transform.xml", FileStorage::READ);
	f3["H"] >> camera1_H;
	f3.release();
	camera1_H.at<double>(2, 2) = 15;
	FileStorage f4("camera2_transform.xml", FileStorage::READ);
	f4["H"] >> camera2_H;
	f4.release();
	camera2_H.at<double>(2, 2) = 15;
	capture2 >> frame2;
	capture1 >> frame1;

	Size camera1__size = frame1.size();
	Size camera2__size = frame2.size();
	//摄像头1参数
	Mat camera1_src;
	Mat camera1_dst;
	int camera1_x = 0;
	int camera1_y = 0;
	double camera1_h33 = 15;//控制图像大小参数
	double camera1_angle = 0;
	Mat camera1_mapx = Mat(camera1__size, CV_32FC1);
	Mat camera1_mapy = Mat(camera1__size, CV_32FC1);
	Mat camera1_R = Mat::eye(3, 3, CV_32F);
	int roi_x = 640;   //设置ROI区域大小
	int roi_y = 480;
	//摄像头2参数
	Mat camera2_src;
	Mat camera2_dst;
	int camera2_x = 108;//108
	int camera2_y = 62;//62
	double camera2_h33 = 13.2;//13.2 控制图像大小参数
	double camera2_angle = 92.7;//92.7
	Mat camera2_mapx = Mat(camera2__size, CV_32FC1);
	Mat camera2_mapy = Mat(camera2__size, CV_32FC1);
	Mat camera2_R = Mat::eye(3, 3, CV_32F);
	initUndistortRectifyMap(camera1_intrinsic_matrix, camera1_distortion_coeffs, camera1_R, camera1_intrinsic_matrix, camera1__size, CV_32FC1, camera1_mapx, camera1_mapy);
	initUndistortRectifyMap(camera2_intrinsic_matrix, camera2_distortion_coeffs, camera2_R, camera2_intrinsic_matrix, camera2__size, CV_32FC1, camera2_mapx, camera2_mapy);
	
	while (1)
	{
		Mat big(1024, 1024, CV_8UC3, Scalar(0, 0, 0));
		if (_kbhit())
		{
			ch2 = _getch();
			if ('q' == ch2)
			{
				system("pause");
				break;
			}
			if ('s' == ch2)
			{
				model++;
				if (model == 5)  model = 1;
				cout << "model=" << model << endl;
			}
			if (model == 1)
			{
				Key(ch2, camera1_x, camera1_y, camera1_h33, camera1_angle, roi_x);
			}
			if (model == 2)
			{
				Key(ch2, camera2_x, camera2_y, camera2_h33, camera2_angle, roi_x);
			}
		}
		capture1 >> camera1_src;
		capture2 >> camera2_src;
		/*rejust_light(camera2_src, camera1_src);*/
		Mat camera1_dst(camera1_src.rows, camera1_src.cols, CV_8UC3, Scalar(0, 0, 0));
		Mat camera2_dst(camera2_src.rows, camera2_src.cols, CV_8UC3, Scalar(0, 0, 0));
		process(camera1_H, camera1_src, camera1_dst, camera1_h33, camera1_angle, camera1_mapx, camera2_mapy, roi_x, roi_y);
		process(camera2_H, camera2_src, camera2_dst, camera2_h33, camera2_angle, camera2_mapx, camera2_mapy, roi_x, roi_y);
		rejust_light(camera1_dst, camera2_dst, camera2_x, camera2_y); //亮度调整
		//复制camera1
		Mat imageROI1 = big(Rect(camera1_x, camera1_y, camera1_dst.cols, camera1_dst.rows));
		Mat mask1;
		cvtColor(camera1_dst, mask1, COLOR_RGB2GRAY); //灰度化获取掩膜
		camera1_dst.copyTo(imageROI1, mask1);
		//findOverlap(camera2_dst, big);
		//复制camera2
		Mat imageROI2 = big(Rect(camera2_x, camera2_y, camera2_dst.cols, camera2_dst.rows));
		Mat mask2;
		cvtColor(camera2_dst, mask2, COLOR_RGB2GRAY); //灰度化获取掩膜
		camera2_dst.copyTo(imageROI2, mask2);
		OptimizeSeam(camera1_dst, camera2_dst, big, camera2_x, camera2_y);
		//rectangle(big, cvPoint(270, 160), cvPoint(370, 320), Scalar(0, 0, 255), 1, 8, 0);//横矩形 
		////画线
		//cv::line(big, cvPoint(270, 160), cvPoint(0, 0), cv::Scalar(0, 0, 255), 1, 4);
		//cv::line(big, cvPoint(370, 160), cvPoint(639, 0), cv::Scalar(0, 0, 255), 1, 4);
		//cv::line(big, cvPoint(270, 320), cvPoint(0, 479), cv::Scalar(0, 0, 255), 1, 4);
		//cv::line(big, cvPoint(370, 320), cvPoint(639, 479), cv::Scalar(0, 0, 255), 1, 4);
		//imshow("掩膜", mask);
		imshow("摄像头1", camera1_src);
		imshow("摄像头2", camera2_src);
		imshow("俯视图1", camera1_dst);
		imshow("俯视图2", camera2_dst);
		imshow("大图", big);
		waitKey(30);
	}
	//保存平移旋转尺度参数
	FileStorage fs("camera1.xml", FileStorage::WRITE);
	//开始文件写入
	fs << "camera1_x" << camera1_x;
	fs << "camera1_y" << camera1_y;
	fs << "camera1_h33" << camera1_h33;
	fs << "camera1_angle" << camera1_angle;
	fs << "camera1_intrinsic_matrix" << camera1_intrinsic_matrix;
	fs << "camera1_distortion_coeffs" << camera1_distortion_coeffs;
	fs << "camera1_H" << camera1_H;
	fs.release();
	return 0;
}