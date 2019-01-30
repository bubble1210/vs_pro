#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;
 
int main()
{
	VideoCapture capture(0);
	//capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	capture.set(CV_CAP_PROP_FPS, 30);
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	Mat frame; 
	capture >> frame;
	Size camera__size = frame.size();
	//写入鱼眼矫正参数
	FileStorage fs("camera4_fish_720.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs;
	fs["intrinsic_matrix"] >> intrinsic_matrix;
	fs["distortion_coeffs"] >> distortion_coeffs;
	fs.release();
	Mat intrinsic_matrix2(intrinsic_matrix), new_intrinsic_mat;

	intrinsic_matrix2.copyTo(new_intrinsic_mat);
	//调节视场大小,乘的系数越小视场越大
	new_intrinsic_mat.at<double>(0, 0) *= 1;
	new_intrinsic_mat.at<double>(1, 1) *= 1;
	//调节校正图中心，一般不做改变
	new_intrinsic_mat.at<double>(0, 2) += 0;
	new_intrinsic_mat.at<double>(1, 2) += 0;
	//摄像头1参数
	Mat camera_src;
	Mat camera_dst;
	Mat camera_mapx = Mat(camera__size, CV_32FC1);
	Mat camera_mapy = Mat(camera__size, CV_32FC1);
	Mat camera_R = Mat::eye(3, 3, CV_32F);
	initUndistortRectifyMap(new_intrinsic_mat, distortion_coeffs, camera_R, new_intrinsic_mat, camera__size, CV_32FC1, camera_mapx, camera_mapy);
	//remap(src, dst, camera_mapx, camera_mapy, INTER_LINEAR);//鱼眼矫正, camera_dst, camera_mapx, camera_mapy, INTER_LINEAR);//鱼眼矫正
	//imshow("单张测试", dst);
	//imwrite("camera_fish1.jpg", dst);
	while (1)
	{
		capture >> camera_src;
		//undistort(camera_src, camera_dst, intrinsic_matrix2, distortion_coeffs, new_intrinsic_mat);
		remap(camera_src, camera_dst, camera_mapx, camera_mapy, INTER_LINEAR);//鱼眼矫正
		imshow("摄像头原图", camera_src);
		imshow("鱼眼矫正", camera_dst);
		waitKey(1);
	}
	return 0;
}