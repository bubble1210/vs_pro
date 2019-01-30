#include <opencv2\opencv.hpp>
#include <fstream>
using namespace std;
using namespace cv;


Mat Calculation_H(Mat &src_img)
{
	/*-------------------------------------------------------------------��ȡת����ͼ��Ӧ�Ծ���--------------------------------------------------------------------*/
	int board_w = 6;
	int board_h = 4;
	int board_n = board_w*board_h;
	Size board_sz = Size(board_w, board_h);

	//Ѱ��4���Ӧ������
	//Mat src = imread("img1.jpg");
	//Mat src = frame1.clone();201112
	Mat srcImage = src_img.clone();
	imshow("The undistort image", srcImage);
	
	Mat grayImage, srcCopy;
	srcCopy = srcImage.clone();
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
	namedWindow("grayImage");
	imshow("grayImage", grayImage);
	waitKey(1000);

	vector<Point2f> corners2;
	bool found = findChessboardCorners(srcImage, board_sz, corners2);
	//bool found = findChessboardCorners(srcImage, srcImage.size(), corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
	//CALIB_CB_FAST_CHECK);
	if (found == 1)
	{
		//cornerSubPix(grayImage, corners, Size(5, 5), Size(-1, -1),TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.1));
		cornerSubPix(grayImage, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	}
	cout << corners2 << endl;
	drawChessboardCorners(srcCopy, board_sz, corners2, found);
	namedWindow("chess");
	imshow("chess", srcCopy);
	waitKey(1000);
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
	cout << objectPoints[3] << imagePoints[3] << endl;

	Mat H;
	//���㵥Ӧ����
	/*����obj��img�ĵ�Ӧ���������ֱ�Ӽ���img��obj��Ӧ�������Ҫԭ���ǿɿ�ת��������ͼ��Ĵ�С
	����Ʋ�����Ϊh33������MATLAB�������֪h33����������Ե�����ϵ���Ӷ��ܱ�֤ͼ��ʧ��*/
	H = getPerspectiveTransform(objectPoints, imagePoints);

	//д��xml�ļ�
	FileStorage fsout("camera1_transform.xml", FileStorage::WRITE);
	//��ʼ�ļ�д��
	fsout << "H" << H;
	fsout.release();

	return H;
}
int main()
{
	Mat frame1;
	Mat camera_src;
	Mat camera_dst;
	VideoCapture capture1(1);
	//capture1.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	capture1.set(CV_CAP_PROP_FPS, 30);
	capture1.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture1.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	for (int i = 0; i < 50; i++)
	{
		capture1 >> frame1;
	}
	
	//д�����۽�������
	FileStorage fs("camera2_fish_720_2.xml", FileStorage::READ);
	Mat intrinsic_matrix, distortion_coeffs;
	fs["intrinsic_matrix"] >> intrinsic_matrix;
	fs["distortion_coeffs"] >> distortion_coeffs;
	fs.release();
	Size camera__size = frame1.size();
	Mat camera_mapx = Mat(camera__size, CV_32FC1);
	Mat camera_mapy = Mat(camera__size, CV_32FC1);
	Mat camera_R = Mat::eye(3, 3, CV_32F);
	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, camera_R, intrinsic_matrix, camera__size, CV_32FC1, camera_mapx, camera_mapy);
	remap(frame1, camera_dst, camera_mapx, camera_mapy, INTER_LINEAR);//���۽���
	/*-------------------------------------------------------------------��ȡת����ͼ��Ӧ�Ծ���--------------------------------------------------------------------*/
	Mat H;
	Mat birdImage;
	Mat src = imread("img1.jpg");
	H = Calculation_H(camera_dst);


	/*H = getPerspectiveTransform(imagePoints, objectPoints);*/
	H.at<double>(2, 2) = 16;//����ͼ���С����
	//Mat a = H.inv();
	cout << H << endl;
	//warpPerspective(srcImage, birdImage, H, srcImage.size(), WARP_INVERSE_MAP + INTER_LINEAR);
	//namedWindow("birdImage");
	//imshow("birdImage", birdImage);
	
	while (1)
	{
		capture1 >> camera_src;
		remap(camera_src, camera_dst, camera_mapx, camera_mapy, INTER_LINEAR);//���۽���
		warpPerspective(camera_dst, birdImage, H, camera_dst.size(), WARP_INVERSE_MAP + INTER_LINEAR);
		imshow("���۽���", camera_dst);
		imshow("birdImage", birdImage);
		waitKey(30);
		
	}

	//imwrite("right_img.jpg", birdImage);
	return 0;
}