//���л��� VS2012+opencv3.0
#include <opencv2\opencv.hpp>
#include <fstream>
using namespace std;
using namespace cv;

int main()
{
	/*---------------------------------------------��ȡ�����۽�����Ӧ�Ծ����Լ���ת����ͼ��Ӧ�Ծ���--------------------------------------------------*/

	/*-----------------------------------------------------------------��ȡ���۽�����Ӧ�Ծ���------------------------------------------------------------*/

	ofstream fout("caliberation_result.txt");  /**    ���涨�������ļ�     **/

	/************************************************************************
		  ��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��
	*************************************************************************/


	cout << "��ʼ��ȡ�ǵ㡭����������" << endl;
	int image_count = 26;                    /****    ͼ������     ****/
	Size board_size = Size(5, 4);            /****    �������ÿ�С��еĽǵ���       ****/
	vector<Point2f> corners;                  /****    ����ÿ��ͼ���ϼ�⵽�Ľǵ�       ****/
	vector<vector<Point2f>>  corners_Seq;    /****  �����⵽�����нǵ�       ****/
	vector<Mat>  image_Seq;
	int successImageNum = 0;				/****	�ɹ���ȡ�ǵ������ͼ����	****/

	int count = 0;
	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		string imageFileName;
		std::stringstream StrStm;
		StrStm << i + 1;
		StrStm >> imageFileName;
		imageFileName += ".jpg";
		cv::Mat image = imread("img" + imageFileName);
		/* ��ȡ�ǵ� */
		Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
			CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << "can not find chessboard corners!\n";
			continue;
			exit(1);
		}
		else
		{
			/* �����ؾ�ȷ�� */
			cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			/* ���Ƽ�⵽�Ľǵ㲢���� */
			Mat imageTemp = image.clone();
			for (int j = 0; j < corners.size(); j++)
			{
				circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
			}
			//imshow("cornwe", imageTemp);
			//waitKey(1);
			string imageFileName;
			std::stringstream StrStm;
			StrStm << i + 1;
			StrStm >> imageFileName;
			imageFileName += "_corner.jpg";
			imwrite(imageFileName, imageTemp);
			cout << "Frame corner#" << i + 1 << "...end" << endl;

			count = count + corners.size();
			successImageNum = successImageNum + 1;
			corners_Seq.push_back(corners);
		}
		image_Seq.push_back(image);
	}
	cout << "�ǵ���ȡ��ɣ�\n";
	/************************************************************************
								���������
	*************************************************************************/
	cout << "��ʼ���ꡭ����������" << endl;
	Size square_size = Size(28, 28);
	vector<vector<Point3f>>  object_Points;        /****  ���涨����Ͻǵ����ά����   ****/

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   ������ȡ�����нǵ�   *****/
	vector<int>  point_counts;
	/* ��ʼ��������Ͻǵ����ά���� */
	for (int t = 0; t < successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				/* ���趨��������������ϵ��z=0��ƽ���� */
				Point3f tempPoint;
				tempPoint.x = i*square_size.width;
				tempPoint.y = j*square_size.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	for (int i = 0; i < successImageNum; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* ��ʼ���� */
	Size image_size = image_Seq[0].size();
	cv::Mat intrinsic_matrix;    /*****    ������ڲ�������    ****/
	cv::Mat distortion_coeffs;     /* �������5������ϵ����k1,k2,p1,p2,k3*/
	std::vector<cv::Vec3d> rotation_vectors;                           /* ÿ��ͼ�����ת���� */
	std::vector<cv::Vec3d> translation_vectors;                        /* ÿ��ͼ���ƽ������ */
	calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, 0);
	//int flags = 0;
	//flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	//flags |= cv::fisheye::CALIB_CHECK_COND;
	//flags |= cv::fisheye::CALIB_FIX_SKEW;
	//fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
	cout << "������ɣ�\n";

	/************************************************************************
	�Զ�������������
	*************************************************************************/
	cout << "��ʼ���۶�����������������" << endl;
	double total_err = 0.0;                   /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0;                        /* ÿ��ͼ���ƽ����� */
	vector<Point2f>  image_points2;             /****   �������¼���õ���ͶӰ��    ****/

	cout << "ÿ��ͼ��Ķ�����" << endl;
	cout << "ÿ��ͼ��Ķ�����" << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/****    ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ��     ****/
		//projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	cout << "����ƽ����" << total_err / image_count << "����" << endl;
	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
	cout << "������ɣ�" << endl;

	/************************************************************************
	���涨����
	*************************************************************************/
	cout << "��ʼ���涨����������������" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */

	fout << "����ڲ�������" << endl;
	fout << intrinsic_matrix << endl;
	fout << "����ϵ����\n";
	fout << distortion_coeffs << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << rotation_vectors[i] << endl;

		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(rotation_vectors[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << translation_vectors[i] << endl;
	}
	cout << "��ɱ���" << endl;
	fout << endl;
	/************************************************************************
	��ʾ������
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);

	cout << "�������ͼ��" << endl;
	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
		initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
		Mat t = image_Seq[i].clone();
		cv::remap(image_Seq[i], t, mapx, mapy, INTER_LINEAR);
		string imageFileName;
		std::stringstream StrStm;
		StrStm << i + 1;
		StrStm >> imageFileName;
		imageFileName += "_d.jpg";
		imwrite(imageFileName, t);
	}
	cout << "�������" << endl;

	/************************************************************************
	���۽�����Ӧ�Ծ������һ��ͼƬ
	*************************************************************************/
	if (1)
	{
		cout << "TestImage ..." << endl;
		Mat distort_img = imread("1.jpg", 1);
		imshow("���۽���ԭͼƬ",distort_img);
		Mat undistort_img;
		//FileStorage fs("test.xml", FileStorage::READ);
		//��ʼ�ļ�д��
		//Mat intrinsic_matrix2, distortion_coeffs2;
		//fs["intrinsic_matrix"] >> intrinsic_matrix;
		//fs["distortion_coeffs"] >> distortion_coeffs;
		//fs.release();
		//Mat intrinsic_mat(intrinsic_matrix2), new_intrinsic_mat;

		//intrinsic_mat.copyTo(new_intrinsic_mat);
		//�����ӳ���С,�˵�ϵ��ԽС�ӳ�Խ��
		//new_intrinsic_mat.at<double>(0, 0) *= 0.5;
		//new_intrinsic_mat.at<double>(1, 1) *= 0.5;
		//����У��ͼ���ģ�һ�㲻���ı�
		//new_intrinsic_mat.at<double>(0, 2) += 0;
		//new_intrinsic_mat.at<double>(1, 2) += 0;

		//undistort(distort_img, undistort_img, intrinsic_matrix2, distortion_coeffs2, new_intrinsic_mat);
		//initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
		cv::remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);
		imwrite("1_1.jpg", undistort_img);
		imshow("���۽�����ͼƬ", undistort_img);
		cout << "�������" << endl;
	}

	/*-------------------------------------------------------------------��ȡת����ͼ��Ӧ�Ծ���--------------------------------------------------------------------*/

	Mat birdImage;
	int board_w = 6;
	int board_h = 4;
	int board_n = board_w*board_h;
	Size board_sz = Size(board_w, board_h);


	//Ѱ��4���Ӧ������
	Mat src = imread("1_3.jpg");
	Mat srcImage = src.clone();
	imshow("The undistort image", srcImage);
	//Mat srcImage = src_1(Range(101, 385), Range(81, 540));     //�Խ���ͼƬ���м���*************
															   //Mat testImage = imread("img2.jpg");
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

	FileStorage fs("test.xml", FileStorage::WRITE);
	//��ʼ�ļ�д��
	fs << "H" << H;
	//fs << "intrinsic_matrix" << intrinsic_matrix;
	//fs << "distortion_coeffs" << distortion_coeffs;
	fs.release();
	/*H = getPerspectiveTransform(imagePoints, objectPoints);*/
	H.at<double>(2, 2) = 40;//����ͼ���С����
	Mat a = H.inv();
	cout << H << "\n" << a << endl;
	warpPerspective(srcImage, birdImage, H, srcImage.size(), WARP_INVERSE_MAP + INTER_LINEAR);
	namedWindow("birdImage");
	imshow("birdImage", birdImage);
	
	
	/************************************************************************
	���۽�����Ӧ�Ծ������һ��ͼƬ
	*************************************************************************/
	/*if (1)
	{
		cout << "TestImage ..." << endl;
		Mat distort_img = imread("1.jpg", 1);
		Mat undistort_img;
		FileStorage fs("test.xml", FileStorage::READ);
		//��ʼ�ļ�д��
		Mat intrinsic_matrix2, distortion_coeffs2;
		fs["intrinsic_matrix"] >> intrinsic_matrix2;
		fs["distortion_coeffs"] >> distortion_coeffs2;
		fs.release();
		Mat intrinsic_mat(intrinsic_matrix2), new_intrinsic_mat;

		intrinsic_mat.copyTo(new_intrinsic_mat);
		//�����ӳ���С,�˵�ϵ��ԽС�ӳ�Խ��
		new_intrinsic_mat.at<double>(0, 0) *= 0.5;
		new_intrinsic_mat.at<double>(1, 1) *= 0.5;
		//����У��ͼ���ģ�һ�㲻���ı�
		new_intrinsic_mat.at<double>(0, 2) += 0;
		new_intrinsic_mat.at<double>(1, 2) += 0;

		undistort(distort_img, undistort_img, intrinsic_matrix2, distortion_coeffs2, new_intrinsic_mat);
		imwrite("1_2.jpg", undistort_img);
		cout << "�������" << endl;
	}*/
	waitKey(0);
	return 0;
}