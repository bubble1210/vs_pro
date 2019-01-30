#include <opencv2\opencv.hpp>
#include <conio.h>
#include <fstream>
using namespace std;
using namespace cv;

int main()
{
	if (1)
	{
		/*---------------------------------------------��ȡ�����۽�����Ӧ�Ծ����Լ���ת����ͼ��Ӧ�Ծ���--------------------------------------------------*/

		/*-----------------------------------------------------------------��ȡ���۽�����Ӧ�Ծ���------------------------------------------------------------*/

		//ofstream fout("caliberation_result.txt");  /**    ���涨�������ļ�     **/
		FileStorage fswrite("caliberation_result.xml", FileStorage::WRITE); //����XML�ļ� 
		/************************************************************************
		��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��
		*************************************************************************/


		cout << "��ʼ��ȡ�ǵ㡭����������" << endl;
		int image_count = 29;                    /****    ͼ������     ****/
		Size board_size = Size(6, 4);            /****    �������ÿ�С��еĽǵ���       ****/
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
			//fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
		}
		cout << "����ƽ����" << total_err / image_count << "����" << endl;
		//fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
		cout << "������ɣ�" << endl;

		/************************************************************************
		���涨����
		*************************************************************************/
		cout << "��ʼ���涨����������������" << endl;
		Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
		//����ڲ�������intrinsic_matrix
		fswrite << "intrinsic_matrix" << intrinsic_matrix;
		//�������distortion_coeffs
		fswrite << "distortion_coeffs" << distortion_coeffs;
		for (int i = 0; i < image_count; i++)
		{
			//fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
			//fout << rotation_vectors[i] << endl;

			/* ����ת����ת��Ϊ���Ӧ����ת���� */
			Rodrigues(rotation_vectors[i], rotation_matrix);
			//fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
			//fout << rotation_matrix << endl;
			//fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
			//fout << translation_vectors[i] << endl;
		}
		cout << "��ɱ���" << endl;
		//fout << endl;
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
	}
	//���۽�������ͼƬ����
	if (0)
	{
		//Mat dst;
		//Mat src = imread("camera1.jpg");
		VideoCapture capture(1);
		capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		Mat frame;
		capture >> frame;
		Size camera__size = frame.size();
		//д�����۽�������
		FileStorage fs("caliberation_result.xml", FileStorage::READ);
		Mat intrinsic_matrix, distortion_coeffs;
		fs["intrinsic_matrix"] >> intrinsic_matrix;
		fs["distortion_coeffs"] >> distortion_coeffs;
		fs.release();
		//����ͷ1����
		Mat camera_src;
		Mat camera_dst;
		Mat camera_mapx = Mat(camera__size, CV_32FC1);
		Mat camera_mapy = Mat(camera__size, CV_32FC1);
		Mat camera_R = Mat::eye(3, 3, CV_32F);
		initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, camera_R, intrinsic_matrix, camera__size, CV_32FC1, camera_mapx, camera_mapy);
		//remap(src, dst, camera_mapx, camera_mapy, INTER_LINEAR);//���۽���, camera_dst, camera_mapx, camera_mapy, INTER_LINEAR);//���۽���
		//imshow("���Ų���", dst);
		//imwrite("camera_fish1.jpg", dst);
		while (1)
		{
			capture >> camera_src;
			remap(camera_src, camera_dst, camera_mapx, camera_mapy, INTER_LINEAR);//���۽���
			imshow("����ͷԭͼ", camera_src);
			imshow("���۽���", camera_dst);
			//imshow("��ͼ", big);
			waitKey(30);
		}
	}
	waitKey(0);
	return 0;
}