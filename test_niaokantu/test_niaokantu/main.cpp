//运行环境 VS2012+opencv3.0
#include <opencv2\opencv.hpp>
#include <fstream>
using namespace std;
using namespace cv;

int main()
{
	/*---------------------------------------------求取《鱼眼矫正单应性矩阵》以及《转俯视图单应性矩阵》--------------------------------------------------*/

	/*-----------------------------------------------------------------求取鱼眼矫正单应性矩阵------------------------------------------------------------*/

	ofstream fout("caliberation_result.txt");  /**    保存定标结果的文件     **/

	/************************************************************************
		  读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
	*************************************************************************/


	cout << "开始提取角点………………" << endl;
	int image_count = 26;                    /****    图像数量     ****/
	Size board_size = Size(5, 4);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	vector<vector<Point2f>>  corners_Seq;    /****  保存检测到的所有角点       ****/
	vector<Mat>  image_Seq;
	int successImageNum = 0;				/****	成功提取角点的棋盘图数量	****/

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
		/* 提取角点 */
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
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			/* 绘制检测到的角点并保存 */
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
	cout << "角点提取完成！\n";
	/************************************************************************
								摄像机定标
	*************************************************************************/
	cout << "开始定标………………" << endl;
	Size square_size = Size(28, 28);
	vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   保存提取的所有角点   *****/
	vector<int>  point_counts;
	/* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t < successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
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
	/* 开始定标 */
	Size image_size = image_Seq[0].size();
	cv::Mat intrinsic_matrix;    /*****    摄像机内参数矩阵    ****/
	cv::Mat distortion_coeffs;     /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3*/
	std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
	std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
	calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, 0);
	//int flags = 0;
	//flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	//flags |= cv::fisheye::CALIB_CHECK_COND;
	//flags |= cv::fisheye::CALIB_FIX_SKEW;
	//fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
	cout << "定标完成！\n";

	/************************************************************************
	对定标结果进行评价
	*************************************************************************/
	cout << "开始评价定标结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/

	cout << "每幅图像的定标误差：" << endl;
	cout << "每幅图像的定标误差：" << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
		//projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
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
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	cout << "评价完成！" << endl;

	/************************************************************************
	保存定标结果
	*************************************************************************/
	cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */

	fout << "相机内参数矩阵：" << endl;
	fout << intrinsic_matrix << endl;
	fout << "畸变系数：\n";
	fout << distortion_coeffs << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rotation_vectors[i] << endl;

		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rotation_vectors[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << translation_vectors[i] << endl;
	}
	cout << "完成保存" << endl;
	fout << endl;
	/************************************************************************
	显示定标结果
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);

	cout << "保存矫正图像" << endl;
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
	cout << "保存结束" << endl;

	/************************************************************************
	鱼眼矫正单应性矩阵测试一张图片
	*************************************************************************/
	if (1)
	{
		cout << "TestImage ..." << endl;
		Mat distort_img = imread("1.jpg", 1);
		imshow("鱼眼矫正原图片",distort_img);
		Mat undistort_img;
		//FileStorage fs("test.xml", FileStorage::READ);
		//开始文件写入
		//Mat intrinsic_matrix2, distortion_coeffs2;
		//fs["intrinsic_matrix"] >> intrinsic_matrix;
		//fs["distortion_coeffs"] >> distortion_coeffs;
		//fs.release();
		//Mat intrinsic_mat(intrinsic_matrix2), new_intrinsic_mat;

		//intrinsic_mat.copyTo(new_intrinsic_mat);
		//调节视场大小,乘的系数越小视场越大
		//new_intrinsic_mat.at<double>(0, 0) *= 0.5;
		//new_intrinsic_mat.at<double>(1, 1) *= 0.5;
		//调节校正图中心，一般不做改变
		//new_intrinsic_mat.at<double>(0, 2) += 0;
		//new_intrinsic_mat.at<double>(1, 2) += 0;

		//undistort(distort_img, undistort_img, intrinsic_matrix2, distortion_coeffs2, new_intrinsic_mat);
		//initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
		cv::remap(distort_img, undistort_img, mapx, mapy, INTER_LINEAR);
		imwrite("1_1.jpg", undistort_img);
		imshow("鱼眼矫正后图片", undistort_img);
		cout << "保存结束" << endl;
	}

	/*-------------------------------------------------------------------求取转俯视图单应性矩阵--------------------------------------------------------------------*/

	Mat birdImage;
	int board_w = 6;
	int board_h = 4;
	int board_n = board_w*board_h;
	Size board_sz = Size(board_w, board_h);


	//寻找4组对应点坐标
	Mat src = imread("1_3.jpg");
	Mat srcImage = src.clone();
	imshow("The undistort image", srcImage);
	//Mat srcImage = src_1(Range(101, 385), Range(81, 540));     //对矫正图片进行剪切*************
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
	//计算单应矩阵
	/*计算obj到img的单应矩阵而不是直接计算img到obj单应矩阵的主要原因是可控转换后输入图像的大小
	其控制参数即为h33，经过MATLAB的运算可知h33与坐标成线性单增关系，从而能保证图像不失真*/
	H = getPerspectiveTransform(objectPoints, imagePoints);

	FileStorage fs("test.xml", FileStorage::WRITE);
	//开始文件写入
	fs << "H" << H;
	//fs << "intrinsic_matrix" << intrinsic_matrix;
	//fs << "distortion_coeffs" << distortion_coeffs;
	fs.release();
	/*H = getPerspectiveTransform(imagePoints, objectPoints);*/
	H.at<double>(2, 2) = 40;//控制图像大小参数
	Mat a = H.inv();
	cout << H << "\n" << a << endl;
	warpPerspective(srcImage, birdImage, H, srcImage.size(), WARP_INVERSE_MAP + INTER_LINEAR);
	namedWindow("birdImage");
	imshow("birdImage", birdImage);
	
	
	/************************************************************************
	鱼眼矫正单应性矩阵测试一张图片
	*************************************************************************/
	/*if (1)
	{
		cout << "TestImage ..." << endl;
		Mat distort_img = imread("1.jpg", 1);
		Mat undistort_img;
		FileStorage fs("test.xml", FileStorage::READ);
		//开始文件写入
		Mat intrinsic_matrix2, distortion_coeffs2;
		fs["intrinsic_matrix"] >> intrinsic_matrix2;
		fs["distortion_coeffs"] >> distortion_coeffs2;
		fs.release();
		Mat intrinsic_mat(intrinsic_matrix2), new_intrinsic_mat;

		intrinsic_mat.copyTo(new_intrinsic_mat);
		//调节视场大小,乘的系数越小视场越大
		new_intrinsic_mat.at<double>(0, 0) *= 0.5;
		new_intrinsic_mat.at<double>(1, 1) *= 0.5;
		//调节校正图中心，一般不做改变
		new_intrinsic_mat.at<double>(0, 2) += 0;
		new_intrinsic_mat.at<double>(1, 2) += 0;

		undistort(distort_img, undistort_img, intrinsic_matrix2, distortion_coeffs2, new_intrinsic_mat);
		imwrite("1_2.jpg", undistort_img);
		cout << "保存结束" << endl;
	}*/
	waitKey(0);
	return 0;
}