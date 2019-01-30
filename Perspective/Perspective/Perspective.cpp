#include <iostream>

#include "highgui.h"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;

void main()
{
	// 定义相关参数
	const int num = 43339;
	char fileName[50];
	char saveName[50];
	int i = 617;
		// sprintf读入指定路径下图片序列
		sprintf_s(fileName, "F:\\\\Middle_Camera_Highway\\\\%d.jpg", i);
		sprintf_s(saveName, "D:\\\\VS_PRO\\\\Perspective\\\\\Perspective\\\\%d.jpg", i);
		//cout << fileName << endl;

		// get original image.
		cv::Mat originalImage = cv::imread(fileName);

		// perspective image.
		cv::Mat perspectiveImage;

		// perspective transform
		cv::Point2f objectivePoints[4], imagePoints[4];

		// original image points.
		imagePoints[0].x = 63.0; imagePoints[0].y = 907.0;
		imagePoints[1].x = 606.0; imagePoints[1].y = 495.0;
		imagePoints[2].x = 683.0; imagePoints[2].y = 495.0;
		imagePoints[3].x = 1254.0; imagePoints[3].y = 907.0;

		// objective points of perspective image.
		// move up the perspective image : objectivePoints.y - value .
		// move left the perspective image : objectivePoints.x - value.
		double moveValueX = 0.0;
		double moveValueY = 0.0;

		objectivePoints[0].x = 100.0 + moveValueX; objectivePoints[0].y = 1200.0 + moveValueY;
		objectivePoints[1].x = 100.0 + moveValueX; objectivePoints[1].y = 100.0 + moveValueY;
		objectivePoints[2].x = 900.0 + moveValueX; objectivePoints[2].y = 100.0 + moveValueY;
		objectivePoints[3].x = 900.0 + moveValueX; objectivePoints[3].y = 1200.0 + moveValueY;

		cv::Mat transform = cv::getPerspectiveTransform(objectivePoints, imagePoints);
		cout << transform << endl;
		// perspective.
		cv::warpPerspective(originalImage,
			perspectiveImage,
			transform,
			cv::Size(originalImage.rows, originalImage.cols),
			cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

		// cv::imshow("perspective image", perspectiveImage);
		// cvWaitKey(0);

		cv::imwrite(saveName, perspectiveImage);
	cin >> saveName;

}