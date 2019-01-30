#define SEED_1 20000
#define SEED_2 2000
#define SEED_3 200
#define SEED_4 100
#define SEED_5 50
#define SEED_6 25
#define SEED_7 10
#define SEED_8 10
#define SEED_9 10
#define SEED_10 10
#define SEED_11 10
#define SEED_12 12

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace cv;
using namespace std;
struct seed_NUM{
	int in_SeedNum;
	int in_SeedCount;
	string in_FileName;
};
ofstream ofresult("result.txt", ios::app);

void Compute_Seed_Parameter(seed_NUM SeedData)
{
	Mat image = imread(SeedData.in_FileName);
	Mat grayImage;
	cvtColor(image, grayImage, CV_BGR2GRAY);
	Mat blurImage;
	GaussianBlur(grayImage, blurImage, Size(5, 5), 0.9, 0.9);
	Mat thresholdImage;
	threshold(blurImage, thresholdImage, 129, 256, CV_THRESH_BINARY);
	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;
	findContours(thresholdImage, contours,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));  //Ñ°ÕÒÂÖÀª
	Mat ResultImage = Mat::zeros(thresholdImage.size(), CV_8UC3);
	vector<vector<Point>>::iterator itc = contours.begin();
	while (itc != contours.end())
	{
		if (itc->size() < 10)
		{
			itc = contours.erase(itc);
		}
		else
		{
			++itc;
		}
	}
	double area;
	double length;
	area = contourArea(contours[0]);
	length = arcLength(Mat(contours[0]), true);
	ofresult << area << " " << length << " " << SeedData.in_SeedNum << endl;
}

int main()
{
	cout << "hello" << endl;
	seed_NUM in_PicData[12];
	in_PicData[0].in_SeedNum = 1;
	in_PicData[0].in_SeedCount = 21728;
	in_PicData[1].in_SeedNum = 2;
	in_PicData[1].in_SeedCount = 1762;
	in_PicData[2].in_SeedNum = 3;
	in_PicData[2].in_SeedCount = 395;
	in_PicData[3].in_SeedNum = 4;
	in_PicData[3].in_SeedCount = 125;
	in_PicData[4].in_SeedNum = 5;
	in_PicData[4].in_SeedCount = 50;
	in_PicData[5].in_SeedNum = 6;
	in_PicData[5].in_SeedCount = 23;
	in_PicData[6].in_SeedNum = 7;
	in_PicData[6].in_SeedCount = 12;
	in_PicData[7].in_SeedNum = 8;
	in_PicData[7].in_SeedCount = 2;
	in_PicData[8].in_SeedNum = 9;
	in_PicData[8].in_SeedCount = 2;
	in_PicData[9].in_SeedNum = 10;
	in_PicData[9].in_SeedCount = 0;
	in_PicData[10].in_SeedNum = 11;
	in_PicData[10].in_SeedCount = 0;
	in_PicData[11].in_SeedNum = 12;
	in_PicData[11].in_SeedCount = 1;

	in_PicData[1].in_FileName = "Pic_";
	in_PicData[1].in_FileName = in_PicData[1].in_FileName + "1";
	in_PicData[1].in_FileName = "H:\\SeedPic\\" + in_PicData[1].in_FileName + ".jpg";
	Compute_Seed_Parameter(in_PicData[1]);


	/*
	for (int i = 0; i < 12; i++)
	{
		int n = 1;
		while (n <= in_PicData[i].in_SeedCount)
		{
			in_PicData[i].in_FileName = "Pic_";
			stringstream ss;
			string str;
			ss << n;
			ss >> str;
			in_PicData[i].in_FileName = in_PicData[i].in_FileName + str;
			in_PicData[i].in_FileName = "H:\\SeedPic\\" + in_PicData[i].in_FileName + ".jpg";
			Compute_Seed_Parameter(in_PicData[i]);
			n++;
		}
	}
	*/
	system("pause");
	return 0;
}