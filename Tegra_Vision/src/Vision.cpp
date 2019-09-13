/*
 * Vision.cpp
 *
 *  Created on: July, 2017 - Present
 *      Author: Resurgence Robotics Development Team
 */

//System Includes
#include <sys/types.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <math.h>


//OpenCV Includes
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>


using namespace cv;
using namespace std;



//Camera Constants used for distance calculations
#define X_IMAGE_RES 640
#define Y_IMAGE_RES 360
#define VIEW_ANGLE 34.8665269
#define VIEW_ANGLE_X 61
#define VIEW_ANGLE_Y 34.3
const double PI = 3.141592653589793;

//Image Locations
#define CAM_ID 0 //USB Camera
#define CAM_IP 10.10.80.250 //IP Camera -- Not Used

//Edge profile constants of our shape
#define Twidth 2 //Inches
#define Theight 5 //Inches
#define TRatio Twidth/Theight

//Define the HSV threshold values
int thresh = 100;
int Hue[] {70, 255};
int Sat[] {168, 255};
int Val[] {40, 255};
int minArea = 20.0; //Play with this
double minPerimeter = 0.0;
double minWidth = 0.0;
double maxWidth = 1000.0;
double minHeight = 0.0;
double maxHeight = 1000.0;
int solidity[] = {0, 1000};
double maxVertices = 1000000.0;
double minVertices = 0; //Play with this
int minRatio = 50; //Play with this
double maxRatio = 1000.0;

//Create a array of contrors that make up a shape
typedef vector<vector<Point> > ShapeArray;
typedef vector<Point> Points;

string Image("C:\\Users\\ellen.estep.one\\FRC_Vision\\Tegra_Vision\\dev\\CargoSideStraightDark36in.jpg");

//Function to match two value's scales proportionally
float Map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return ((x - in_min) * (out_max- out_min) / (in_max - in_min) + out_min);
}

float Limit(float num)
{
	if(num > 1.0)
	{
		return 1.0;
	}

	if(num < -1.0)
	{
		return -1.0;
	}

	return num;
}

//Structure to represent the scores for various tests used for target identification
struct Report
{
	int GoalCount;
	Point Center;
	double xEdge;
	double yEdge;
	double Distance;
	double Skew;
	float Angle;
	double PointOfAim;
	double AverageDistance;
};

void HSVThreshold(Mat &input, Mat &output, int Hue[], int Sat[], int Val[]);
void GetContours(Mat &input, ShapeArray &contours, Mat &Output);
void FilterContours(ShapeArray &input, ShapeArray &output, Mat &OutputImage);
void GenerateTargetReport(ShapeArray &input, Report Goal[], Mat &FilteredGoals);
void FindTargets(ShapeArray &input, Report Goal[], Mat &FilteredGoals);
void sendToNetworkTables(double input1, double input2);


int main()
{
	//Create a new VideoCapture instance called Stream
	/*VideoCapture Stream(CAM_ID);

	//Check if the stream is open if not stop the program
	if(!Stream.isOpened())
		return -1;
*/
	Mat ImageMat(X_IMAGE_RES, Y_IMAGE_RES, CV_CN_MAX);
	Mat image;

	       // LOAD image
	image = imread(Image, CV_LOAD_IMAGE_COLOR);   // Read the file "image.jpg".
	              //This file "image.jpg" should be in the project folder.
	              //Else provide full address : "D:/images/image.jpg"




	int FPSWait = 20;

	while(true)
	{
		//Create a frame from the video stream
		Mat frame(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC3);
		frame = image;
		//Stream >> frame;
		Mat Blur(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC3);
		//Gaussian Blur
		GaussianBlur(frame, Blur, Size(5, 5), 0, 0);

		Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
		    //dilate with larger element so make sure object is nicely visible
		Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

		erode(Blur,Blur,erodeElement);
		erode(Blur,Blur,erodeElement);

		//dilate(Blur,Blur,dilateElement);
		//dilate(Blur,Blur,dilateElement);

		//HSV Filter
		Mat HSVThresholdOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC3);
		HSVThreshold(Blur, HSVThresholdOutput, Hue, Sat, Val);


		//Identify Contours
		ShapeArray contours;
		Mat ContoursOutput(X_IMAGE_RES, Y_IMAGE_RES, CV_8UC1);
		GetContours(HSVThresholdOutput, contours, ContoursOutput);
		//Filter Contours
		ShapeArray Goals;
		FilterContours(contours, Goals, frame);
		//Calculate and score objects
		Report GoalReport[Goals.size()];
		FindTargets(Goals, GoalReport, frame);
		//GenerateTargetReport(Goals, GoalReport, frame);
		imshow("Final Image", frame);
		imshow("Da Blur", Blur);
		imshow("Binary Mask", ContoursOutput);
		imshow("HSV", HSVThresholdOutput);

		int max_thresh = 255;
		char source_window[] = "Source";
		namedWindow(source_window, WINDOW_NORMAL);
		createTrackbar( " MaxHue:", "Source", &Hue[1], max_thresh);
		createTrackbar( " MinHue:", "Source", &Hue[0], max_thresh);
		createTrackbar( " MaxSat:", "Source", &Sat[1], max_thresh);
		createTrackbar( " MinSat:", "Source", &Sat[0], max_thresh);
		createTrackbar( " MaxVal:", "Source", &Val[1], max_thresh);
		createTrackbar( " MinVal:", "Source", &Val[0], max_thresh);
		createTrackbar( " FPSWait:", "Source", &FPSWait, 250);
		createTrackbar( " MinArea:", "Source", &minArea, 5000);
		createTrackbar( " MinRatio:", "Source", &minRatio, 200);
		createTrackbar( " MinSolid:", "Source", &solidity[0], 1000);
		waitKey(FPSWait);// waits to exit program until we have pressed a key
	}

	return 0;


}


void ReduceBrightness(Mat &input, Mat &output, int factor)
{
//Mat new_image = Mat::zeros( input.size(), input.type() );
	for(int i = 1; i < X_IMAGE_RES-1; i++)
	{
		usleep(1000);
		for(int j = 1; j < Y_IMAGE_RES-1; j++)
		{
			for(int c=0; c < 3; c++)
			{
				//int value = int(input.at<Vec3i>(j,i)[c]);
				//value = value*float(factor);
				//cout << value << endl;
				input.at<Vec3i>(j,i)[c] = 0;
				printf("%i,%i,%i \n",i, j, c );

			}

		}

	}
	printf("--------------------------------------------------------------------------------------------------------------------\n");
}

void HSVThreshold(Mat &input, Mat &output, int Hue[], int Sat[], int Val[])// this function uses an input and output givin by the user allowing major flexibility
{
	cvtColor(input,output,COLOR_BGR2HSV);// convert the image from color image channles to 2 channel HSV "black white", binary
	inRange(output, Scalar(Hue[0],Sat[0],Val[0]),Scalar(Hue[1],Sat[1],Val[1]),output);
	//imwrite("HSVfilteredImage" ,output);// create a new image to show the processing worked
}

void GetContours(Mat &input, ShapeArray &contours, Mat &output)
{
	Mat CannyOutput;
	Canny(input, CannyOutput, X_IMAGE_RES, Y_IMAGE_RES);
	vector<Vec4i> hierarchy;
	findContours(CannyOutput, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//imwrite("ContoursImage", CannyOutput);
	output=CannyOutput;
}

void FilterContours(ShapeArray &input, ShapeArray &output, Mat &OutputImage )
{

	ShapeArray hull(input.size());


	output.clear();
	int sizeOfArray=int(input.size())-1;
	vector<Vec4i> hierarchy;

	for (int i=0; i<= sizeOfArray; i++)
			{
				Points store =input[i];
				Rect bb = boundingRect(store);
				if (bb.width < minWidth || bb.width > maxWidth) continue;
				if (bb.height < minHeight || bb.height > maxHeight) continue;
				double area = contourArea(store);
				if (area < minArea) continue;
				if (arcLength(store, true) < minPerimeter) continue;
				convexHull( Mat(store,true), hull[i], false );
				double solid = 100 * area / contourArea(hull[i]);
				if (solid < solidity[0] || solid > solidity[1]) continue;
				if (store.size() < minVertices || store.size() > maxVertices)	continue;
				double ratio = (double) bb.width / (double) bb.height;
				if (ratio < (minRatio/100) || ratio > maxRatio) continue;
				output.push_back(store);
				store.clear();

			}
			//drawContours(OutputImage, input, -1, 255, 3);
			//imwrite("filteredImage", OutputImage);
}

void GenerateTargetReport(ShapeArray &input, Report Goal[], Mat &FilteredGoals)
{
	int sizeOfArray=int(input.size());// cast to signed int and subtract 1 to prevent using a number outside the scope of the array
	if(sizeOfArray==0)
	{
		printf("0 goal(s), \n");
		return;
	}

	for(int i=0; i<sizeOfArray; i++)
	{
		Rect bb = boundingRect(input[i]);
		Goal[i].Center.x = bb.x + (bb.width/2);
		Goal[i].Center.y = bb.y + (bb.height/2);
		Goal[i].Center.x = bb.x + (bb.width/2);
		Goal[i].Center.y = bb.y + (bb.height/2);
		Goal[i].xEdge = bb.width;
		Goal[i].yEdge = bb.height;
		Goal[i].GoalCount = sizeOfArray;
		Goal[i].PointOfAim = Map(Goal[i].Center.x,0,X_IMAGE_RES,-1,1);
		Goal[i].Distance = (Theight / (tan(bb.height * (VIEW_ANGLE_Y / Y_IMAGE_RES) * (3.14159 / 180))) - 4);
		Goal[i].Angle = acos(2.0 * (Goal[i].Distance * (tan(float(bb.width) * (float(VIEW_ANGLE_X) / float(X_IMAGE_RES)) * (3.14159 / float(180)))/2)) / float(Twidth / 2));
		printf("%i goal(s), Goal[%i] Center is: [%i,%i], POI is:[%f], Distance is:[%f], Angle is:[%f] \n", Goal[i].GoalCount, i+1, Goal[i].Center.x, Goal[i].Center.y, Goal[i].PointOfAim, Goal[i].Distance, Goal[i].Angle);

	}


}

void FindTargets(ShapeArray &input, Report Goal[], Mat &FilteredGoals)
{
	int sizeOfArray = int(input.size());

	//If two rectangles are found
	if(sizeOfArray <= 2)
	{
		for(int i=0; i<sizeOfArray; i++){

		Rect bb1 = boundingRect(input[i]);

		//Find out usefull stuff about the goals
		Goal[i].Center.x = bb1.x + (bb1.width/2);
		Goal[i].Center.y = bb1.y + (bb1.height/2);
		Goal[i].xEdge = bb1.width;
		Goal[i].yEdge = bb1.height;
		Goal[i].GoalCount = sizeOfArray;

		//printf("Goal %i, Goal Center is: [%i,%i] \n", Goal[i].GoalCount, Goal[i].Center.x, Goal[i].Center.y);

			//Finds only one goal
			if(Goal[i].GoalCount == 1)
			{
				printf("Only part of the target found \n");
				return;
			}

			//Finds two goal which is one target
			if(Goal[i].GoalCount == 2)
			{

				//Calculates the distance and the angle the target
				Goal[i].Distance = (Theight / (tan(bb1.height * (VIEW_ANGLE_Y / Y_IMAGE_RES) * (PI / 180))) - 4);
				Goal[i].Angle = acos(2.0 * (Goal[i].Distance * (tan(float(bb1.width) * (float(VIEW_ANGLE_X) / float(X_IMAGE_RES)) * (PI / float(180)))/2)) / float(Twidth / 2));
				printf("Found one target, Distance: [%f], Angle: [%f] \n", Goal[i].Distance, Goal[i].Angle);
				sendToNetworkTables(Goal[i].Distance, Goal[i].Angle);
				return;
			}

		}

	}

	//If there are more than two goals or one target
	else{
		printf("Too many goals \n");
		return;
	}
}



void sendToNetworkTables(double input1, double input2)
{
	return;
}



