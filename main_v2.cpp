/****************************************************************************\
* Copyright (C) 2017 Infineon Technologies & pmdtechnologies ag
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#include <royale.hpp>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

// OpenCV 4.1.0
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// OpenCV 3.1.0
//#include <opencv/cv.h>
//#include <opencv/cxcore.h>
//#include <opencv/highgui.h>
//

//Kinova::Header
#include <Windows.h>
#include "kinova/CommunicationLayer.h"
#include "kinova/CommandLayer.h"
#include <conio.h>
#include "kinova/KinovaTypes.h"
//Kinova::Header [end]

#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

//Kinova API
//A handle to the API.
HINSTANCE commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetAngularCommand)(AngularPosition &);
int(*MyGetCartesianCommand)(CartesianPosition &);
int(*MyStartForceControl)();
int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
//Kinova API End

int imagetype = -1;
Vec3f d_XYZt_Kinova = { 0,0,0 };

#define PI 3.141592

class MyListener : public IDepthDataListener
{

public:

	MyListener() : undistortImage(false)
	{
	}

	void onNewData(const DepthData *data)
	{
		// this callback function will be called for every new depth frame

		std::lock_guard<std::mutex> lock(flagMutex);

		float *zRowPtr, *grayRowPtr = NULL;
		// zImage
		zImage.create(Size(data->width, data->height), CV_32FC1);
		zImage = Scalar::all(0); // set the image to zero

		int k = 0;
		for (int y = 0; y < zImage.rows; y++)
		{
			zRowPtr = zImage.ptr<float>(y);

			for (int x = 0; x < zImage.cols; x++, k++)
			{
				auto curPoint = data->points.at(k);
				if (curPoint.depthConfidence > 0)
				{
					// if the point is valid, map the pixel from 3D world
					// coordinates to a 2D plane (this will distort the image)					
					zRowPtr[x] = adjustZValue(curPoint.z);
				}
			}
		}
		zImage8.create(Size(data->width, data->height), CV_8UC1);
		zImage.convertTo(zImage8, CV_8UC1);		// normalize(zImage, zImage8, 0, 255, NORM_MINMAX, CV_8UC1)

		if (undistortImage)
		{
			// call the undistortion function on the z image
			Mat temp = zImage8.clone();
			undistort(temp, zImage8, cameraMatrix, distortionCoefficients);
		}

		scaledZImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);		// scale and display the depth image
		resize(zImage8, scaledZImage, scaledZImage.size());

		imshow ("Depth", scaledZImage);
		imshow("~Depth", ~scaledZImage);

		// grayImage
		grayImage.create(Size(data->width, data->height), CV_32FC1);
		grayImage = Scalar::all(0); // set the image to zero

		k = 0;
		for (int y = 0; y < grayImage.rows; y++)
		{
			grayRowPtr = grayImage.ptr<float>(y);

			for (int x = 0; x < grayImage.cols; x++, k++)
			{
				auto curPoint = data->points.at(k);
				if (curPoint.depthConfidence > 0)
				{
					// if the point is valid, map the pixel from 3D world
					// coordinates to a 2D plane (this will distort the image)
					grayRowPtr[x] = adjustGrayValue(curPoint.grayValue);
				}
			}
		}

		grayImage8.create(Size(data->width, data->height), CV_8UC1);
		grayImage.convertTo(grayImage8, CV_8UC1);		// normalize(grayImage, grayImage8, 0, 255, NORM_MINMAX, CV_8UC1)

		if (undistortImage)
		{
			// call the undistortion function on the gray image
			Mat temp = grayImage8.clone();
			undistort(temp, grayImage8, cameraMatrix, distortionCoefficients);
		}

		// scale and display the gray image
		scaledGrayImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
		resize(grayImage8, scaledGrayImage, scaledGrayImage.size());

		imshow("Gray", scaledGrayImage);

		imagetype = 0;
		Vec4f pOutLine;
		result_ZImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);		
		overlay_Bounding_Box(scaledZImage, data, result_ZImage, &pOutLine);
		Kinova_calib(&pOutLine, &d_XYZt_Kinova);
		imshow("Test_Z", result_ZImage);

		imagetype = 1;		
		result_GImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
		overlay_Bounding_Box(scaledGrayImage, data, result_GImage, &pOutLine);
		Kinova_calib(&pOutLine, &d_XYZt_Kinova);
		imshow("Test_G", result_GImage);


	}


	void Kinova_calib(Vec4f* OutLine, Vec3f *d_XYZt_Kinova) {
		// Kinova Gripper Initial : (Z=0, theta=0) ===>> pico : (X =490/960, Y=220/720), real_world : (0.055 m, 0.015 m)
		float dx = ((*OutLine)[2] *-0.034 + 15)*0.01;
		float dy = ((*OutLine)[3] *-0.03 + 7)*0.01;
		float dth = atan((float)(*OutLine)[1] / (float)(*OutLine)[0]) - PI/2.0f;

		*d_XYZt_Kinova = { dx, dy, dth };
		
		//cout << " Out : " << dx << ", " << dy << ", " << dth << endl;
		//cout << " OutLine : " << *OutLine << endl;
		//cout << " d_Kinova : " << *d_XYZt_Kinova << endl;
	}


	void overlay_Bounding_Box(Mat tImage, const DepthData *data, Mat result_tImage, Vec4f *pOutLine) {

		normalize(tImage, tImage8, 0, 255, NORM_MINMAX, CV_8UC1);
		
		if (imagetype == 0) { // Z image
			cv::threshold(~tImage8, tImage8, 170, 255, cv::THRESH_TOZERO + cv::THRESH_OTSU);// +cv::THRESH_BINARY);
			//Adaptive ThresholdingÀ» ÇÑ´Ù.
			//adaptiveThreshold(tImage8, tImage8, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 10);
			//tImage8 = ~tImage8;
		}
		else if (imagetype == 1) { // Gray image
			cv::threshold(tImage8, tImage8, 127, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);// +cv::THRESH_BINARY);
		}
		cv::GaussianBlur(tImage8, tImage8, Size(5, 5), 1, 1, 1);
		Mat tImageMat = tImage8.clone();

		double area, max_area = 0;
		int max_id = 0;
		Rect bounding_rect;
		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours;
		findContours(tImageMat, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++) {
			area = contourArea(contours[i], false);
			if (area > max_area) {
				max_area = area;
				max_id = i;
				bounding_rect = boundingRect(contours[i]);
			}
		}
		/************/
		// cout << "N of contours are " << contours.size() << ", and the max_area is " << max_id << "th contour with area" << max_area << endl;

		//drawContours(tImage8, contours, max_id, Scalar(50, 50, 50), CV_FILLED, 1, hierarchy); // Hand shape contour display
		rectangle(tImage8, bounding_rect, Scalar(100, 100, 100), 1, 0);

		//// Largest area convexhull display
		//vector<Point> hull;
		//vector<int> hull_sI;
		//convexHull(Mat(contours[max_id]), hull, false);
		//convexHull(Mat(contours[max_id]), hull_sI, false);

		//drawContours(tImage8, vector<vector<Point>> {hull}, 0, Scalar(50, 50, 50), 1, 8);

		//vector<Vec4i> defects;
		//if (hull.size() > 3) {
		//	convexityDefects(Mat(contours[max_id]), hull_sI, defects);
		//}

		//for (int i = 0; i < defects.size(); i++) {
		//	const Vec4i& v = defects[i];
		//	float depth = v[3] / 256;
		//	if (depth > 10) //  filter defects by depth
		//	{
		//		int startidx = v[0]; Point ptStart(contours[max_id][startidx]);
		//		int endidx = v[1]; Point ptEnd(contours[max_id][endidx]);
		//		int faridx = v[2]; Point ptFar(contours[max_id][faridx]);

		//		line(tImage8, ptEnd, ptStart, Scalar(0, 255, 0), 1);
		//		line(tImage8, ptFar, ptStart, Scalar(0, 255, 0), 1);
		//		line(tImage8, ptFar, ptEnd, Scalar(0, 255, 0), 1);
		//		circle(tImage8, ptFar, 4, Scalar(0, 255, 0), 2);
		//	}
		//}		

		Vec4f OutLine;
		fitLine(Mat(contours[max_id]), OutLine, DIST_L2, 0, 0.01, 0.01);
		/****************/
		//cout << "OutLine Data" << OutLine << endl;
		line(tImage8, Point(OutLine[2],OutLine[3]), Point(100 * OutLine[0] + OutLine[2], 100 * OutLine[1] + OutLine[3]), Scalar(50, 50, 50), 2);
		//line(tImage, Point(0,0), Point(200,200), Scalar(50, 50, 50), 2);
		circle(tImage8, Point(OutLine[2], OutLine[3]), 4, Scalar(0, 255, 0), 2);
		circle(tImage8, Point(20 * OutLine[0] + OutLine[2], 20 * OutLine[1] +OutLine[3]), 4, Scalar(0, 255, 0), 2);

		//Sleep(1);		
		/*scaledtImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
		resize(tImage8, scaledtImage, scaledtImage.size());
		imshow("Test", scaledtImage);*/
		*pOutLine = OutLine;
		
		resize(tImage8, result_tImage, result_tImage.size());
	}

	void setLensParameters(const LensParameters &lensParameters)
	{
		// Construct the camera matrix
		// (fx   0    cx)
		// (0    fy   cy)
		// (0    0    1 )
		cameraMatrix = (Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
			0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
			0, 0, 1);

		// Construct the distortion coefficients
		// k1 k2 p1 p2 k3
		distortionCoefficients = (Mat1d(1, 5) << lensParameters.distortionRadial[0],
			lensParameters.distortionRadial[1],
			lensParameters.distortionTangential.first,
			lensParameters.distortionTangential.second,
			lensParameters.distortionRadial[2]);
	}

	void toggleUndistort()
	{
		std::lock_guard<std::mutex> lock(flagMutex);
		undistortImage = !undistortImage;
	}

private:

	// adjust z value to fit fixed scaling, here max dist is 2.5m
	// the max dist here is used as an example and can be modified
	float adjustZValue(float zValue)
	{
		float clampedDist = std::min(2.5f, zValue);
		float newZValue = clampedDist / 2.5f * 255.0f;
		return newZValue;
	}

	// adjust gray value to fit fixed scaling, here max value is 180
	// the max value here is used as an example and can be modified
	float adjustGrayValue(uint16_t grayValue)
	{
		float clampedVal = std::min(2350.0f, grayValue * 1.0f); // 180.0f (Original clamp Value)
		float newGrayValue = clampedVal / 2350.f * 255.0f;
		return newGrayValue;
	}

	// define images for depth and gray
	// and for their 8Bit and scaled versions
	Mat zImage, zImage8, scaledZImage, result_ZImage;
	Mat grayImage, grayImage8, scaledGrayImage, result_GImage;

	// 2018Oct27 YJ
	Mat tImage, tImage8, scaledtImage; //Test image

									   // lens matrices used for the undistortion of
									   // the image
	Mat cameraMatrix;
	Mat distortionCoefficients;

	std::mutex flagMutex;
	bool undistortImage;
};

int main(int argc, char *argv[])
{
	// Kinova JACO2 Initialization
	commandLayer_handle = LoadLibrary("CommandLayerWindows.dll");

	//Initialise the function pointer from the API
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
	MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) GetProcAddress(commandLayer_handle, "RunGravityZEstimationSequence");

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetAngularCommand == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL))

	{
		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		_getch();
		return 0;
	}

	std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

	int result = (*MyInitAPI)();

	AngularPosition currentCommand;
	
	std::cout << "Initialization's result :" << result << endl;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);
	std::cout << "Device Count : " << devicesCount << endl;

	if (devicesCount > 0) {
		std::cout << "Initializing Right Arm (S/N : " << list[0].SerialNumber << ")" << endl;
		MySetActiveDevice(list[0]);
		std::cout << "Send the robot to HOME position" << endl;
		MyMoveHome();
		Sleep(2000);
		std::cout << "Initializing the fingers" << endl;
		MyInitFingers();
		std::cout << "Move to Intial task position" << endl;

		TrajectoryPoint pointToSend_C;
		CartesianPosition currentCommand_C;

		pointToSend_C.InitStruct();
		pointToSend_C.Position.Type = CARTESIAN_POSITION;

		pointToSend_C.Position.CartesianPosition.X = -0.085;
		pointToSend_C.Position.CartesianPosition.Y = -0.416;
		pointToSend_C.Position.CartesianPosition.Z = -0.095;
		pointToSend_C.Position.CartesianPosition.ThetaX = -3.085;
		pointToSend_C.Position.CartesianPosition.ThetaY = 0.005;
		pointToSend_C.Position.CartesianPosition.ThetaZ = 0.073;

		MySendBasicTrajectory(pointToSend_C);

	}
	else {
		std::cout << "No Arm Detected." << endl;
		//return 1;
	}

	// Pico Flexx Initialization
	PlatformResources resources;
	MyListener listener;
	std::unique_ptr<ICameraDevice> cameraDevice;
	{
		CameraManager manager;
		
		royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
		cout << "Detected " << camlist.size() << " camera(s)." << endl;

		if (!camlist.empty())
		{
			cameraDevice = manager.createCamera(camlist[0]);
		}
		else
		{
			cerr << "No suitable camera device detected." << endl;
			return 1;
		}

		camlist.clear();
	}

	if (cameraDevice == nullptr)
	{
		// no cameraDevice available
		cerr << "Cannot create the camera device" << endl;
		return 1;
	}

	// IMPORTANT: call the initialize method before working with the camera device
	auto status = cameraDevice->initialize();
	if (status != CameraStatus::SUCCESS)
	{
		cerr << "Cannot initialize the camera device, error string : " << getErrorString(status) << endl;
		return 1;
	}

	royale::String usecaseName = "MODE_5_35FPS_600"; // MODE_9_5FPS_2000,		MODE_9_10FPS_1000,		MODE_9_15FPS_700,		MODE_9_25FPS_450,
													// MODE_5_35FPS_600,		MODE_5_45FPS_500,		MODE_MIXED_30_5

	status = cameraDevice->setUseCase(usecaseName);
	if (status == royale::CameraStatus::SUCCESS) {
		cout << " The use cases is successfully adapted to " << usecaseName << endl;
	}
	else {
		cout << "Fail to set use case." << endl;
	}

	// retrieve the lens parameters from Royale
	LensParameters lensParameters;
	status = cameraDevice->getLensParameters(lensParameters);
	if (status != CameraStatus::SUCCESS)
	{
		cerr << "Can't read out the lens parameters" << endl;
		return 1;
	}

	listener.setLensParameters(lensParameters);

	// register a data listener
	if (cameraDevice->registerDataListener(&listener) != CameraStatus::SUCCESS)
	{
		cerr << "Error registering data listener" << endl;
		return 1;
	}

	// create windows
	namedWindow("Depth", WINDOW_AUTOSIZE);
	namedWindow("Gray", WINDOW_AUTOSIZE);
	namedWindow("Test_Z", WINDOW_AUTOSIZE);
	namedWindow("Test_G", WINDOW_AUTOSIZE);
	namedWindow("~Depth", WINDOW_AUTOSIZE);

	// start capture mode
	if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
	{
		cerr << "Error starting the capturing" << endl;
		return 1;
	}

	int currentKey = 0;
	int moving_flag = 0; // Once 'm' is entered change to 1 keep moving until dist_err < 0.01 (1cm)
	int countloop = 0; // Check loop number for one 'm' command
	CartesianPosition current_XYZ;

	/*TrajectoryPoint pointToSend_XYZ;
	pointToSend_XYZ.InitStruct();
	pointToSend_XYZ.Position.Type = CARTESIAN_VELOCITY;*/

	while (currentKey != 27)
	{				
		// wait until a key is pressed
		currentKey = waitKey(1) & 255;		
		MyGetCartesianCommand(current_XYZ);
		//cout << "Gripper XYZ t :" << current_XYZ.Coordinates.X << ", " << current_XYZ.Coordinates.Y << ", " << current_XYZ.Coordinates.Z << ", " << current_XYZ.Coordinates.ThetaZ << endl;
		//cout << "Kinova Calib :" << d_XYZt_Kinova[0] << ", " << d_XYZt_Kinova[1] << ", " << d_XYZt_Kinova[2] << endl << endl;
		
		float dx = d_XYZt_Kinova[0];
		float dy = d_XYZt_Kinova[1];
		float dist_err = sqrt(dx*dx + dy*dy);
		float dth = d_XYZt_Kinova[2];
				
		if (moving_flag == 1 & dist_err > 0.01) {
			printf("**** Distance Error is %6.3f. Keep Moving to the Object. \n", dist_err);
			currentKey = 'm';
			countloop++;
		} else if (moving_flag == 1 & dist_err <= 0.01) {
			moving_flag = 0;
			countloop = 0;
			printf("**** Distance Error is %6.3f. STOP Moving. \n", dist_err);
			currentKey = 'r';
		}

		if (currentKey == 'm')
		{
			if (countloop > 5) {
				printf("Distance Error Standard cannot be reached. STOP Moving.\n");
				moving_flag = 0;
				countloop = 0;
				continue;
			}

			if (dist_err > 0.01) {
				cout << "!!!!!!!!!!!!!!Move!!!!!!!!!!!!!!" << endl;
				moving_flag = 1;
				TrajectoryPoint pointToSend;
				CartesianPosition currentCommand;
				MyGetCartesianCommand(currentCommand);
				pointToSend.InitStruct();
				pointToSend.Position.Type = CARTESIAN_POSITION;
				pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X - dx;
				pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y + dy;
				pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
				pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
				pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
				pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;
				MySendBasicTrajectory(pointToSend);
				Sleep(1000);
			}
			else {
				printf("**** Distance Error is %6.3f. STOP Moving. \n", dist_err);

				moving_flag = 0;				
				countloop = 0;
			}
		}
		else if (currentKey == 'r') {
			printf("**** Rotate Hand to Orientation. Start Moving. \n", dist_err);
			TrajectoryPoint pointToSend;
			CartesianPosition currentCommand;
			MyGetCartesianCommand(currentCommand);
			dth = currentCommand.Coordinates.ThetaZ - dth;
			pointToSend.InitStruct();
			pointToSend.Position.Type = CARTESIAN_POSITION;
			pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
			pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y;
			pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
			pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
			pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
			pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ - dth;
			MySendBasicTrajectory(pointToSend);
			Sleep(1000);

			/*std::cout << "Move to Grasping position" << endl;

			TrajectoryPoint pointToSend_C;
			CartesianPosition currentCommand_C;

			MyGetCartesianCommand(currentCommand_C);

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = currentCommand_C.Coordinates.X;
			pointToSend_C.Position.CartesianPosition.Y = currentCommand_C.Coordinates.Y;
			pointToSend_C.Position.CartesianPosition.Z = -0.22;
			pointToSend_C.Position.CartesianPosition.ThetaX = currentCommand_C.Coordinates.ThetaX;
			pointToSend_C.Position.CartesianPosition.ThetaY = currentCommand_C.Coordinates.ThetaY;
			pointToSend_C.Position.CartesianPosition.ThetaZ = currentCommand_C.Coordinates.ThetaZ;

			MySendBasicTrajectory(pointToSend_C);
			Sleep(5000);
			std::cout << "Start Grasping" << endl;
			MyInitFingers();
			MyGetCartesianCommand(currentCommand_C);

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = currentCommand_C.Coordinates.X;
			pointToSend_C.Position.CartesianPosition.Y = currentCommand_C.Coordinates.Y;
			pointToSend_C.Position.CartesianPosition.Z = currentCommand_C.Coordinates.Z;
			pointToSend_C.Position.CartesianPosition.ThetaX = currentCommand_C.Coordinates.ThetaX;
			pointToSend_C.Position.CartesianPosition.ThetaY = currentCommand_C.Coordinates.ThetaY;
			pointToSend_C.Position.CartesianPosition.ThetaZ = currentCommand_C.Coordinates.ThetaZ;

			pointToSend_C.Position.Fingers.Finger1 = 5000;
			pointToSend_C.Position.Fingers.Finger2 = 5000;
			pointToSend_C.Position.Fingers.Finger3 = 5000;

			MySendBasicTrajectory(pointToSend_C);
			Sleep(2000);

			std::cout << "Start Lifting" << endl;
			MyInitFingers();
			MyGetCartesianCommand(currentCommand_C);

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = currentCommand_C.Coordinates.X;
			pointToSend_C.Position.CartesianPosition.Y = currentCommand_C.Coordinates.Y;
			pointToSend_C.Position.CartesianPosition.Z = 0.0;
			pointToSend_C.Position.CartesianPosition.ThetaX = currentCommand_C.Coordinates.ThetaX;
			pointToSend_C.Position.CartesianPosition.ThetaY = currentCommand_C.Coordinates.ThetaY;
			pointToSend_C.Position.CartesianPosition.ThetaZ = currentCommand_C.Coordinates.ThetaZ;

			pointToSend_C.Position.Fingers.Finger1 = 5000;
			pointToSend_C.Position.Fingers.Finger2 = 5000;
			pointToSend_C.Position.Fingers.Finger3 = 5000;

			MySendBasicTrajectory(pointToSend_C);
			Sleep(2000);*/

		}
		else if (currentKey == 'h')
		{
			std::cout << "Initializing the fingers" << endl;
			MyInitFingers();
			std::cout << "Move to Intial task position" << endl;

			TrajectoryPoint pointToSend_C;
			CartesianPosition currentCommand_C;

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = -0.085;
			pointToSend_C.Position.CartesianPosition.Y = -0.416;
			pointToSend_C.Position.CartesianPosition.Z = -0.095;
			pointToSend_C.Position.CartesianPosition.ThetaX = -3.085;
			pointToSend_C.Position.CartesianPosition.ThetaY = 0.005;
			pointToSend_C.Position.CartesianPosition.ThetaZ = 0.073;

			MySendBasicTrajectory(pointToSend_C);
		}	
		else if (currentKey == 'd') {
			listener.toggleUndistort();
		}
		else if (currentKey == 'g') {
						
			std::cout << "Move to Grasping position" << endl;

			TrajectoryPoint pointToSend_C;
			CartesianPosition currentCommand_C;

			MyGetCartesianCommand(currentCommand_C);

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = currentCommand_C.Coordinates.X;
			pointToSend_C.Position.CartesianPosition.Y = currentCommand_C.Coordinates.Y;
			pointToSend_C.Position.CartesianPosition.Z = -0.33;
			pointToSend_C.Position.CartesianPosition.ThetaX = currentCommand_C.Coordinates.ThetaX;
			pointToSend_C.Position.CartesianPosition.ThetaY = currentCommand_C.Coordinates.ThetaY;
			pointToSend_C.Position.CartesianPosition.ThetaZ = currentCommand_C.Coordinates.ThetaZ;

			
			cout << currentCommand_C.Coordinates.Z << endl;
			MySendBasicTrajectory(pointToSend_C);
			Sleep(2000);
			std::cout << "Start Grasping" << endl;
			MyInitFingers();
			MyGetCartesianCommand(currentCommand_C);

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = currentCommand_C.Coordinates.X;
			pointToSend_C.Position.CartesianPosition.Y = currentCommand_C.Coordinates.Y;
			pointToSend_C.Position.CartesianPosition.Z = currentCommand_C.Coordinates.Z;
			pointToSend_C.Position.CartesianPosition.ThetaX = currentCommand_C.Coordinates.ThetaX;
			pointToSend_C.Position.CartesianPosition.ThetaY = currentCommand_C.Coordinates.ThetaY;
			pointToSend_C.Position.CartesianPosition.ThetaZ = currentCommand_C.Coordinates.ThetaZ;

			pointToSend_C.Position.Fingers.Finger1 = 5000;
			pointToSend_C.Position.Fingers.Finger2 = 5000;
			pointToSend_C.Position.Fingers.Finger3 = 5000;

			MySendBasicTrajectory(pointToSend_C);
			Sleep(2000);

			std::cout << "Start Lifting" << endl;
			MyInitFingers();
			MyGetCartesianCommand(currentCommand_C);

			pointToSend_C.InitStruct();
			pointToSend_C.Position.Type = CARTESIAN_POSITION;

			pointToSend_C.Position.CartesianPosition.X = currentCommand_C.Coordinates.X;
			pointToSend_C.Position.CartesianPosition.Y = currentCommand_C.Coordinates.Y;
			pointToSend_C.Position.CartesianPosition.Z = 0.0;
			pointToSend_C.Position.CartesianPosition.ThetaX = currentCommand_C.Coordinates.ThetaX;
			pointToSend_C.Position.CartesianPosition.ThetaY = currentCommand_C.Coordinates.ThetaY;
			pointToSend_C.Position.CartesianPosition.ThetaZ = currentCommand_C.Coordinates.ThetaZ;

			pointToSend_C.Position.Fingers.Finger1 = 5000;
			pointToSend_C.Position.Fingers.Finger2 = 5000;
			pointToSend_C.Position.Fingers.Finger3 = 5000;

			MySendBasicTrajectory(pointToSend_C);
			Sleep(2000);

		}
	}

	// stop capture mode
	if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
	{
		cerr << "Error stopping the capturing" << endl;
		return 1;
	}
	cout << endl << "C L O S I N G   A P I" << endl;
	result = (*MyCloseAPI)();
	return 0;
}
