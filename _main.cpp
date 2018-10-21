#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <KinectConnector.h>
#include <math.h>
#include "SerialPort.h"
#include "cv.h"
#include "highgui.h"
#include "RobotConnector.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace std;
using namespace cv;

#define Create_Comport "COM3"

KinectConnector kin;

int main() {

	if (!kin.Connect()) {
		cout << "Kinect not connected!" << endl;
		kin = KinectConnector();
		return 1;
	}

	CreateData	robotData;
	RobotConnector	robot;

	ofstream	record;

	record.open("../data/robot.txt");

	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");

	while (true) {
		char c = cvWaitKey(30);
		if (c == ' ') break;

		switch (c)
		{
		case 'w': vx = +1; break;
		case 's': vx = -1; break;
		case 'a': vz = +1; break;
		case 'd': vz = -1; break;
		case ' ': vx = vz = 0; break;
		case 'c': robot.Connect(Create_Comport); break;
		}

		double vl = vx - vz;
		double vr = vx + vz;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		if (!robot.DriveDirect(velL, velR))
			cout << "SetControl Fail" << endl;

		if (!robot.ReadData(robotData))
			cout << "ReadData Fail" << endl;
	}

	robot.Disconnect();

	return 0;
}
