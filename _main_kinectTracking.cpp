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

/* === Params === */

KinectConnector kin;

// Integral and derivative params
int started, prevError;
double inte, prev_error;
double kp, ki, kd, error_thresh;
double step_time, step_time_inv;
int setDist = 450;

/* === Util func === */

// FUNC : Initialize all PID params
void PID_Init() {
	// Gain for PID & Error threshold
	kp = 0.2, ki = 0.0, kd = 0.4;
	error_thresh = 50;
	// Controller step time
	step_time = 5;
	step_time_inv = 5 / step_time;
	// Initialize integral and derivative calculations
	started = 0;
	inte = 0;
}

// FUNC : Update actuator command
double PID_Update(int error) {

	if (error < -450) return -0.07575; // 0.101

									   // If error magnitude is below the threshold update the error integral
	inte += (fabs(error) < error_thresh) ? step_time * error : 0;

	// Compute the error derivative
	double deriv;
	if (!started) {
		deriv = 0;
		started = 1;
	}
	else {
		deriv = (error - prev_error) * step_time_inv;
	}
	// Store current error
	prev_error = error;

	return (((kp * error) + (ki * inte) + (kd * deriv)) / 300);
}

// FUNC : Get current error
int getError() {
	// 2 tiles between robot and wall
	int dist = setDist;

	// Image data
	Mat depthImg;
	Mat colorImg;
	Mat indexImg;
	Mat pointImg;

	kin.GrabData(depthImg, colorImg, indexImg, pointImg);

	int currentDist = depthImg.at<int>(140, 111) / 100000;

	return dist - currentDist;
}

// FUNC : Get x speed base on current error
double updateSpeed(int error) {
	int absError = fabs(error);
	if (absError > 350) return 0.1; // 0.2
	if (absError > 250) return 0.2; // 0.3
	return 0.3; // 0.4
}

// FUNC :

/* === Main === */
int main() {

	/* === Kinect init === */

	if (!kin.Connect()) {
		cout << "Kinect not connected!" << endl;
		kin = KinectConnector();
		return 1;
	}

	/* === Robot init === */

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

	/* === PID Init === */

	PID_Init();

	/* === Main loop === */

	int zeroTick = 0;

	while (true) {

		// Update PID and calculate speeds
		double vx, vz;
		int currentError = getError();
		if (prevError < -700 && currentError == setDist) {
			while (currentError == setDist) currentError = getError();
		}
		if (currentError == 450 && setDist == 450) {
			zeroTick++;
			if (zeroTick > 10) setDist = 300;
		}
		if (setDist == 300) {
			cout << "Distance reduced!" << endl;
			if (currentError < 0) {
				setDist = 450;
				zeroTick = 0;
			}
		}

		prevError = currentError;
		cout << currentError << " " << PID_Update(currentError) << endl;

		vx = updateSpeed(currentError);
		vz = PID_Update(currentError);

		if (vz >= 0.4) vz = 0.3; // 0.4
		if (vz <= -0.4) vz = -0.3; // 0.4
		if (vz == -0.07575) vx = 0.3; // 0.4

									  // Move robot
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
