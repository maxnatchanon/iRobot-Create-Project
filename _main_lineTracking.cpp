
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

bool isRecord = false;

/* === Util func === */

// FUNC : Check if inputed sensor is on the black tile or not
bool isBlack(CreateData	robotData, int sensorNumber) {
	// White threshold - 1000/1000/800/1300
	// Black threshold - 700/700/600/1100
	int blackValue[4];
	blackValue[0] = 700; blackValue[1] = 700; blackValue[2] = 600; blackValue[3] = 1100;
	if (robotData.cliffSignal[sensorNumber] < blackValue[sensorNumber]) return true;
	return false;
}

/* === Main === */

int main()
{
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

	/* === Params === */

	// For line tracker code
	// 0 = Idle , 1 = Black on left , 2 = Black on right , 3 = Else , 99 = Initial state
	int state = 99;

	// Switch mode - Line tracker(true) / Wall tracker(false)
	bool isLineTracker = false;

	/* === Main loop === */

	while (true)
	{
		double vx, vz;
		vx = vz = 0.0;

		char c = cvWaitKey(30);
		if (c == ' ') break;

		/* === Line tracker code === */

		if (isLineTracker) {
			// Get sensor data
			bool FL = isBlack(robotData, 1), FR = isBlack(robotData, 2);

			if (state == 99) {
				// Check initial sensor data then assign state
				if (FL && !FR) state = 1;
				else if (!FL && FR) state = 2;
				else state = 3;
			}
			if (state == 1) {
				vx = 1;
				vz = 0;
				// Turn back into the line 
				if (!FL) {
					vx = 1;
					vz = +0.6;
				}
				else if (FR) {
					vx = 1;
					vz = -0.6;
				}
			}
			else if (state == 2) {
				vx = 1;
				vz = 0;
				// Turn back into the line 
				if (!FR) {
					vx = 1;
					vz = -0.6;
				}
				else if (FL) {
					vx = 1;
					vz = +0.6;
				}
			}
			else if (state == 3) {
				// Move in a big circular motion to find the line
				vx = 0.75;
				vz = -0.2;
				if (FL && !FR) state = 1;
				else if (!FL && FR) state = 2;
			}
		}

		/* === Wall tracker code === */

		else {
			// cout << robotData.wallSignal << endl;
			
			int wallVal = robotData.wallSignal;

			// In range -> Go stright
			if (wallVal >= 20 && wallVal <= 50) {
				vx = 0.3;
				vz = 0;
			}
			// Too far -> Turn right
			else if (wallVal < 20 && wallVal >= 5) {
				vx = 0.3;
				vz = -0.1;
			}
			// Too close -> Turn left
			else if (wallVal > 50 && wallVal <= 100) {
				vx = 0.3;
				vz = 0.09;
			}
			// Else -> Turn around
			else {
				vx = 0;
				vz = -0.2;
			}
		}

		/* === Move robot === */
		
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

		int color = (abs(velL) + abs(velR)) / 4;
		color = (color < 0) ? 0 : (color > 255) ? 255 : color;

		int inten = (robotData.cliffSignal[1] + robotData.cliffSignal[2]) / 8 - 63;
		inten = (inten < 0) ? 0 : (inten > 255) ? 255 : inten;

		// cout << color << " " << inten << " " << robotData.cliffSignal[1] << " " << robotData.cliffSignal[2] << endl;
		// cout << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
		// [0] - Left

		// cout << robotData.bumper[0] << "\t" << robotData.bumper[1] << endl;
		// [0] - Left
		
		// cout << robotData.infrared << endl;
		// cout << robotData.wallSignal << endl;

		robot.LEDs(velL > 0, velR > 0, color, inten);

		if (!robot.DriveDirect(velL, velR))
			cout << "SetControl Fail" << endl;

		if (!robot.ReadData(robotData))
			cout << "ReadData Fail" << endl;

		if (isRecord)
			record << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
	}

	robot.Disconnect();

	return 0;
}