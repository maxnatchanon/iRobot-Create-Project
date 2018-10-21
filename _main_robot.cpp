
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

bool isRecord = false;

int main()
{
	CreateData	robotData;
	RobotConnector	robot;

	ofstream	record;
	record.open("../data/robot.txt");

	if( !robot.Connect(Create_Comport) )
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");


	while(true)
	{
		char c = cvWaitKey(30);
		if( c == 27 ) break;
	
		double vx, vz;
		vx = vz = 0.0;

		switch(c)
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

		int color = (abs(velL)+abs(velR))/4;
		color = (color < 0) ? 0 : (color > 255) ? 255 : color;

		int inten = (robotData.cliffSignal[1] + robotData.cliffSignal[2])/8 - 63;
		inten = (inten < 0) ? 0 : (inten > 255) ? 255 : inten;

		//cout << color << " " << inten << " " << robotData.cliffSignal[1] << " " << robotData.cliffSignal[2] << endl;

		robot.LEDs(velL > 0, velR > 0, color, inten);
		
		if( !robot.DriveDirect(velL, velR) )
			cout << "SetControl Fail" << endl;

		if( !robot.ReadData(robotData) )
			cout << "ReadData Fail" << endl;

		if( isRecord )
			record << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
		
		cout << "Robot " << robotData.infrared << endl;
	}

	robot.Disconnect();

	return 0;
}