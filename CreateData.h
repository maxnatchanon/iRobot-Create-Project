
#ifndef _CREATEDATA_H_
#define _CREATEDATA_H_

#include <iostream>
#include <fstream>

using namespace std;

class CreateData
{
public :
	// Package 1 : 7-16
	bool bumper[2];			// Left[0], Right[1]
	bool wheeldrop[3];		// Left[0], Caster[1], Right[2]
	bool wall;				// Wall
	bool cliff[4];			// Left[0], FrontLeft[1], FrontRight[2], Right[3]
	bool virtualWall;		// Virtual Wall
	bool overcurrent[2];	// Left[0], Right[1]
	bool lowsideDriver[3];	// LD[0], LD[1], LD[2]

	// Package 2 : 17-20
	int infrared;			// Infrared type
	bool button[2];			// Play[0], Advance[1]
	int distance;			// Distance
	int angle;				// Angle
	
	// Package 3 : 21-26
	int chargingSate;		// Charging State
	int voltage;			// Voltage
	int current;			// Current
	int battTemp;			// Battery Temperature
	int battCharge;			// Battery Charge
	int battCap;			// Battery Capacity

	// Package 4 : 27-34
	int wallSignal;
	int cliffSignal[4];		// Left[0], FrontLeft[1], FrontRight[2], Right[3]
	bool digitalInput[5];	// Pin[15], Pin[6], Pin[18], Pin[5], Pin[17]
	int analogInput;
	bool charger[2];		// HomeBase[0], Interal[1]
	
	// Package 5 : 35-42
	int oiMode;
	int songNumber;
	bool songPlaying;
	int streamNumber;
	int requestVelocity;
	int requestRadius;
	int requestVelLeft;
	int requestVelRight;

	CreateData()
	{
		memset(this, 0, sizeof(CreateData));
	}

	void Copy(CreateData &src)
	{
		//Package1:7-16
		bumper[0]			= src.bumper[0];
		bumper[1]			= src.bumper[1];
		wheeldrop[0]		= src.wheeldrop[0];
		wheeldrop[1]		= src.wheeldrop[1];
		wheeldrop[2]		= src.wheeldrop[2];
		wall				= src.wall;
		cliff[0]			= src.cliff[0];
		cliff[1]			= src.cliff[1];
		cliff[2]			= src.cliff[2];
		cliff[3]			= src.cliff[3];
		virtualWall			= src.virtualWall;
		overcurrent[0]		= src.overcurrent[0];
		overcurrent[1]		= src.overcurrent[1];
		lowsideDriver[0]	= src.lowsideDriver[0];
		lowsideDriver[1]	= src.lowsideDriver[1];
		lowsideDriver[2]	= src.lowsideDriver[2];
							
		//Package2:17-20
		infrared			= src.infrared;
		button[0]			= src.button[0];
		button[1]			= src.button[1];
		distance			= src.distance;
		angle				= src.angle;
							
		//Package3:21-26
		chargingSate		= src.chargingSate;
		voltage				= src.voltage;
		current				= src.current;
		battTemp			= src.battTemp;
		battCharge			= src.battCharge;
		battCap				= src.battCap;
							
		//Package4:27-34
		wallSignal			= src.wallSignal;
		cliffSignal[0]		= src.cliffSignal[0];
		cliffSignal[1]		= src.cliffSignal[1];
		cliffSignal[2]		= src.cliffSignal[2];
		cliffSignal[3]		= src.cliffSignal[3];
		digitalInput[0]		= src.digitalInput[0];
		digitalInput[1]		= src.digitalInput[1];
		digitalInput[2]		= src.digitalInput[2];
		digitalInput[3]		= src.digitalInput[3];
		digitalInput[4]		= src.digitalInput[4];
		analogInput			= src.analogInput;
		charger[0]			= src.charger[0];
		charger[1]			= src.charger[1];
							
		//Package5:35-42
		oiMode				= src.oiMode;
		songNumber			= src.songNumber;
		songPlaying			= src.songPlaying;
		streamNumber		= src.streamNumber;
		requestVelocity		= src.requestVelocity;
		requestRadius		= src.requestRadius;
		requestVelLeft		= src.requestVelLeft;
		requestVelRight		= src.requestVelRight;
	}

	friend istream& operator >> (istream &is, CreateData &m) 
	{
		//Package1:7-16
		is >> m.bumper[0];
		is >> m.bumper[1];
		is >> m.wheeldrop[0];
		is >> m.wheeldrop[1];
		is >> m.wheeldrop[2];
		is >> m.wall;
		is >> m.cliff[0];
		is >> m.cliff[1];
		is >> m.cliff[2];
		is >> m.cliff[3];
		is >> m.virtualWall;
		is >> m.overcurrent[0];
		is >> m.overcurrent[1];
		is >> m.lowsideDriver[0];
		is >> m.lowsideDriver[1];
		is >> m.lowsideDriver[2];

		//Package2:17-20
		is >> m.infrared;
		is >> m.button[0];
		is >> m.button[1];
		is >> m.distance;
		is >> m.angle;
			  
		//Package3:21-26
		is >> m.chargingSate;
		is >> m.voltage;
		is >> m.current;
		is >> m.battTemp;
		is >> m.battCharge;
		is >> m.battCap;

		//Package4:27-34
		is >> m.wallSignal;
		is >> m.cliffSignal[0];
		is >> m.cliffSignal[1];
		is >> m.cliffSignal[2];
		is >> m.cliffSignal[3];
		is >> m.digitalInput[0];
		is >> m.digitalInput[1];
		is >> m.digitalInput[2];
		is >> m.digitalInput[3];
		is >> m.digitalInput[4];
		is >> m.analogInput;
		is >> m.charger[0];
		is >> m.charger[1];

		//Package5:35-42
		is >> m.oiMode;
		is >> m.songNumber;
		is >> m.songPlaying;
		is >> m.streamNumber;
		is >> m.requestVelocity;
		is >> m.requestRadius;
		is >> m.requestVelLeft;
		is >> m.requestVelRight;
			 
		return is;
	}

	friend ostream& operator << (ostream &os, CreateData &m) 
	{
		//Package1:7-16
		os << m.bumper[0] << " ";
		os << m.bumper[1] << " ";
		os << m.wheeldrop[0] << " ";
		os << m.wheeldrop[1] << " ";
		os << m.wheeldrop[2] << " ";
		os << m.wall << " ";
		os << m.cliff[0] << " ";
		os << m.cliff[1] << " ";
		os << m.cliff[2] << " ";
		os << m.cliff[3] << " ";
		os << m.virtualWall << " ";
		os << m.overcurrent[0] << " ";
		os << m.overcurrent[1] << " ";
		os << m.lowsideDriver[0] << " ";
		os << m.lowsideDriver[1] << " ";
		os << m.lowsideDriver[2] << " ";

		//Package2:17-20
		os << m.infrared << " ";
		os << m.button[0] << " ";
		os << m.button[1] << " ";
		os << m.distance << " ";
		os << m.angle << " ";

		//Package3:21-26
		os << m.chargingSate << " ";
		os << m.voltage << " ";
		os << m.current << " ";
		os << m.battTemp << " ";
		os << m.battCharge << " ";
		os << m.battCap << " ";

		//Package4:27-34
		os << m.wallSignal << " ";
		os << m.cliffSignal[0] << " ";
		os << m.cliffSignal[1] << " ";
		os << m.cliffSignal[2] << " ";
		os << m.cliffSignal[3] << " ";
		os << m.digitalInput[0] << " ";
		os << m.digitalInput[1] << " ";
		os << m.digitalInput[2] << " ";
		os << m.digitalInput[3] << " ";
		os << m.digitalInput[4] << " ";
		os << m.analogInput << " ";
		os << m.charger[0] << " ";
		os << m.charger[1] << " ";

		//Package5:35-42
		os << m.oiMode << " ";
		os << m.songNumber << " ";
		os << m.songPlaying << " ";
		os << m.streamNumber << " ";
		os << m.requestVelocity << " ";
		os << m.requestRadius << " ";
		os << m.requestVelLeft << " ";
		os << m.requestVelRight << " ";

		return os;
	}
};
#endif // _CREATEDATA_H_
