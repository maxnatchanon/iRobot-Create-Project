
#ifndef _ROBOTCONNECTOR_H_
#define _ROBOTCONNECTOR_H_

#include "SerialPort.h"
#include "CreateData.h"

#define Create_MaxVel	500

class RobotConnector
{
public :
	bool Connect(const char *port = "COM4");
	void Disconnect();
	bool ReadData(CreateData &data);
	bool DriveDirect(int velLeft, int velRight);
	bool LEDs(bool advLed, bool playLed, int color, int intensity);

private:
	SerialPort serial;
	char _buff[MAX_PATH];

	CreateData	_data;

	bool RequestData();
	bool Beep();
};
#endif // _ROBOTCONNECTOR_H_