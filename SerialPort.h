
#ifndef _SERIALPORT_H_
#define _SERIALPORT_H_

#include <windows.h>

// load in the multimedia library
#ifdef _MSC_VER
#pragma comment(lib,"winmm.lib")
#endif

class SerialPort  
{
public:
	void Set_RTS_State(bool state);
	void Set_DTR_State(bool state);
	bool Get_RI_State();
	bool Get_DSR_State();
	bool Get_CTS_State();
	bool Get_CD_State();
	bool ChangeBaudRate (int buadRate);	
	bool SetHardwareControl(bool hardwareControl);
	bool Write(const char* Buffer, unsigned long BufferSize);
	int  Read(char* buffer, unsigned long bufferSize, unsigned int msWait);
	int  ReadByte(char* buffer, unsigned int msWait);
	bool IsOpen();
	void Close();

    // Use PortName usually "COM1:" ... "COM4:" note that the name must end by ":"
	virtual bool Open(TCHAR* PortName, 
		unsigned long BaudRate		= 9600, 
		unsigned char ByteSize		= 8, 
		unsigned char Parity		= NOPARITY, 
		unsigned char StopBits		= ONESTOPBIT, 
		unsigned long DesiredAccess = GENERIC_READ|GENERIC_WRITE);

	SerialPort();
	virtual ~SerialPort();

protected:
	int wrRetries;
	int rdRetries;
	DCB dcb;

	HANDLE m_PortHandle;
};

#endif // _SERIALPORT_H_