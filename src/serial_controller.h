//
// serial_control.h
// 2020.11.19
//

#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<linux/serial.h>
#include<sys/ioctl.h>
#include<termios.h>
#include<sys/signal.h>
#include<string>
#include<strings.h>

#ifndef _DEFINED_SERIAL_CONTROL_H_
#define _DEFINED_SERIAL_CONTROL_H_

//using namespace std;

class SerialPort
{
private:
	int fd;
	speed_t baudrate;
	std::string device;
	std::string new_line;
	struct termios oldtio, newtio;
	struct sigaction saio;

public:
	SerialPort();
	SerialPort(const char* _device);
	
	static SerialPort* sp_ptr;
	static void (*irq_func)(SerialPort* sp);
	static void callback(int status);

	void Init();	
	int  Open();
	bool Close();
	void Flush();
	
	void setBaud(speed_t _baudrate);
	void setDevice(std::string _device);
	int  setNewLine(std::string _new_line);
	int  Readable();
	char ReadByte();
	int  Read(unsigned char* _str, int _len);
	int  ReadLine(char* _str, int len);
	void Write(const unsigned char* _str, int len);
	void Write(std::string _str);
	void WriteLine(const unsigned char* _str, int len);
	void WriteLine(std::string _str);
	void Attach(void (*fptr)(SerialPort *sp), SerialPort* sp2);
};

#endif
