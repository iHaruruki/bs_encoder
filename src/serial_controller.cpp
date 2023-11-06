#include"serial_controller.h"
#include <sys/time.h>

SerialPort* SerialPort::sp_ptr = NULL;
void (*SerialPort::irq_func)(SerialPort* sp) = NULL;

SerialPort::SerialPort()
{
	fd = -1;
	baudrate = 9600;
	device = "/dev/ttyACM0";
	new_line = "\n";
}

SerialPort::SerialPort(const char* _device)
{
	fd = -1;
	baudrate = 9600;
	device = _device;
	new_line = "\n";
}

void SerialPort::Init()
{
	tcgetattr(fd, &oldtio);

	bzero(&newtio, sizeof(newtio));

	// control modes
	newtio.c_cflag |= CS8 | CLOCAL | CREAD | CRTSCTS;
	// input modes
	newtio.c_iflag |= IGNPAR | IGNBRK;
	// output modes
	newtio.c_oflag = 0;
	// local modes
	newtio.c_lflag &= ~(ICANON | ECHO);	// Echo off
	// special characters
	newtio.c_cc[VTIME] = 1;    // Time out x 1/10sec 
	newtio.c_cc[VMIN] = 1;     // Read Byte
	// set baudrate
	cfsetspeed(&newtio, baudrate);

	struct serial_struct serial_setting;
	ioctl(fd, TIOCGSERIAL, &serial_setting);
	serial_setting.flags |= ASYNC_LOW_LATENCY;
	ioctl(fd, TIOCSSERIAL, &serial_setting);

	Flush();
	tcsetattr(fd, TCSANOW, &newtio);
}

int SerialPort::Open()
{
	fd = open(device.c_str(), O_RDWR | O_NOCTTY);
	if(fd < 0) return fd;

	Init();
	Flush();

	return fd;
}

bool SerialPort::Close()
{
	if(fd < 0) return false;
	tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
	return true;
}

void SerialPort::Flush()
{
	if(fd >= 0) {
		tcflush(fd, TCIFLUSH);	// receive/send buffer clear
		tcflush(fd, TCOFLUSH);	// send buffer clear
	}
}

void SerialPort::setBaud(speed_t _baudrate)
{
	baudrate = _baudrate;
}

void SerialPort::setDevice(std::string _device)
{
	device = _device;
}

int SerialPort::setNewLine(std::string _new_line)
{
	if(_new_line.compare("\r")==0 || _new_line.compare("\n")==0 || _new_line.compare("\r\n")==0) {
		new_line = _new_line;
		return 0;
	}
	return -1;
}

int SerialPort::Readable()
{
	if(fd < 0) return 0;
	int size = 0;
	ioctl(fd, FIONREAD, &size);
	return size;
}

char SerialPort::ReadByte()
{
	if(fd < 0) return 0;
	char c;
	read(fd, &c, 1);
	return c;
}

int SerialPort::Read(unsigned char* _str, int len)
{
	if(fd < 0) return 0;
	return read(fd, _str, len);
/*	char c;
	int read_size = 0;
	while(read_size < len) {
		if(Readable() > 0) {
			read(fd, &c, 1);
			_str[read_size] = c;
			read_size ++;
		}
		else break;
	}
	return read_size;
*/
}

int SerialPort::ReadLine(char* _str, int len)
{
	if(fd < 0) return 0;

	char c;
	int read_size = 0;
	static int nl_flag = 0;

	while(read_size < len) {
		if(read(fd, &c, 1) > 0)
		{
			if(new_line.size() == 1) {
				if(c == new_line[0]) {
					_str[read_size] = 0;
					break;
				}
			}
			else {
				if(nl_flag == 0) {
					if(c == new_line[0]) {
						nl_flag = 1;
						continue;
					}
				}
				else {
					if(c == new_line[1]) {
						_str[read_size] = 0;
						nl_flag = 0;
						break;
					}
					else {
						nl_flag = 0;
					}
				}
			}
			_str[read_size] = c;
			read_size ++;
		}
	}
	return read_size;
}

void SerialPort::Write(const unsigned char* _str, int len)
{
	if(fd < 0) return;
	write(fd, _str, len);
}

void SerialPort::Write(std::string _str)
{
	if(fd < 0) return;
	write(fd, _str.c_str(), _str.size());
}

void SerialPort::WriteLine(const unsigned char* _str, int len)
{
	if(fd < 0) return;
	write(fd, _str, len);
	write(fd, new_line.c_str(), new_line.size());
}

void SerialPort::WriteLine(std::string _str)
{
	if(fd < 0) return;
	_str += new_line;
	write(fd, _str.c_str(), _str.size());
}

void SerialPort::callback(int status)
{
	SerialPort::irq_func(SerialPort::sp_ptr);
}

void SerialPort::Attach(void (*fptr)(SerialPort* sp), SerialPort* sp2)
{
	if(fd < 0) return;

	irq_func = fptr;
	sp_ptr = sp2;

	saio.sa_handler = SerialPort::callback;
	sigemptyset(&saio.sa_mask);
	saio.sa_flags = 0;
	saio.sa_restorer = NULL;
	sigaction(SIGIO, &saio, NULL);

	fcntl(fd, F_SETOWN, getpid());
	fcntl(fd, F_SETFL, FASYNC);
}


