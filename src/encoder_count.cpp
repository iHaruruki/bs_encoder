#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "serial_controller.h"

SerialPort sp;
unsigned char encoder_send[] = {0x02, 0x01, 0x05, 0xF9, 0x03};
unsigned char encoder_response[10];

int Encoder_right = 0;
int Encoder_left = 0;
int Encoder_right_before = 0;
int Encoder_left_before = 0;

void encoder_count()
{	
	sp.Write(encoder_send, 5);
	sp.Read(encoder_response, 10);

	static int Encoder_right_af = 0;
	static int Encoder_left_af = 0;

	Encoder_right = ((encoder_response[5] << 8) | encoder_response[6]);	// ((encoder_response[5] << 32) | encoder_response[6] << 16)
	Encoder_right -= 32768;		//-(65535/2) = -32767.5 
	Encoder_left = ((encoder_response[7] << 8) | encoder_response[8]);
	Encoder_left -= 32768;		//

	int rotation_right = (Encoder_right_before - Encoder_right)*16*(1/500)*(2/147);	//
	int rotation_left = (Encoder_left_before - Encoder_left)*16*(1/500)*(2/147);	//

	std::cout << "Encoder_right: " << (Encoder_right) << std::endl;
	std::cout << "Encoder_Left:  " << (Encoder_left) << std::endl;
	std::cout << "rotate_right:  " << (rotation_right) << std::endl;
	std::cout << "rotate_Left:   " << (rotation_left) << std::endl;

	Encoder_right_before = Encoder_right;
	Encoder_left_before = Encoder_left;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_count");
    ros::NodeHandle nh;
    //ros::spin();

	// Start
	sp.setDevice("/dev/ttyUSB0");
	sp.setBaud(19200);			//通信速度設定＜確認＞
	sp.Open();

	ros::Rate loop_rate(100);
	sp.Flush();
	while(ros::ok()){
		encoder_count();
		loop_rate.sleep();
	}
	
	sp.Flush();
	sp.Close();
	
    return 0;
}
