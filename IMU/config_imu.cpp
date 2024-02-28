#include "imu.h"

int main()
{
	imu_init(0x6B);

	// go_to_config();
	vector<unsigned char> cmd;

	// sleep(5);

	// cmd.push_back(0x03);
	// cmd.push_back(0x24);
	// cmd.push_back(0x00);
	
	
	
	// cmd.push_back(0x1C); // length excluding cs, length, preamble
	// cmd.push_back(0x50);
	// cmd.push_back(0x40);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);
	// cmd.push_back(0x50);
	// cmd.push_back(0x20);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);
	// cmd.push_back(0x20);
	// cmd.push_back(0x30);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);
	// cmd.push_back(0x80);
	// cmd.push_back(0x20);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);
	// cmd.push_back(0xD0);
	// cmd.push_back(0x10);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);
	// cmd.push_back(0x30);
	// cmd.push_back(0x10);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);
	// cmd.push_back(0x40);
	// cmd.push_back(0x20);
	// cmd.push_back(0x00);
	// cmd.push_back(0x64);



	// send_xbus_msg(cmd);

	//sleep(5);

	go_to_measurement();

}
