#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>        //NEED TO UNCOMMENT THIS TO RUN
#include <linux/i2c-dev.h>    //NEED TO UNCOMMENT TO RUN
#include <iostream>
#include <vector>
// #include <chrono>
#include <cmath>
#include <map>
#include "axes.h"
#include "../busynano/busynano.h"

// Opcodes
#define PRTCL_INFO 0x01
#define CONF_PRTCL 0x02
#define CNTRL_PIPE 0x03
#define PIPE_STATUS 0x04
#define NOTIF_PIPE 0x05
#define MEAS_PIPE 0x06

// #define IMU_SPI_DEVICE "/dev/spidev0.0"
// #define IMU_SPI_MODE SPI_MODE_3

using namespace std;

struct imu_data_t
{
    struct gps_t
    {
        float lat = -1;
        float lon = -1;
    } gps;

    float alt = -1;

    axes_t velocity;
    axes_t ang_v;
    axes_t heading;
    axes_t accel;

    unsigned int pressure = 0;
};

int imu_init();
bool go_to_config();
bool go_to_measurement();
imu_data_t imu_read_data();
void parse_msg(imu_data_t &imu_data);
void parse_float(float &num, int num_offset);
void parse_int(unsigned int &num, int num_offset);
imu_data_t rotate_axes(imu_data_t old_axes);
bool send_xbus_msg(vector<unsigned char> cmd);
bool check_sum();
// int continuous_read(vector<unsigned char> *buf, unsigned char opcode);
inline void set_offset(imu_data_t &offset);
// void imu_moving_avg_calibrate(map<int, imu_data_t> &data_set);
