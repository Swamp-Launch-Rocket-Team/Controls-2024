#include "imu.h"
#include "../Altimiter/spi.h"
#include <wiringPi.h>

#include "../Altimiter/bitbang.h"

static int heading_byte_offset = 0; // Heading data offset from start of messages in bytes
static int accel_byte_offset = 0; // Acceleration data offset from start of messages in bytes
static int ang_v_byte_offset = 0; // Angular velocity data offset from start of messages in bytes
static int del_v_byte_offset = 0; // Delta V data offset from start of messages in bytes
static vector<unsigned char> buf; // Buffer where received data is stored

// Initializes IMU file and sets I2C slave
int imu_init()
{
    // Open the I2C device file
    // if ((file = open("/dev/i2c-1", O_RDWR)) < 0)
    // {
    //     cout << "Error opening I2C device file" << endl;
    //     return -1;
    // }

    // // Set the device address
    // if (ioctl(file, I2C_SLAVE, address) < 0)
    // {
    //     cout << "Error setting I2C device address" << endl;
    //     return -1;
    // }

    // digitalWrite(14, HIGH);

    // wiringPiSetup();

    // pinMode(14, OUTPUT);
    // pinMode(11, OUTPUT);
    // pinMode(10, OUTPUT);

    // digitalWrite(10, LOW);
    // digitalWrite(11, LOW);



    // while (true)
    // {
    //     digitalWrite(14, HIGH);
    //     delay(500);
    //     digitalWrite(14, LOW);
    //     delay(500);
    // }

    spi_init_bitbang();

    buf.resize(110);

    return 0;
}

// Sets IMU to configuration state
bool go_to_config()
{    
    char cmd[4] = {CNTRL_PIPE,0x30,0x00,0xD1};

    // spi_write(IMU_SPI_DEVICE, IMU_SPI_MODE, cmd, 4);
    // TODO, error checking
    // if ( != 4)
    // {
    //     std::cout << "Error writing to I2C device: device not set to config mode" << std::endl;
    //     return false;
    // }

    return true;
}

// Sets IMU to measurement state
bool go_to_measurement()
{    
    char cmd[4] = {CNTRL_PIPE,0x10,0x00,0xF1};
    char buff[4] = {0};
    
    // spi_transact(IMU_SPI_DEVICE, IMU_SPI_MODE, cmd, buff, 4);
    spi_transfer_bitbang(cmd, buff, 4, 3, 0);

    for (int i = 0; i < 4; i++)
    {
        std::cout << hex << (int)buff[i] << std::endl;
    }
    // TODO, error checking
    // if (write(file, cmd, 4) != 4)
    // {
    //     std::cout << "Error writing to I2C device: device not set to measurement mode" << std::endl;
    //     return false;
    // }

    return true;
}

// Reads data using the measurement pipe opcode and write data to buf
imu_data_t imu_read_data()
{
    imu_data_t imu_data;

    char send[111] = {0};
    send[0] = MEAS_PIPE;
    // const unsigned char READ_DATA = MEAS_PIPE;
    char receive[111] = {0};

    // spi_transact(IMU_SPI_DEVICE, IMU_SPI_MODE, send, receive, 5);
    spi_transfer_bitbang(send, receive, 111, 3, 0);
    for (int i = 0; i < buf.size(); i++)
    {
        buf[i] = receive[i+4];
    }
    // std::cout << std::hex << (int)receive[0] << std::endl;
    // std::cout << std::hex << (int)buf[0] << std::endl;
    // return imu_data;


    // if (write(file, &READ_DATA, 1) != 1 || read(file, &buf[0], buf.size()) != buf.size())
    // {
        
        
        // for (int i = 0; i < buf.size(); i++)
        // {
        //     cout << hex << (int)buf[i] << " ";
        // }
        // cout << endl;
        // cout << "Error writing/reading from I2C device" << endl;
        // return imu_data;
    // }

    while (!check_sum())
    {
        cout << "check sum fail" << endl;
        // std::cout << std::hex << (int)receive[0] << std::endl;
        // std::cout << std::hex << (int)buf[0] << std::endl;
        busy10ns(150000);
        // spi_transact(IMU_SPI_DEVICE, IMU_SPI_MODE, send, receive, 111);
        send[0] = MEAS_PIPE;
        spi_transfer_bitbang(send, receive, 111, 3, 0);
        for (int i = 0; i < buf.size(); i++)
        {
            buf[i] = receive[i+4];
        }
        // if (write(file, &READ_DATA, 1) != 1 || read(file, &buf[0], buf.size()) != buf.size())
        // {
        //     cout << "Error writing/reading from I2C device" << endl;
        //     return imu_data;
        // }
    }

    

    parse_msg(imu_data);

    // Prints out if data is NaN
    // if (isnan(imu_data.heading.x))
    // {
    //     cout << "Error: NaN found in message - ";
        // for (int i = 0; i < 110; ++i)
        // {
        //     cout << hex << (int)buf[i] << " ";
        // }
        // cout << endl;
    // }

    imu_data = rotate_axes(imu_data);

    return imu_data;
}

// Write data to imu_data struct
void parse_msg(imu_data_t &imu_data)
{

    int data_len = buf[1];

    for (int i = 2; i < data_len; ++i)
    {
        if (buf[i] == 0x50 && buf[i+1] == 0x40 && buf[i+2] == 0x08) // XDI_LatLon 100 Hz
        {
            parse_float(imu_data.gps.lat, i + 3);
            parse_float(imu_data.gps.lon, i + 7);
        }
        else if (buf[i] == 0x50 && buf[i+1] == 0x20 && buf[i+2] == 0x04) // XDI_AltitudeEllipsoid 100 Hz
        {
            parse_float(imu_data.alt, i + 3);
        }
        else if (buf[i] == 0xD0 && buf[i+1] == 0x10 && buf[i+2] == 0x0C) // XDI_VelocityXYZ 100 Hz
        {
            parse_float(imu_data.velocity.x, i + 3);
            parse_float(imu_data.velocity.y, i + 7);
            parse_float(imu_data.velocity.z, i + 11);
        }
        else if (buf[i] == 0x80 && buf[i+1] == 0x20 && buf[i+2] == 0x0C) //XDI_RateOfTurn 100 Hz
        {
            parse_float(imu_data.ang_v.x, i + 3);
            parse_float(imu_data.ang_v.y, i + 7);
            parse_float(imu_data.ang_v.z, i + 11);
        }
        else if (buf[i] == 0x20 && buf[i+1] == 0x30 && buf[i+2] == 0x0C) // XDI_EulerAngles 100 Hz
        {
            parse_float(imu_data.heading.x, i + 3);
            parse_float(imu_data.heading.y, i + 7);
            parse_float(imu_data.heading.z, i + 11);
        }
        else if (buf[i] == 0x30 && buf[i+1] == 0x10 && buf[i+2] == 0x04) // XDI_EulerAngles 100 Hz
        {
            parse_int(imu_data.pressure, i + 3);
        }
        else if (buf[i] == 0x40 && buf[i+1] == 0x20 && buf[i+2] == 0x0C) // XDI_EulerAngles 100 Hz
        {
            parse_float(imu_data.accel.x, i + 3);
            parse_float(imu_data.accel.y, i + 7);
            parse_float(imu_data.accel.z, i + 11);
        }
    }

    return;
}

// parses buf[] for the num at the num_offset
void parse_float(float &num, int num_offset)
{
    unsigned char arr[4] = {0,0,0,0};

    arr[3] = buf[num_offset];
    arr[2] = buf[num_offset + 1];
    arr[1] = buf[num_offset + 2];
    arr[0] = buf[num_offset + 3];

    num = *(float*)&arr;
}

void parse_int(unsigned int &num, int num_offset)
{
    unsigned char arr[4] = {0,0,0,0};

    arr[3] = buf[num_offset];
    arr[2] = buf[num_offset + 1];
    arr[1] = buf[num_offset + 2];
    arr[0] = buf[num_offset + 3];

    num = *(unsigned int*)&arr;
}

// checks the checksum byte to confirm valid message
bool check_sum()
{
    unsigned char sum = 0xFF;
    int msg_len = buf[1] + 3;
    for (int i = 0; i < msg_len; ++i)
    {
        sum += buf[i];
    }

    if (sum != 0x00)
    {
        // for(int x : buf)
        // {
        //     cout << hex << x << " ";
        // }
        // cout << "\nCalc sum: " << hex << (int)(sum) << " ";
	    return false;
    }
    return true;
}

// Rotate the axes such that the axes align with the drone's coordinate system
imu_data_t rotate_axes(imu_data_t old_axes)
{
    imu_data_t new_axes = old_axes; // THIS IS WRONG BECAUSE WE ARE DUMB
    // // Drone X = -IMU Y
    // // Drone Y =  IMU X
    // // Drone Z =  IMU Z
    new_axes.heading.x = old_axes.heading.y;
    new_axes.heading.y = -old_axes.heading.x;

    new_axes.velocity.x = old_axes.velocity.y;
    new_axes.velocity.y = -old_axes.velocity.x;

    new_axes.ang_v.x = old_axes.ang_v.y;
    new_axes.ang_v.y = -old_axes.ang_v.x;

    // BELOW IS FOR THE PROTOTYPE

    // new_axes.heading.y = -old_axes.heading.y;
    // new_axes.heading.z = -old_axes.heading.z;

    // new_axes.velocity.y = -old_axes.velocity.y;
    // new_axes.velocity.z = -old_axes.velocity.z;

    // new_axes.ang_v.y = -old_axes.ang_v.y;
    // new_axes.ang_v.z = -old_axes.ang_v.z;

    return new_axes;
}

// Takes cmd without checksum, calculates checksum and sends message to IMU
bool send_xbus_msg(vector<unsigned char> cmd)
{
    int file = 0; // THIS DOES NOT WORK
    unsigned char checksum = 0xFF;

    for (int i = 1; i < cmd.size(); ++i)
    {
        checksum += cmd[i];
    }

    checksum = (~checksum) + 1;

    cmd.push_back(checksum);
    vector<unsigned char> ack;
    ack.resize(60);
    unsigned char temp = 0x05;
    unsigned char meas = 0x06;
    // if (write(file, &temp, 1) != 1 || read(file,&ack[0],ack.size()) != ack.size())
    // {
	//     cout << "Clear buf fail" << endl;
    // }
    // int count = 0;
    // while (ack[0] != 0x00)
    // {
	//     if (write(file,&temp,1) != 1);
    //     {
	//         cout << "write" << endl;
	//         break;
	//     }
    //     if (read(file,&ack[0],ack.size()) != ack.size())
    //     {
    //         cout << "read" << endl;
    //         break;
    //     }
    //     if (count > 1000)
    //     {
    //         cout << "count too big" << endl;
    //         break;
    //     }
    // }
    // cout << "Count: " << count << endl;
    if (write(file, &cmd[0], cmd.size()) != cmd.size())// || read(file,&ack[0],ack.size()) != ack.size())
    {
        cout << "Error writing xbus command to I2C device" << endl;
        return false;
    }

    if (write(file, &temp, 1) != 1 || read(file,&ack[0],ack.size()) != ack.size())
    {
	cout << "Error reading notif pipe" << endl;
    }

    for (int i = 0; i < ack.size(); ++i)
    {
	cout << hex << (int)ack[i] << " ";
    }

    // temp = 0x04;
    // if (write(file, &temp, 1) != 1 || read(file,&ack[0],ack.size()) != ack.size())
    // {
	// cout << "Error reading pipe status" << endl;
    // }
    // cout << endl;
    // for (int i = 0; i < ack.size(); ++i)
    // {
	// cout << hex << (int)ack[i] << " ";
    // }

    close(file);
    return true;
}

// IN PROGRESS, MAY NOT WORK CORRECTLY!!! Reads continuously up to 1 second for a message from the specified opcode and writes the message of specified length to buf \return Time taken to receive message, -1 if no message received
// int continuous_read(vector<unsigned char> *data, unsigned char opcode)
// {
//     vector<unsigned char> temp = *data;

//     auto start = chrono::high_resolution_clock::now();
//     auto current = chrono::high_resolution_clock::now();
//     auto duration = chrono::duration_cast<chrono::milliseconds>(current - start);

//     while (duration.count() < 1000)
//     {


//     if ((write(file, &opcode, 1) == 1 || read(file, &data[0], data->size()) == data->size()) && temp != *data)
//     {
//         return duration.count();
//     }

//     current = chrono::high_resolution_clock::now();
//     duration = chrono::duration_cast<chrono::milliseconds>(current - start);
//     }

//     return -1;
// }
