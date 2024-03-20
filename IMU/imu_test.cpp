#include "imu.h"
#include <chrono>

#include "../bitbang/bitbang.h"

void print_data(unsigned char data[], int len);
void print_data(const vector<unsigned char> &data);
void print_all(imu_data_t *imu_data);

int main()
{
    // Set up variables for the I2C device address, file descriptor, and data buffer
    // unsigned char data[40];
    spi_init_bitbang();

    imu_data_t imu_data;

    imu_init();

    // unsigned char test[19] = {CNTRL_PIPE,0xC0,0x10,0x20,0x30,0x00,0x64,0x40,0x30,0x00,0x64,0x80,0x20,0x00,0x64,0x40,0x10,0x00,0x64}; // set config
    // vector<unsigned char> cmd;
    // cmd.insert(cmd.begin(), test, test + 19);

    // go_to_config();
    // sleep(3);
    // send_xbus_msg(cmd);
    // sleep(3);
    // go_to_measurement();
    // sleep(3);

    auto start = chrono::high_resolution_clock::now();
    auto cur = chrono::high_resolution_clock::now();
    while (true)
    {
        start = chrono::high_resolution_clock::now();

        imu_data = imu_read_data();

        print_all(&imu_data);

        cur = chrono::high_resolution_clock::now();
        // usleep(10000 - chrono::duration_cast<chrono::microseconds>(cur - start).count());
    }

    return 0;
}

void print_data(unsigned char data[], int len)
{
    for (int i = 0; i < len; ++i)
    {
        std::cout << std::hex << (int)data[i];
        std::cout << " ";
    }
    std::cout << std::endl;
}

void print_data(const vector<unsigned char> &data)
{
    for (int i = 0; i < data.size(); ++i)
    {
        std::cout << std::hex << (int)data[i];
        std::cout << " ";
    }
    std::cout << std::endl;
}

void print_all(imu_data_t *imu_data)
{
    // printf("Heading: %.4f\t%.4f\t%.4f\tGPS: %.4f\t%.4f\tAlt: %.4f\t\n",
    //     imu_data->heading.x,imu_data->heading.y,imu_data->heading.z,
    //     imu_data->gps.lat, imu_data->gps.lon,
    //     imu_data->alt);

    printf("Heading: %.4f\t%.4f\t%.4f\tACCEL: %.4f\t%.4f\t%.4f\t\n",
        imu_data->heading.x,imu_data->heading.y,imu_data->heading.z,
        imu_data->accel.x, imu_data->accel.y, imu_data->accel.z);

    usleep(10000);

    // cout << flush;
}
