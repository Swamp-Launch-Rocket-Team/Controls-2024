#include "imu.h"
#include <chrono>
#include <fstream>
#include <map>

void write_data(ofstream &out_file, axes_t &imu_data);
void print_data(unsigned char data[], int len);
void print_data(const vector<unsigned char> &data);
void print_all(imu_data_t &imu_data);

int main()
{
    // Set up variables for the I2C device address, file descriptor, and data buffer
    int address = 0x6B;
    int file;
    unsigned char data[40];

    imu_data_t imu_data;

    file = imu_init(address);

    // vector<imu_data_t> data_vec;
    map<int,imu_data_t> data_map;

    auto cur_time = chrono::high_resolution_clock::now();
    //auto loop_time = chrono::high_resolution_clock::now();
    auto start = chrono::high_resolution_clock::now();

    int i = 1;

    while (chrono::duration_cast<chrono::minutes>(cur_time - start).count() < 15)
    {
        //loop_time = chrono::high_resolution_clock::now();

        imu_data = imu_read_data();

        // print_all(imu_data);
        // data_vec.push_back(imu_data);
        data_map.emplace(i,imu_data);

        cur_time = chrono::high_resolution_clock::now();

        //if (!(chrono::duration_cast<chrono::microseconds>(cur_time - loop_time).count() > 10000))
        //{
        //    usleep(10000 - chrono::duration_cast<chrono::microseconds>(cur_time - loop_time).count());
        //}
        //else
        //{
        //    cout << "Time exceeds 10 ms: " << i << " " << chrono::duration_cast<chrono::milliseconds>(cur_time - loop_time).count() << endl;
        //}

        cout << chrono::duration_cast<chrono::seconds>(cur_time - start).count();

        cout << "\r" << flush;

        ++i;
    }

    ofstream out_file;
    out_file.open("imu_data.csv");

    for (auto it = data_map.begin(); it != data_map.end(); ++it)
    {
        out_file << it->first << ",";
        write_data(out_file,it->second.heading);
        out_file << ",";
        write_data(out_file,it->second.accel);
        out_file << ",";
        write_data(out_file,it->second.del_v);
        out_file << ",";
        write_data(out_file,it->second.ang_v);
        out_file << endl;
    }


    // Close the I2C device file
    close(file);

    return 0;
}

void write_data(ofstream &out_file, axes_t &imu_data)
{
    out_file << imu_data.x << "," << imu_data.y << "," << imu_data.z;
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

void print_all(imu_data_t &imu_data)
{
    printf("Heading: %.4f \t %.4f \t %.4f \t", // \t %.4f \t Accel: %.4f \t %.4f \t %.4f \t      ", // Ang v: %.4f | %.4f | %.4f ||\t Del v: %.4f | %.4f | %.4f ||\n",
        imu_data.heading.x,imu_data.heading.y,imu_data.heading.z);
        // imu_data->accel.x); imu_data->accel.y,imu_data->accel.z);
        // imu_data->ang_v.x,imu_data->ang_v.y,imu_data->ang_v.z,
        // imu_data->del_v.x,imu_data->del_v.y,imu_data->del_v.z);

    usleep(10000);

    cout << "\r" << flush;
}
