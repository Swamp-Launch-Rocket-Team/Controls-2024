#include "imu.h"
#include <chrono>
#include "../controller/state.h"
#include <fstream>

void print_all(imu_data_t &imu_data);
void print_all(axes_t &data);
void write_data(ofstream &out_file, axes_t &imu_data);

int main()
{
    // Set up variables for the I2C device address, file descriptor, and data buffer
    int address = 0x6B;

    map<int, imu_data_t> data_set;
    state_t state;

    imu_init(address);

    for (int i = 0; i < 100; ++i)
    {
        auto start_loop = chrono::high_resolution_clock::now();

        data_set.emplace(i,imu_read_data());

        auto end_loop = chrono::high_resolution_clock::now();

        if (!(chrono::duration_cast<chrono::microseconds>(start_loop - end_loop).count() > 10000))
        {
            usleep(10000 - chrono::duration_cast<chrono::microseconds>(start_loop - end_loop).count());
        }
        else
        {
            cout << "Time exceeds 10 ms: " << i << " " << chrono::duration_cast<chrono::milliseconds>(start_loop - end_loop).count() << endl;
        }
    }

    imu_moving_avg_calibrate(data_set);

	cout << "Calibration set" << endl;

    map<int,imu_data_t> data_map;

    auto start = chrono::high_resolution_clock::now();
    auto end_loop = chrono::high_resolution_clock::now();

    int i = 1;

    while (chrono::duration_cast<chrono::minutes>(end_loop - start).count() < 5)
    {
        auto start_loop = chrono::high_resolution_clock::now();

        // cout << "\r" << flush;
// cout << "1" << endl;

        state.imu_data = imu_read_data();

// cout << "2" << endl;

        if (isnan(state.imu_data.accel.x))
        {
		cout << "sdnjuh" << endl;
		continue;
        }

        state.velocity.x += state.imu_data.del_v.x;
        state.velocity.y += state.imu_data.del_v.y;
        state.velocity.z += state.imu_data.del_v.z;

        data_map.emplace(i,state.imu_data);

	// if(isnan(state.velocity.x) || state.velocity.x == 0)
	// {
	// 	cout << "Read Error" << "\n";
	// }

        end_loop = chrono::high_resolution_clock::now();

        if (!(chrono::duration_cast<chrono::microseconds>(start_loop - end_loop).count() > 9000))
        {
            usleep(10000 - chrono::duration_cast<chrono::microseconds>(start_loop - end_loop).count());
        }
        else
        {
            cout << "Time exceeds 10 ms: " << chrono::duration_cast<chrono::milliseconds>(start_loop - end_loop).count() << endl;
        }

	// if (i % 100 == 0)
	// {
        cout << "Del v: ";
	    print_all(state.imu_data.del_v);
        cout << "Velocity: ";
	    print_all(state.velocity);
		cout << endl;
	// }

        // cout << chrono::duration_cast<chrono::seconds>(end_loop - start).count();

        ++i;
    }

    ofstream out_file;
    out_file.open("velocity_drift.csv");

    for (auto it = data_map.begin(); it != data_map.end(); ++it)
    {
        out_file << it->first << ",";
        write_data(out_file,it->second.del_v);
        out_file << endl;
    }

    return 0;
}

void print_all(imu_data_t &imu_data)
{
    cout << "\r" << flush;

    printf("Heading: %.4f      %.4f      %.4f     ", // \t %.4f \t Accel: %.4f \t %.4f \t %.4f \t      ", // Ang v: %.4f | %.4f | %.4f ||\t Del v: %.4f | %.4f | %.4f ||\n",
        imu_data.accel.x,imu_data.accel.y,imu_data.accel.z);
        // imu_data->accel.x); imu_data->accel.y,imu_data->accel.z);
        // imu_data->ang_v.x,imu_data->ang_v.y,imu_data->ang_v.z,
        // imu_data->del_v.x,imu_data->del_v.y,imu_data->del_v.z);
}

void print_all(axes_t &data)
{
    // cout << "\r" << flush;

    printf("%.4f      %.4f      %.4f      ", // \t %.4f \t Accel: %.4f \t %.4f \t %.4f \t      ", // Ang v: %.4f | %.4f | %.4f ||\t Del v: %.4f | %.4f | %.4f ||\n",
        data.x,data.y,data.z);
        // imu_data->accel.x); imu_data->accel.y,imu_data->accel.z);
        // imu_data->ang_v.x,imu_data->ang_v.y,imu_data->ang_v.z,
        // imu_data->del_v.x,imu_data->del_v.y,imu_data->del_v.z);
}

void write_data(ofstream &out_file, axes_t &imu_data)
{
    out_file << imu_data.x << "," << imu_data.y << "," << imu_data.z;
}
