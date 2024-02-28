#include "imu.h"
#include <chrono>
#include <fstream>
#include <list>
#include <wiringPi.h>
#include <exception>
#include <signal.h>
#include <stdlib.h>
#include <utility>
#include <ctime>
#include <iomanip>

using namespace std;

class InterruptException : public std::exception
{
    public:
        InterruptException(int s) : S(s) {}
        int S;
};

void sig_to_exception(int s);

int main()
{
    // Turn off buzzer
    wiringPiSetup();
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);

    // Initialize all subsystems
    // Initialize IMU
    int file;
    int imu_address = 0x6B;
    file = imu_init(imu_address);

    imu_data_t state;

    auto start = chrono::high_resolution_clock::now();
    auto cur = chrono::high_resolution_clock::now();
    auto write = chrono::high_resolution_clock::now();

    long index = 0;
    ofstream log_file;
    auto t = time(nullptr);
    auto tm = *localtime(&t);
    stringstream temp;
    temp <<  put_time(&tm, "plog_%d-%m-%Y_%H-%M-%S.csv");
    string file_name = temp.str();
    log_file.open(file_name);

    list<pair<long, float>> datavec;

    bool first = true;
    auto it2 = datavec.begin();
    auto it = datavec.begin();

    bool a = false;

    try
    {
        while (chrono::duration_cast<chrono::minutes>(cur - start).count() < 30)
        {
            cur = chrono::high_resolution_clock::now();

            state = imu_read_data();

            //int delay = 1000000;
            //busy10ns(delay); // 10 ms delay

        datavec.push_back(make_pair(chrono::duration_cast<chrono::microseconds>(cur - start).count(), state.pressure));

        if (first)
        {
            it2 = datavec.begin();
            it = datavec.begin();
            first = false;
        }
        else{
            it = it2;
        }

          //  if (state.alt != 0)
          //  {
          //      log_file << chrono::duration_cast<chrono::microseconds>(cur - start).count() << "," << state.pressure << endl;
          //  }
    a = false;

            if (chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - write).count() >= 2*60)
            {
                write = chrono::high_resolution_clock::now();
                digitalWrite(21, HIGH);
                sleep(1);
        for (it; it != datavec.end(); it++)
                {
                    log_file << (*it).first << "," << (*it).second << endl;
                    if (a)
                    {
                        it2++;
                    }
                    a = true;
                }
                digitalWrite(21, LOW);
            }
            while (chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - cur).count() < 9500);
        }
    }
    catch (InterruptException& e)
    {
        log_file << "End" << endl;
        log_file.close();
        return 1;
    }



    log_file.close();

    return 0;
}

void sig_to_exception(int s)
{
    throw InterruptException(s);
}
