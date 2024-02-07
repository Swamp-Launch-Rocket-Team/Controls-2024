#include <iostream>
#include <chrono>
#include <list>
#include <fstream>
#include <stdlib.h>
#include <filesystem>
#include <thread>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <wiringPi.h>

//
//Here list the includes for the other header files used, IMU, altimeter, state, dynamics model, controller, servo
#include "State/state.h"
#include "Dynamics_Model_Controller/controller.h"
#include "Dynamics_Model_Controller/dynamics_model.h"
#include "Dynamics_Model_Controller/PI.h"
#include "Servo/Servo_arming.h"
//

//
//Here list the "prototypes", this is just the initialization of the functions, all state transition functions, write data, launch detect, send servo command, etc..
void PAD_status(Servo &pad, vector<int> homing_limit_switches, vector<int> max_limit_switches, int Pwm_pin, float Pwm_init_val, state_t &state);
void ARMED_status(int Pwm_pin, double Pwm_home_value, state_t &state, pair<long, state_t> (&launch_detect_log)[1024], int &index, chrono::_V2::system_clock::time_point &start, chrono::_V2::system_clock::time_point &launch_time);
void LAUNCH_DETECTED_status();
void ACTUATION_status();
void APOGEE_DETECTED_status();


bool detect_launch(pair<long, state_t> (&launch_detect_log)[1024], int index);
//

using namespace std;

int main()
{
    //Initialize all Sensors and Subsystems
    //Initialize IMU
    int file;
    int imu_address = 0x6B;
    file = imu_init(imu_address);

    //Initialize Altimeter
    //
    
    //Initialize Limit Switches
    const vector<int> homing_limit_switches{1,2};           //vector of GPIO pins for the homing limit switches
    const vector<int> max_limit_switches{3,4,5,6};          //vector of GPIO pins for the max limit switches

    wiringPiSetup();                                        //access the wiringPi library
    for(int i = 0; i < homing_limit_switches.size(); i++)   //Defining the pins for the home limit switches
    {
       pinMode(homing_limit_switches[i], INPUT);            //Set eatch home limit switch as an input
    }

    for(int i = 0; i < max_limit_switches.size(); i++)      //Defininig the pins for the max limit switches
    {
        pinMode(max_limit_switches[i], INPUT);              //Sets each max limit switch as an input
    }

    //Initialize Servo Motor
    const int Pwm_pin = 1;              //GPIO hardware PWM pin
    const float Pwm_init_val = 500;     //initial PWM value for the servo, should correspond to around halfway between home and max
    pinMode(Pwm_pin, PWM_OUTPUT);       //setting the Pwm pin as a PWM output pin
    pwmWrite(Pwm_pin, Pwm_init_val);    //Set the servo position to the initial PWM value, should be between home and max switches
    Servo pad;                          //create instance of Servo

    //Store the airbrakes current state
    state_t state; // Stores information of airbrake state

    //Idk what exactly this does but Jason has it, the detect launch index is used in the launch detect algorithm
    list<pair<long, state_t>> data_log;
    pair<long, state_t> launch_detect_log[1024];
    int launch_detect_log_index = 0;

    //Initialize and define variables for starting time and current time
    auto start = chrono::high_resolution_clock::now();
    auto cur = chrono::high_resolution_clock::now();
    auto launch_time = chrono::high_resolution_clock::now();        //This needs to get reset once launch is detected

    //Set the status to PAD
    state.status = state_t::PAD;

    //While loop that runs until the APOGEE_DETECTED state is reach, aka this is run from being powered on the pad to apogee
    while (true && state.status != state_t::APOGEE_DETECTED)
    {
        cur = chrono::high_resolution_clock::now();
        state.imu_data = imu_read_data();
        //add a read from the altimeter here

        //Switch cases that transition between the different states
        switch (state.status)
        {
            case state_t::PAD:
                //PAD_status() this function will call the PAD_status void function that will call the Servo_arming file
                PAD_status(pad, homing_limit_switches, max_limit_switches, Pwm_pin, Pwm_init_val, state);
                break;

            case state_t::ARMED:
                //ARMED_status() this function will call the ARMED_status void function that will call the detect_launch function
                ARMED_status(Pwm_pin, pad.get_Pwm_home_value(), state, launch_detect_log, launch_detect_log_index, start, launch_time);
                break;

            case state_t::LAUNCH_DETECTED:
                //LAUNCH_DETECTED_status() this function will call the LAUNCH_DETECTED_status void function that will call the detect_motor_burn_end function or maybe just a timer
                pwmWrite(Pwm_pin, pad.get_Pwm_home_value());        //Ensures the servo is at the home position
                break;

            case state_t::ACTUATION:
                //ACTUATION_status() this function will call the ACTUATION_status void function that will call the dynamics model and controller from Dynamics_Model_Controller folder, and a detect_apogee function
                break;
        }

        //Log the data
        data_log.push_back(make_pair(chrono::duration_cast<chrono::microseconds>(cur - start).count(), state));

        //Delay so that IMU and altimeter do not read too fast
        while (chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - cur).count() < 9500);
    }

    //Write the data, command the servo to return to the home position, put the Pi to sleep
    APOGEE_DETECTED_status(state, data_log, cmd_log);

    return 0;
}

//
//Add all of the void state status functions here
void PAD_status(Servo &pad, vector<int> homing_limit_switches, vector<int> max_limit_switches, int Pwm_pin, float Pwm_init_val, state_t &state)
{
    pad.Servo_cal(Pwm_pin, Pwm_init_val, homing_limit_switches, max_limit_switches);
    delay(1000);        //delay of 1 s
    if(pad.get_Pwm_home_value() != pad.get_Pwm_init_value() && pad.get_Pwm_max_value() != pad.get_Pwm_init_value() && pad.get_Pwm_home_value() < pad.get_Pwm_init_value() && pad.get_Pwm_max_value() > pad.get_Pwm_init_value())
    {
        state.status = state_t::ARMED;
    }
}

void ARMED_status(int Pwm_pin, double pad.get_Pwm_home_value(), state_t &state, pair<long, state_t> (&launch_detect_log)[1024], int &index, chrono::_V2::system_clock::time_point &start, chrono::_V2::system_clock::time_point &launch_time)
{
    pwmWrite(Pwm_pin, pad.get_Pwm_home_value());        //Ensures servo is at the home position
    
    launch_detect_log[index & 1023] = make_pair(chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start).count(), state);

    if (detect_launch(launch_detect_log, index))
    {
        state.status = state_t::LAUNCH_DETECTED;
    }

    index++;

    return;
}
//

//
//Add all of the other functions here: launch detect, maybe servo arming, detect motor burn end, detect apogee, write data from Jasons main, 
bool detect_launch(pair<long, state_t> (&launch_detect_log)[1024], int index)
{
    float sorted_log[300];

    float p_avg_accel = 0;

    for (int i = 0; i < 300; ++i)
    {
        p_avg_accel += axes_mag(launch_detect_log[(index - (300 + i)) & 1023].second.imu_data.accel);
    }

    p_avg_accel /= 300;

    float stdev = 0;

    for (int i = 0; i < 300; ++i)
    {
        stdev += pow(axes_mag(launch_detect_log[(index - (300 + i)) & 1023].second.imu_data.accel) - p_avg_accel, 2.0);
    }

    stdev = sqrt(stdev / 299);

    float avg_accel = 0;
    int count;

    for (int i = 0; i < 300; ++i)
    {
        if (axes_mag(launch_detect_log[(index - (300 + i)) & 1023].second.imu_data.accel) < 3 * stdev)
        {
            avg_accel += axes_mag(launch_detect_log[(index - (300 + i)) & 1023].second.imu_data.accel);
            count++;
        }
    }

    avg_accel /= count;

    if (avg_accel > 5 * 9.8) // 5 gs of acceleration
    {
        return true;
    }

    return false;
}
//