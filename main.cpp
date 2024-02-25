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
//#include "Servo/Servo_arming.h"
//

//
//Here list the "prototypes", this is just the initialization of the functions, all state transition functions, write data, launch detect, send servo command, etc..
void PAD_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state);
void ARMED_status(state_t &state, pair<long, state_t> (&launch_detect_log)[1024], int &index, chrono::_V2::system_clock::time_point &start, chrono::_V2::system_clock::time_point &launch_time);
void LAUNCH_DETECTED_status(state_t &state, auto motor_burn_time, float &theta_0, vector<int> &theta_region, vector<float> &theta_vector);  //Needs more inputs i think
void ACTUATION_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state, float x, float z, float U_airbrake, dynamics_model &dynamics, controller &airbrake, vector<int> theta_region, vector<float> theta_vector);  //Need to add other stuff
void APOGEE_DETECTED_status();


bool detect_launch(pair<long, state_t> (&launch_detect_log)[1024], int index);
pair<vector<int>, vector<float>> pitchanglevector(float theta_0); 
//Idk if I actually need these next 2 functions, can possibly add a method in the state header that calcs these 2 quants and add these to a new struct maybe      
float xdot_calc(state_t state);
float zdot_calc(state_t state);     //Also needs altitude for the derivative as input!!!
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
    
    //Initialize Limit Switches     DON'T NEED ANYMORE
    // const vector<int> homing_limit_switches{1,2};           //vector of GPIO pins for the homing limit switches
    // const vector<int> max_limit_switches{3,4,5,6};          //vector of GPIO pins for the max limit switches

    // wiringPiSetup();                                        //access the wiringPi library
    // for(int i = 0; i < homing_limit_switches.size(); i++)   //Defining the pins for the home limit switches
    // {
    //    pinMode(homing_limit_switches[i], INPUT);            //Set eatch home limit switch as an input
    // }

    // for(int i = 0; i < max_limit_switches.size(); i++)      //Defininig the pins for the max limit switches
    // {
    //     pinMode(max_limit_switches[i], INPUT);              //Sets each max limit switch as an input
    // }

    //Initialize Servo Motor
    const int Pwm_pin = 23;              //GPIO hardware PWM pin
    wiringPiSetup();                     //Access the wiringPi library
    pinMode(Pwm_pin, PWM_OUTPUT);        //Set the PWM pin as a PWM OUTPUT
    pwmSetMode(PWM_MODE_MS);

    int PWM_prescaler = 48;     //Base freq is 19.2 MHz, THIS IS THE VALUE U CHANGE 
    int pwm_range = 1000 * 48 / PWM_prescaler;
    int min_Pwm = pwm_range/5;
    
    pwmSetClock(PWM_prescaler);     //Setting up Pi PWM protocol
    pwmSetRange(pwm_range);         //Setting up Pi PWM protocol

    const float m = (480.0/180.0);        //THIS IS THE ACTUAL SLOPE WE USE
    const float b = 345;                  //THIS IS THE B WE USE, THIS IS ALSO THE PWM_HOME VALUE AKA 0 DEGREE EXTENSION
    const float Pwm_home_value = b;       //Home value, 0 degrees
    const float Pwm_max_value = m*105 + b;    //Max value, 105 degrees

    pwmWrite(Pwm_pin, Pwm_home_value);           //Commands the servo to the zero deg position, aka "home"
    delay(1000);


    // const float Pwm_init_val = 500;     //initial PWM value for the servo, should correspond to around halfway between home and max
    // pinMode(Pwm_pin, PWM_OUTPUT);       //setting the Pwm pin as a PWM output pin
    // pwmWrite(Pwm_pin, Pwm_init_val);    //Set the servo position to the initial PWM value, should be between home and max switches
    // Servo pad;                          //create instance of Servo

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
    auto motor_burn_time = chrono::high_resolution_clock::now();    //This gets reset once motor burn is detected

    //Set the status to PAD
    state.status = state_t::PAD;

    //Initial conditions for dynamics model and controller
    vector<int> theta_region[8501];
    vector<float> theta_vector[8501];
    float theta_0 = 20.0;
    float t = 0.0;
    float dt = 0.1;
    int num_integrated = 0;
    float x = 0.0;
    float U_airbrake = 0.0;


    //Initialize the dynamics model object and controller object
    dynamics_model dynamics;
    dynamics.init_model();
    controller airbrake;
    airbrake.init_controller(Pwm_home_value, Pwm_max_value);


    //While loop that runs until the APOGEE_DETECTED state is reach, aka this is run from being powered on the pad to apogee
    while (true && state.status != state_t::APOGEE_DETECTED)
    {
        cur = chrono::high_resolution_clock::now();
        state.imu_data = imu_read_data();
        //add a read from the altimeter here
        float z = 0.0;      //Replace this with the altimeter read

        //Switch cases that transition between the different states
        switch (state.status)
        {
            case state_t::PAD:
                //PAD_status() this function will call the PAD_status void function that will do the servo extension test
                PAD_status(Pwm_pin, Pwm_home_value, Pwm_max_value, state);
                break;

            case state_t::ARMED:
                //ARMED_status() this function will call the ARMED_status void function that will call the detect_launch function
                ARMED_status(state, launch_detect_log, launch_detect_log_index, start, launch_time);
                break;

            case state_t::LAUNCH_DETECTED:
                //LAUNCH_DETECTED_status() this function will call the LAUNCH_DETECTED_status void function that will call the detect_motor_burn_end function or maybe just a timer
                LAUNCH_DETECTED_status(state, motor_burn_time, theta_0, theta_region, theta_vector); //Needs more inputs i think
                pwmWrite(Pwm_pin, Pwm_home_value);        //Ensures the servo is at the home position
                break;

            case state_t::ACTUATION:
                //ACTUATION_status() this function will call the ACTUATION_status void function that will call the dynamics model and controller from Dynamics_Model_Controller folder, and a detect_apogee function
                ACTUATION_status(Pwm_pin, Pwm_home_value, Pwm_max_value, state, x, z, U_airbrake, theta_region, theta_vector);
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
void PAD_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state)
{
    float end_val = 0;
    for(int i = Pwm_home_value; i < Pwm_max_value; i = i + 10)
    {
        pwmWrite(Pwm_pin, i);
        delay(500);
        end_val = i;
    }
    delay(2000);
    for(int i = end_val; i > Pwm_home_value; i = i - 10)
    {
        pwmWrite(Pwm_pin, i);
        delay(500);
    }
    delay(500);
    pwmWrite(Pwm_pin, Pwm_home_value);

    state.status = state_t::ARMED;


    // pad.Servo_cal(Pwm_pin, Pwm_init_val, homing_limit_switches, max_limit_switches);
    // delay(1000);        //delay of 1 s
    // if(pad.get_Pwm_home_value() != pad.get_Pwm_init_value() && pad.get_Pwm_max_value() != pad.get_Pwm_init_value() && pad.get_Pwm_home_value() < pad.get_Pwm_init_value() && pad.get_Pwm_max_value() > pad.get_Pwm_init_value())
    // {
    //     state.status = state_t::ARMED;
    // }
}

void ARMED_status(state_t &state, pair<long, state_t> (&launch_detect_log)[1024], int &index, chrono::_V2::system_clock::time_point &start, chrono::_V2::system_clock::time_point &launch_time)
{    
    launch_detect_log[index & 1023] = make_pair(chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start).count(), state);

    if (detect_launch(launch_detect_log, index))
    {
        state.status = state_t::LAUNCH_DETECTED;
    }

    index++;

    return;
}

void LAUNCH_DETECTED_status(state_t &state, auto motor_burn_time, float &theta_0, vector<int> &theta_region, vector<float> &theta_vector)
{
    //Have motor burn detection function here, or simply a time delay equal to the motor burn + maybe 0.25 seconds?


    if (state.status == state_t::ACTUATION)     //This is going to set the initial theta_0 and form theta_region and theta_vector
    {
        theta_0 = sqrt(pow(state.imu_data.heading.x, 2) + pow(state.imu_data.heading.z, 2));      //CHECK THIS!!
        if (theta_0 > 35.0)       //This is just in case the angle reading is not accurate, just approximate it as 20 deg?
        {
            theta_0 = 20.0;
        
            auto ret = pitchanglevector(theta_0);
            vector<int> theta_region = ret.first;
            vector<float> theta_vector = ret.second;
        }
    }
}

void ACTUATION_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state, float x, float z, float U_airbrake, dynamics_model &dynamics, controller &airbrake, vector<int> theta_region, vector<float> theta_vector)
{
    float t = 0.0;
    float dt = 0.1;
    int num_integrated = 0;
    float x_dot = state.imu_data.velocity.x;        //Might not be able to call this, might have to have a function that calcs this
    float z_dot = state.imu_data.velocity.z;        //Might not be able to call this, might have to have a function that calcs this

    dynamics.init_model();
    dynamics.dynamics(t, x, z, x_dot, z_dot, dt, theta_region, theta_vector, U_airbrake, theta_region, theta_vector);
    float Mach = 0.6;   //THIS NEEDS TO BE CHANGED, PROLLY MAKE A FUNCTION THAT IS IDENTICAL TO THE ONE IN DRAG.CPP
    float output = airbrake.controller_loop(test.get_apogee_expected(), Mach, z);        //method that finds the airbrake output in PWM signal
    pwmWrite(Pwm_pin, output);
    U_airbrake = airbrake.get_airbrake_output();    //I think, should be the [0->1]
    //SHOULD BE MORE STUFF I THINK

}

void APOGEE_DETECTED_status()
{

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

pair<vector<int>, vector<float>> pitchanglevector(float theta_0)
{
    const static vector<float> m_theta{ 0.000525089, 0.000690884, 0.001009584, 0.001398228, 0.001801924 };    //Slopes for linear region, determined in excel

    vector<int> theta_region(8501);     //Size of theta region

    for (size_t i = 0; i < theta_region.size(); i++)        //Sets theta region from altitude of 2500 ft o 11k feet
    {
        theta_region[i] = i + 2500;
    }

    vector<float> theta_vector(theta_region.size());       //initializes theta vector, same size as theta region

    //Linear fit region, 2.5k ft to 7k ft
    float b;

    if (theta_0 <= 7)        //All of the if statements for theta_0
    {
        b = theta_0 - m_theta[0] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[0] * theta_region[i] + b;
        }
    }
    else if (theta_0 < 7 && theta_0 < 10)
    {
        b = theta_0 - m_theta[1] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[1] * theta_region[i] + b;
        }
    }
    else if (theta_0 >= 10 && theta_0 < 14)
    {
        b = theta_0 - m_theta[2] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[2] * theta_region[i] + b;
        }
    }
    else if (theta_0 >= 14 && theta_0 < 19)
    {
        b = theta_0 - m_theta[3] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[3] * theta_region[i] + b;
        }
    }
    else
    {
        b = theta_0 - m_theta[4] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[4] * theta_region[i] + b;
        }
    }
    //End of Linear fit region, ends at index 4500 at an altitude of 7k feet

    //Start of the Quadratic fit region, 7k ft to 10k ft
    vector<float> a_theta{ 8.26652482191255e-7, 1.03558936423213e-6, 1.53275631191493e-6, 2.17922684530253e-6, 2.92066636707301e-6 };

    int h_theta = 0;        //Parabola parameter for quadratic region
    float k_theta = theta_vector[4500];        //Initial value of quadratic region

    if (theta_0 < 7)     //if statements for the different initial thetas
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[0] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else if (theta_0 < 7 && theta_0 < 10)
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[1] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else if (theta_0 >= 10 && theta_0 < 14)
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[2] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else if (theta_0 >= 14 && theta_0 < 19)
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[3] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[4] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    //End of Quadratic fit region, ends at index 7500 at an altitude of 10k feet

    //Region after Quadratic region, increase linearly until 90 degrees at a steep slope
    float inc = 0.1;        //increment for the linear section past the quadratic region
    vector<float> int_vec(1000);      //interval vector initialization

    for (size_t i = 0; i < int_vec.size(); i++)     //interval vector definition, 1:1:1000
    {
        int_vec[i] = i + 1.0;
    }

    for (size_t i = 7501; i < theta_vector.size(); i++)     //adds the last linear section past quadratic region
    {
        theta_vector[i] = theta_vector[7500] + inc * int_vec[i - 7501];
    }

    return {theta_region,theta_vector};
}

float xdot_calc(state_t state)
{

}

float zdot_calc(state_t state)     //Also needs altitude for the derivative as input!!!
{

}
//