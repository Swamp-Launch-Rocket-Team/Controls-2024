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
#include <numeric>
#include <wiringPi.h>

//
//Here list the includes for the other header files used, IMU, altimeter, state, dynamics model, controller
#include "State/state.h"
#include "Dynamics_Model_Controller/controller.h"
#include "Dynamics_Model_Controller/dynamics_model.h"
#include "Dynamics_Model_Controller/pi.h"
#include "Dynamics_Model_Controller/drag.h"
#include "Altimiter/altimiter.h"
#include "IMU/imu.h"
#include "bitbang/bitbang.h"
#include "busynano/busynano.h"
#include "Logger/logger.h"

#define m_to_ft 3.28084
#define R 287.058
#define g 9.81
#define press_expected 101060.0        //Expected pressure at launch site in Pa, DEPENDENT          -- 
#define temp_expected 297.6            //Expected temperature at launch site in kelvin, DEPENDENT   --
#define press_min 20000.0              //Min for outlier of, DEPENDENT                              --
#define press_max 105000.0             //Max outlier for press, DEPENDENT                           --
#define temp_min 280.0                 //Min temp in KELVIN, DEPENDENT                              --
#define temp_max 305.15                 //Max temp in KELVIN, DEPENDENT                             --
#define lapse 0.0065                  //Lapse rate for pressure to altitude calc
#define D2R M_PI/180.0                  //Degrees to radians conversion
#define ZDOT_MAX 330.0                   //Max zdot, dynamics model fails at higher values (M > 1)
#define XDOT_MAX 80.0                   //Max xdot, dynamics model fails at higher values (M > 1)

//
//Here list the function prototypes and states
void PAD_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state, chrono::_V2::system_clock::time_point &cal);
void ARMED_status(state_t &state, chrono::_V2::system_clock::time_point start, chrono::_V2::system_clock::time_point &launch_time, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur, vector<float> &press_cal, vector<float> &temp_cal, float &T0, float &P0, int &launch_count);
void LAUNCH_DETECTED_status(state_t &state, chrono::_V2::system_clock::time_point &motor_burn_time, float &theta_0, chrono::_V2::system_clock::time_point launch_time, chrono::_V2::system_clock::time_point cur, unordered_map<int, float> &theta_map, int &ii);  //Needs more inputs i think
void ACTUATION_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state, float &x, float &U_airbrake, dynamics_model &dynamics, controller &airbrake, chrono::_V2::system_clock::time_point motor_burn_time, chrono::_V2::system_clock::time_point &apogee_time, chrono::_V2::system_clock::time_point cur, int &ii);  //Need to add other stuff
void DESCENT_STATUS(state_t &state, float Pwm_home_value, int Pwm_pin, chrono::_V2::system_clock::time_point &apogee_time, chrono::_V2::system_clock::time_point cur);
void LAND_STATUS(state_t &state, int Pwm_pin, float Pwm_home_value);


bool detect_launch(state_t state, int &launch_count);
unordered_map<int, float> pitchanglevector(float theta_0);       //USE THIS IN MAIN
void rotation(state_t &state);
void xdot_calc(state_t &state, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur, float loop_time, bool &velo_windowx, int &velo_counterx, int launch_count, int &end_velostuffx);
void zdot_calc(state_t &state, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur, float loop_time, bool &velo_windowz, int &velo_counterz, int launch_count, int &end_velostuffz);     
void Mach_calc(state_t &state);
float pressure_to_altitude(state_t &state, float T0, float P0, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur);
void pressure_filter(state_t &state, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur);
void theta_calc(state_t &state);
//

using namespace std;

int main()
{
    // Initialize the logging thread
    std::thread logger_thread(log_thread);

    //Initialize SPI bitbang
    spi_init_bitbang();
    
    //Initialize IMU
    imu_init();

    //Initialize Altimeter
    alt_init();

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
    const float b = 345.0;                  //THIS IS THE B WE USE, THIS IS ALSO THE PWM_HOME VALUE AKA 0 DEGREE EXTENSION
    const float Pwm_home_value = b;       //Home value, 0 degrees
    const float Pwm_max_value = m*105.0 + b;    //Max value, 105 degrees

    pwmWrite(Pwm_pin, Pwm_home_value);           //Commands the servo to the zero deg position, aka "home"
    delay(1000);



    //Store the airbrakes current state
    state_t state; // Stores information of airbrake state


    //Initialize and define variables for starting time and current time
    auto start = chrono::high_resolution_clock::now();
    auto cur = chrono::high_resolution_clock::now();
    auto cal = chrono::high_resolution_clock::now();
    auto launch_time = chrono::high_resolution_clock::now();        //This needs to get reset once launch is detected
    auto motor_burn_time = chrono::high_resolution_clock::now();    //This gets reset once motor burn is detected
    auto apogee_time = chrono::high_resolution_clock::now();    //This gets reset once apogee has been detected
    auto t_end = chrono::high_resolution_clock::now();          //This gets reset at the end of each loop in the switch statement
    auto measurement_prev = chrono::high_resolution_clock::now();   //Previous sensor measurement time
    auto measurement_cur = chrono::high_resolution_clock::now();    //Current sensor measurement time


    //Set the status to PAD
    state.status = state_t::PAD;


    //Initial conditions for dynamics model and controller
    // vector<int> theta_region = vector<int>(8501);
    // vector<float> theta_vector = vector<float>(8501);
    float theta_0 = 15.0;
    float x = 0.0;
    float U_airbrake = 0.0;
    int ii = 0;
    float loop_time = 0.0;


    //Initialize the dynamics model object and controller object
    // dynamics_model dynamics;
    // dynamics.init_model();
    unordered_map<int, float> theta_map = pitchanglevector(theta_0);
    dynamics_model dynamics(theta_map);
    dynamics.init_model();
    controller airbrake;
    airbrake.init_controller(Pwm_home_value, Pwm_max_value);


    //Vectors for pressure and temperature calibration
    vector<float> press_cal;
    vector<float> temp_cal;
    float T0 = temp_expected;
    float P0 = press_expected;
    int cal_count = 0;
    int launch_count = 0;

    bool first_loop = true;
    bool velo_windowx = 1;        //int for determining velo window while in ARMED state
    bool velo_windowz = 1;        //int for determining velo window while in ARMED state

    int velo_counterx = 0;       //int for checking the velo window counter while in ARMED state
    int velo_counterz = 0;       //int for checking the velo window counter while in ARMED state
    int end_velostuffx = 0;      //int for determining if we are in that weird period during ARMED where we havent detected launch, but launch has occured
    int end_velostuffz = 0;      //int for determining if we are in that weird period during ARMED where we havent detected launch, but launch has occured

    for (int i = 0; i < 100; i++)
    {
        imu_read_data();
    }


    //While loop that runs until the APOGEE_DETECTED state is reach, aka this is run from being powered on the pad to apogee
    while (true && state.status != state_t::LAND)
    {
        cur = chrono::high_resolution_clock::now();

        if (chrono::duration<double>(cur-cal).count() >= 120.0 && cal_count == 0 && state.status == state_t::ARMED)
        {
            state.altimeter.filt_pressure4 = P0;
            state.altimeter.filt_pressure3 = P0;
            state.altimeter.filt_pressure2 = P0;
            state.altimeter.filt_pressure1 = P0;
            state.altimeter.filt_pressure = P0;
            cal_count = 1;
        }
        state.imu_data = imu_read_data();       //Read from imu and write to the state imu data struct
        theta_calc(state);
        // Altimiter read
        altimiter_t alt_data = get_temp_and_pressure(); 
        state.altimeter.pressure = alt_data.pressure*100.0;;
        state.altimeter.temp = alt_data.temp + 273.15;
        measurement_cur = chrono::high_resolution_clock::now();     //Set current time to the current measurement time
        if (!first_loop)
        {
            loop_time = chrono::duration<double>(measurement_cur - measurement_prev).count();   //accurate loop time is the exact time between measurements
        }
        measurement_prev = measurement_cur;     //Set previous measurement time to the current measurement time
        first_loop = false;
        pressure_filter(state, cal, cur);
        state.altimeter.z = pressure_to_altitude(state, T0, P0, cal, cur);
        
        //Find the current xdot, zdot, and Mach number
        xdot_calc(state, cal, cur, loop_time, velo_windowx, velo_counterx, launch_count, end_velostuffx);       //in m/s i think,
        zdot_calc(state, cal, cur, loop_time, velo_windowz, velo_counterz, launch_count, end_velostuffz);       //in m/s i think,
        Mach_calc(state);       //dimensionless

        //Initialize the dynamics model object if switched to Actuation state
        if (ii == 1)
        {
            dynamics_model dynamics(theta_map);
            dynamics.init_model();
        }

        //Switch cases that transition between the different states
        switch (state.status)
        {
            case state_t::PAD:
                //PAD_status() this function will call the PAD_status void function that will do the servo extension test
                PAD_status(Pwm_pin, Pwm_home_value, Pwm_max_value, state, cal);
                break;

            case state_t::ARMED:
                //ARMED_status() this function will call the ARMED_status void function that will call the detect_launch function
                ARMED_status(state, start, launch_time, cal, cur, press_cal, temp_cal, T0, P0, launch_count);
                break;

            case state_t::LAUNCH_DETECTED:      
                //LAUNCH_DETECTED_status() this function will call the LAUNCH_DETECTED_status void function that will call the detect_motor_burn_end function or maybe just a timer
                LAUNCH_DETECTED_status(state, motor_burn_time, theta_0, launch_time, cur, theta_map, ii); //Needs more inputs i think
                pwmWrite(Pwm_pin, Pwm_home_value);        //Ensures the servo is at the home position
                break;

            case state_t::ACTUATION:        
                //ACTUATION_status() this function will call the ACTUATION_status void function that will call the dynamics model and controller from Dynamics_Model_Controller folder, and a detect_apogee function
                ACTUATION_status(Pwm_pin, Pwm_home_value, Pwm_max_value, state, x, U_airbrake, dynamics, airbrake, motor_burn_time, apogee_time, cur, ii);
                break;

            case state_t::DESCENT:
                DESCENT_STATUS(state, Pwm_home_value, Pwm_pin, apogee_time, cur);
                break;
        }

        //Delay so that IMU and altimeter do not read too fast
        while (chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - cur).count() < 9500);

        log_state(state, start, P0, T0, loop_time, velo_windowx, velo_counterx, theta_0);
            
        // cout << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << "\t" << state.status << "\t" << state.altimeter.z << "\t" << state.theta << "\t" << state.velo.integral_velox << "\t" << state.velo.integral_veloz << "\t" << state.airbrake.apogee_expected << "\t" << state.airbrake.U_airbrake << "\t" << chrono::duration<double>(chrono::high_resolution_clock::now() - motor_burn_time).count() << "\t" << chrono::duration<double>(cur - apogee_time).count() << endl;
    }

    //Write the data, command the servo to return to the home position, put the Pi to sleep
    LAND_STATUS(state, Pwm_pin, Pwm_home_value);

    // End logging thread
    log_stop();
    logger_thread.join();
    
    // cout << "Testing done" << endl;     //For debugging, REMOVE PRIOR TO FLIGHT

    return 0;
}

//
//Add all of the void state status functions here
void PAD_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state, chrono::_V2::system_clock::time_point &cal)
{
    float end_val = 0;
    for(int i = Pwm_home_value; i < Pwm_max_value; i = i + 10)
    {
        pwmWrite(Pwm_pin, i);
        delay(100);
        end_val = i;
    }
    delay(2000);
    for(int i = end_val; i > Pwm_home_value; i = i - 10)
    {
        pwmWrite(Pwm_pin, i);
        delay(100);
    }
    delay(500);
    pwmWrite(Pwm_pin, Pwm_home_value);

    state.status = state_t::ARMED;
    cal = chrono::high_resolution_clock::now();

    return;
}

void ARMED_status(state_t &state, chrono::_V2::system_clock::time_point start, chrono::_V2::system_clock::time_point &launch_time, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur, vector<float> &press_cal, vector<float> &temp_cal, float &T0, float &P0, int &launch_count)
{   

    if (chrono::duration<double>(cur - cal).count() < 120.0)        //Add pressure and temp to calibration vectors if time less than 2 mins
    {
        if (state.altimeter.temp > temp_max || state.altimeter.temp < temp_min) //Check to make sure temp is in expected range
        {
            state.altimeter.temp = temp_expected;       //set temp to expected value if not in expected ranged
        }
        
        press_cal.push_back(state.altimeter.pressure);  //Add the pressure value to the pressure cal vector
        temp_cal.push_back(state.altimeter.temp);       //Add the temp value to the temp cal vector

        P0 = (accumulate(press_cal.begin(),press_cal.end(),0.0))/press_cal.size();  //Find the average pressure and set to ground pressure
        T0 = (accumulate(temp_cal.begin(),temp_cal.end(),0.0)/temp_cal.size());     //Find the average temp and set to ground temp   

    }

    if (detect_launch(state, launch_count))
    {
        state.status = state_t::LAUNCH_DETECTED;
        launch_time = chrono::high_resolution_clock::now();
    }

    return;
}

void LAUNCH_DETECTED_status(state_t &state, chrono::_V2::system_clock::time_point &motor_burn_time, float &theta_0, chrono::_V2::system_clock::time_point launch_time, chrono::_V2::system_clock::time_point cur, unordered_map<int, float> &theta_map, int &ii)
{
    //Have motor burn detection function here, or simply a time delay equal to the motor burn + maybe 0.25 seconds?

    //Time delay implementation
    float t_burn_expected = 4.5;    //Reported motor burn time from OpenRocket/Aerotech, DEPENDENT
    if (chrono::duration<double>(cur - launch_time).count() >= (t_burn_expected + 0.25))
    {
        motor_burn_time = chrono::high_resolution_clock::now();
        state.status = state_t::ACTUATION;
    }


    if (state.status == state_t::ACTUATION)     //This is going to set the initial theta_0 and form theta_region and theta_vector
    {
        theta_0 = abs(state.theta);      //CHECK THIS!!
        if (theta_0 > 35.0)       //This is just in case the angle reading is not accurate, just approximate it as 15 deg?
        {
            theta_0 = 15.0;
        }
        ii = 1;
        theta_map = pitchanglevector(theta_0);

        // auto ret = pitchanglevector(theta_0);
        // theta_region = ret.first;
        // theta_vector = ret.second;
    }

    return;
}

void ACTUATION_status(int Pwm_pin, float Pwm_home_value, float Pwm_max_value, state_t &state, float &x, float &U_airbrake, dynamics_model &dynamics, controller &airbrake, chrono::_V2::system_clock::time_point motor_burn_time, chrono::_V2::system_clock::time_point &apogee_time, chrono::_V2::system_clock::time_point cur, int &ii)
{
    // auto t_start = chrono::high_resolution_clock::now();
    ii = 2;

    float t_min = 16.0;     //minimum time expected to apogee from end of motor burn, DEPENDENT         --
    float t_max = 26.0;     //max time expected to apogee from end of motor burn, DEPENDENT             --

    float t = 0.0;
    float dt = 0.1;
    int num_integrated = 0;
    float x_dot = abs(state.velo.xdot);        
    float z_dot = abs(state.velo.zdot);
    if (z_dot > ZDOT_MAX)
    {
        z_dot = ZDOT_MAX;
    }
    if (x_dot > XDOT_MAX)
    {
        x_dot = XDOT_MAX;
    }        
    float z = abs(state.altimeter.z);
    float a = -0.004*z*m_to_ft + 1116.45;       //From the drag model
    float v_rocket = sqrt(pow(z_dot,2) + pow(x_dot,2))*m_to_ft;
    float cur_mach = v_rocket/a;
    while (cur_mach >= 0.98)        //Need to decrease velocity to get mach below 0.98 so dynamics model does not have an error
    {
        if (x_dot > 0.0)
        {
            x_dot -= 1.0;       //We are assuming zdot is accurate and xdot is the issue
        }
        else
        {
            z_dot -= 1.0;
        }
        v_rocket = sqrt(pow(z_dot,2) + pow(x_dot,2))*m_to_ft;
        cur_mach = v_rocket/a;
    }

    if (chrono::duration<double>(cur - motor_burn_time).count() >= t_max)      //if the time since motor burn end is greater than 24 seconds, we should have hit apogee
    {
        state.status = state_t::DESCENT;
        apogee_time = chrono::high_resolution_clock::now();
    }
    else if ((state.velo.zdot <= 0 && state.velo.zdot_1 <= 0) && chrono::duration<double>(cur - motor_burn_time).count() >= t_min)   //if zdot is negative and time since motor burn end is greater than 16 seconds, it is reasonable that we hit apogee
    {
        state.status = state_t::DESCENT;
        apogee_time = chrono::high_resolution_clock::now();
    }
    else    //We have not hit apogee, continue with regular actuation algorithm
    {
        dynamics.init_model();
        dynamics.dynamics(t, x, z, x_dot, z_dot, dt, U_airbrake);
        float Mach = state.velo.Mach;
        U_airbrake = airbrake.controller_loop(dynamics.get_apogee_expected(), Mach, z);        //method that finds the airbrake output in [0->1]
        state.airbrake.apogee_expected = dynamics.get_apogee_expected();
        float output = airbrake.get_airbrake_output();    //Output PWM signal
        state.airbrake.U_airbrake = output;
        state.airbrake.prev_error = airbrake.get_prev_error();
        if (output > Pwm_max_value || output < Pwm_home_value)
        {
            pwmWrite(Pwm_pin, Pwm_home_value);
        }
        else
        {
            pwmWrite(Pwm_pin, output);
        }
    }

    while (chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - cur).count() < 100);     //Delay to make sure the total time is no less than 100 ms

    return;
}

void DESCENT_STATUS(state_t &state, float Pwm_home_value, int Pwm_pin, chrono::_V2::system_clock::time_point &apogee_time, chrono::_V2::system_clock::time_point cur)
{
    pwmWrite(Pwm_pin, Pwm_home_value);
    if (chrono::duration<double>(cur - apogee_time).count() >= 180.0)
    {
        state.status = state_t::LAND;
    }
}


void LAND_STATUS(state_t &state, int Pwm_pin, float Pwm_home_value)
{
    pwmWrite(Pwm_pin, Pwm_home_value);
    delay(500);

    return;
}
//

//
//Add all of the other functions here: launch detect, maybe servo arming, detect motor burn end, detect apogee, write data from Jasons main, 
bool detect_launch(state_t state, int &launch_count)
{
    float detection_acceleration = 5.0 * 9.81; 
    // fix bs way too high numbers by setting to 0

    if (launch_count > 4) {
        return true;
    }

    float x_accel = state.imu_data.accel.x;
    float y_accel = state.imu_data.accel.y;
    float z_accel = state.imu_data.accel.z;

    float max_accel = 9.81*16.0;

    if (x_accel > max_accel) {
        x_accel = 0.0;
    }
    if (y_accel > max_accel) {
        y_accel = 0.0;
    }
    if (z_accel > max_accel) {
        z_accel = 0.0;
    }

    // z_accel -= 9.81;     //Dont need anymore, the IMU reports acceleration relative to Earth's Gravity!

    float total_accel = sqrt(pow(x_accel, 2) + pow(y_accel, 2) + pow(z_accel, 2));

    if (total_accel >= detection_acceleration) {
        launch_count += 1;
    }
    else {
        if (launch_count > 0) {
            launch_count -= 1;
        }
    }

    return false;
}

unordered_map<int, float> pitchanglevector(float theta_0)
{
    const static vector<float> m_theta{ 0.000525089, 0.000690884, 0.001009584, 0.001398228, 0.001801924 };    //Slopes for linear region, determined in excel


    int min_altitude = 2500;
    int max_altitude = 11000;
    int linear_region_end = 7000;
    int quadratic_region_end = 10000;

    unordered_map<int, float> theta_map;

    // begin linear fit region 2.5k to 7k feet
    int slope_index;
    for (int i =  min_altitude; i <= linear_region_end; i++) {
        if (theta_0 <= 7)        //All of the if statements for theta_0
        {
            slope_index = 0;
        }
        else if (theta_0 < 7 && theta_0 < 10)
        {
            slope_index = 1;
        }
        else if (theta_0 >= 10 && theta_0 < 14)
        {
            slope_index = 2;
        }
        else if (theta_0 >= 14 && theta_0 < 19)
        {
            slope_index = 3;
        }
        else {
            slope_index = 4;
        }

        theta_map[i] = m_theta[slope_index] * i + (theta_0 - m_theta[slope_index]*2500);

    }
    //End of Linear fit region, ends at index 4500 at an altitude of 7k feet

    //Start of the Quadratic fit region, 7k ft to 10k ft
    vector<float> a_theta{ 8.26652482191255e-7, 1.03558936423213e-6, 1.53275631191493e-6, 2.17922684530253e-6, 2.92066636707301e-6 };

    int h_theta = 0;        //Parabola parameter for quadratic region
    float k_theta = theta_map[linear_region_end];        //Initial value of quadratic region

    for (int i = linear_region_end + 1; i <= quadratic_region_end; i++) {
        if (theta_0 <= 7)        //All of the if statements for theta_0
        {
            slope_index = 0;
        }
        else if (theta_0 > 7 && theta_0 < 10)
        {
            slope_index = 1;
        }
        else if (theta_0 >= 10 && theta_0 < 14)
        {
            slope_index = 2;
        }
        else if (theta_0 >= 14 && theta_0 < 19)
        {
            slope_index = 3;
        }
        else {
            slope_index = 4;
        }

        // it was - -h_theta
        theta_map[i] = a_theta[slope_index] * pow((i - 7000 + h_theta), 2) + k_theta;

    }
    //End of Quadratic fit region, ends at index 7500 at an altitude of 10k feet

    //Region after Quadratic region, increase linearly until 90 degrees at a steep slope

    float inc = 0.1; // increment for linear section past quadratic region
    for (int i = quadratic_region_end + 1; i <= max_altitude; i++) { //adds the last linear section past quadratic region
        theta_map[i] = theta_map[quadratic_region_end] + inc * (i - quadratic_region_end);
    }

    return theta_map;
}

void rotation(state_t &state)
{
    //Rotation matrix for accel data from rocket frame to ground frame
}

float pressure_to_altitude(state_t &state, float T0, float P0, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur)
{
    if (state.status != state_t::PAD && state.status != state_t::ARMED)     
    {
    float pressure = state.altimeter.filt_pressure;      //In Pa
    float altitude = (T0/lapse)*(1-pow(pressure/P0,lapse*(R/g)));       
    state.altimeter.z_prev = state.altimeter.z;
    return altitude;
    }
    else
    {
        float altitude = 0.0;
        return altitude;
    }
}

void xdot_calc(state_t &state, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur, float loop_time, bool &velo_windowx, int &velo_counterx, int launch_count, int &end_velostuffx)
{

    //If statement that checks which state we are in
    //For pad and armed states -> set all xdot 0

    if (chrono::duration<double>(cur - cal).count() > 120.0 && end_velostuffx == 0)
    {
        if (velo_counterx == 6)
        {
            if (detect_launch(state, launch_count))
            {
                end_velostuffx = 1;
                //Do actual math for velocity and set that to the state.velo.integral_veloz AND state.velo.prev_integral_veloz
                if (velo_windowx == 0)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_velox = (state.dt_window1[i]/2.0)*(state.accel_window1[i + 1]*sin(state.theta_window1[i + 1]*D2R) + state.accel_window1[i]*sin(state.theta_window1[i]*D2R)) + state.velo.prev_integral_velox;
                        state.velo.prev_integral_velox = state.velo.integral_velox;
                    }

                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_velox = (state.dt_window2[i]/2.0)*(state.accel_window2[i + 1]*sin(state.theta_window2[i + 1]*D2R) + state.accel_window2[i]*sin(state.theta_window2[i]*D2R)) + state.velo.prev_integral_velox;
                        state.velo.prev_integral_velox = state.velo.integral_velox;
                    }

                }
                else
                {
                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_velox = (state.dt_window2[i]/2.0)*(state.accel_window2[i + 1]*sin(state.theta_window2[i + 1]*D2R) + state.accel_window2[i]*sin(state.theta_window2[i]*D2R)) + state.velo.prev_integral_velox;
                        state.velo.prev_integral_velox = state.velo.integral_velox;
                    }

                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_velox = (state.dt_window1[i]/2.0)*(state.accel_window1[i + 1]*sin(state.theta_window1[i + 1]*D2R) + state.accel_window1[i]*sin(state.theta_window1[i]*D2R)) + state.velo.prev_integral_velox;
                        state.velo.prev_integral_velox = state.velo.integral_velox;
                    }
                }
            }
            else
            {
                velo_windowx = !velo_windowx;     //switch the accel window
            }
        }

        if (velo_windowx)
        {
            state.accel_window1[velo_counterx] = state.imu_data.accel.z;
            state.dt_window1[velo_counterx] = loop_time;
            state.theta_window1[velo_counterx] = state.theta;
        }
        else
        {
            state.accel_window2[velo_counterx] = state.imu_data.accel.z;
            state.dt_window2[velo_counterx] = loop_time;
            state.theta_window2[velo_counterx] = state.theta;
        }
        
        if (velo_counterx == 6)
        {
            velo_counterx = 0;
        }
        else
        {
            velo_counterx += 1;
        }

    }

    if (state.status == state_t::PAD)
    {
        //set all zdot to 0
        state.velo.xdot_1 = 0.0;
        state.velo.xdot_2 = 0.0;
        state.velo.xdot_3 = 0.0;
        state.velo.xdot_4 = 0.0;
        state.velo.xdot = 0.0;
    }
    else if ((state.status == state_t::LAUNCH_DETECTED || state.status == state_t::ACTUATION) && end_velostuffx == 1)
    {
        state.velo.integral_velox = (loop_time/2.0)*(state.prev_accelz*sin(state.theta*D2R) + state.imu_data.accel.z*sin(state.theta*D2R)) + state.velo.prev_integral_velox;
        state.velo.prev_integral_velox = state.velo.integral_velox;     

        state.velo.xdot = state.velo.integral_velox;
        state.velo.xdot_4 = state.velo.xdot_3;
        state.velo.xdot_3 = state.velo.xdot_2;
        state.velo.xdot_2 = state.velo.xdot_1;
        state.velo.xdot_1 = state.velo.xdot;
    }
    
}

void zdot_calc(state_t &state, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur, float loop_time, bool &velo_windowz, int &velo_counterz, int launch_count, int &end_velostuffz)     //Also needs altitude for the derivative as input!!!
{
    //If statement that checks which state we are in
    //For pad and armed states -> set all zdots to 0
    //For launch detected state -> use derivative and MAYBE filter it


    if (chrono::duration<double>(cur - cal).count() > 120.0 && end_velostuffz == 0)
    {
        if (velo_counterz == 6)
        {
            if (detect_launch(state, launch_count))
            {
                end_velostuffz = 1;
                //Do actual math for velocity and set that to the state.velo.integral_veloz AND state.velo.prev_integral_veloz
                if (velo_windowz == 0)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_veloz = (state.dt_window1[i]/2.0)*(state.accel_window1[i + 1]*cos(state.theta_window1[i + 1]*D2R) + state.accel_window1[i]*cos(state.theta_window1[i]*D2R)) + state.velo.prev_integral_veloz;
                        state.velo.prev_integral_veloz = state.velo.integral_veloz;
                    }

                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_veloz = (state.dt_window2[i]/2.0)*(state.accel_window2[i + 1]*cos(state.theta_window2[i + 1]*D2R) + state.accel_window2[i]*cos(state.theta_window2[i]*D2R)) + state.velo.prev_integral_veloz;
                        state.velo.prev_integral_veloz = state.velo.integral_veloz;
                    }

                }
                else
                {
                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_veloz = (state.dt_window2[i]/2.0)*(state.accel_window2[i + 1]*cos(state.theta_window2[i + 1]*D2R) + state.accel_window2[i]*cos(state.theta_window2[i]*D2R)) + state.velo.prev_integral_veloz;
                        state.velo.prev_integral_veloz = state.velo.integral_veloz;
                    }

                    for (int i = 0; i < 6; i++)
                    {
                        state.velo.integral_veloz = (state.dt_window1[i]/2.0)*(state.accel_window1[i + 1]*cos(state.theta_window1[i + 1]*D2R) + state.accel_window1[i]*cos(state.theta_window1[i]*D2R)) + state.velo.prev_integral_veloz;
                        state.velo.prev_integral_veloz = state.velo.integral_veloz;
                    }
                }
            }
            else
            {
                velo_windowz = !velo_windowz;     //switch the accel window
            }
        }

        if (velo_windowz)
        {
            state.accel_window1[velo_counterz] = state.imu_data.accel.z;
            state.dt_window1[velo_counterz] = loop_time;
            state.theta_window1[velo_counterz] = state.theta;
        }
        else
        {
            state.accel_window2[velo_counterz] = state.imu_data.accel.z;
            state.dt_window2[velo_counterz] = loop_time;
            state.theta_window2[velo_counterz] = state.theta;
        }
        
        if (velo_counterz == 6)
        {
            velo_counterz = 0;
        }
        else
        {
            velo_counterz += 1;
        }

    }

    if (state.status == state_t::PAD)
    {
        //set all zdot to 0
        state.velo.zdot_1 = 0.0;
        state.velo.zdot_2 = 0.0;
        state.velo.zdot_3 = 0.0;
        state.velo.zdot_4 = 0.0;
        state.velo.zdot = 0.0;
    }
    else if ((state.status == state_t::LAUNCH_DETECTED || state.status == state_t::ACTUATION || state.status == state_t::DESCENT) && end_velostuffz == 1)
    {
        //Take the derivative of the altitude using state.altimeter.z and state.altimeter.z_prev
        state.velo.zdot = (state.altimeter.z - state.altimeter.z_prev)/loop_time;       //in [m/s]

        state.velo.zdot_4 = state.velo.zdot_3;
        state.velo.zdot_3 = state.velo.zdot_2;
        state.velo.zdot_2 = state.velo.zdot_1;
        state.velo.zdot_1 = state.velo.zdot;

        state.velo.integral_veloz = (loop_time/2.0)*(state.prev_accelz*cos(state.theta*D2R) + state.imu_data.accel.z*cos(state.theta*D2R)) + state.velo.prev_integral_veloz;
        state.velo.prev_integral_veloz = state.velo.integral_veloz;     //CHECK THIS
        state.prev_accelz = state.imu_data.accel.z;
    }
    
}

void Mach_calc(state_t &state)
{
    if (state.status == state_t::PAD || state.status == state_t::ARMED)
    {
        state.velo.Mach = 0.0;
    }
    else
    {
        float V_rocket = (sqrt(pow(state.velo.xdot, 2) + pow(state.velo.zdot,2)))*m_to_ft;  //in ft/s
        float h = 13.0 + state.altimeter.z*m_to_ft;   //in ft, first value is DEPENDENT on the launch site          --
        float a = -0.004 * h + 1116.45;
        state.velo.Mach = V_rocket/a;       //Mach number
    }
}


void pressure_filter(state_t &state, chrono::_V2::system_clock::time_point cal, chrono::_V2::system_clock::time_point cur)
{

    //If we are still in the calibration part
    if (state.status == state_t::ARMED && chrono::duration<double>(cur-cal).count() < 120.0)
    {
        if (state.altimeter.pressure > press_max || state.altimeter.pressure < press_min)
        {
            state.altimeter.pressure = press_expected; 
        }

        state.altimeter.pressure4 = state.altimeter.pressure3;
        state.altimeter.pressure3 = state.altimeter.pressure2;
        state.altimeter.pressure2 = state.altimeter.pressure1;
        state.altimeter.pressure1 = state.altimeter.pressure;
        
    }

    //If we are out of the calibration part, filter the pressures using the butterworth filter
    if (state.status != state_t::PAD && chrono::duration<double>(cur - cal).count() > 120.0)
    {
        if (state.altimeter.pressure > press_max || state.altimeter.pressure < press_min)
        {
            state.altimeter.pressure = (state.altimeter.pressure1 - state.altimeter.pressure2) + state.altimeter.pressure1; 
        }   

        // state.altimeter.filt_pressure = state.altimeter.filt_pressure1*1.561 - state.altimeter.filt_pressure2*0.6414 + state.altimeter.pressure*0.0201 + state.altimeter.pressure1*0.0402 + state.altimeter.pressure2*0.0201;
        state.altimeter.filt_pressure = state.altimeter.filt_pressure1*1.143 - state.altimeter.filt_pressure2*0.4128 + state.altimeter.pressure*0.0675 + state.altimeter.pressure1*0.1349 + state.altimeter.pressure2*0.0675;
        state.altimeter.filt_pressure4 = state.altimeter.filt_pressure3;
        state.altimeter.filt_pressure3 = state.altimeter.filt_pressure2;
        state.altimeter.filt_pressure2 = state.altimeter.filt_pressure1;
        state.altimeter.filt_pressure1 = state.altimeter.filt_pressure;

        state.altimeter.pressure4 = state.altimeter.pressure3;
        state.altimeter.pressure3 = state.altimeter.pressure2;
        state.altimeter.pressure2 = state.altimeter.pressure1;
        state.altimeter.pressure1 = state.altimeter.pressure;

        if (state.altimeter.temp > temp_max || state.altimeter.temp < temp_min)
        {
            state.altimeter.temp = abs((state.altimeter.temp1 - state.altimeter.temp2)) + state.altimeter.temp1;
        }

        state.altimeter.temp2 = state.altimeter.temp1;
        state.altimeter.temp1 = state.altimeter.temp;

    }
}

void theta_calc(state_t &state)
{

    state.theta = abs(90.0 - state.imu_data.heading.y);
    // state.theta_magway = sqrt(pow(state.imu_data.heading.x,2) + pow(state.imu_data.heading.y,2));
    // state.imu_data.heading.y -= 90.0;
    // state.imu_data.heading.x += 180.0;
    // state.theta = abs(atan(sqrt(pow(tan(state.imu_data.heading.x*M_PI/180.0),2) + pow(tan(state.imu_data.heading.y*M_PI/180.0),2)))*180.0/M_PI);
    
    
    // state.theta = abs(90.0 - atan(sqrt(pow(tan(state.imu_data.heading.x*M_PI/180.0),2) + pow(tan(state.imu_data.heading.y*M_PI/180.0),2)))*180.0/M_PI);
}