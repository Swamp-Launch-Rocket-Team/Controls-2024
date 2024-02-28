#include <iostream>
#include "drag.h"
#include "dynamics_model.h"
#include "controller.h"
#include "pi.h"
#include <chrono>

using namespace std;

unordered_map<int, float> pitchanglevector(float theta_0);       //USE THIS IN MAIN

// pair<vector<int>, vector<float>> pitchanglevector(float theta_0);       //USE THIS IN MAIN

int main()
{
    //Cd model testing

    // float z = 5005.6;       //In units of ft !!!!
    // float V_rocket = 256.3; //In units of ft/s!!!
    // float U_airbrake = 0;   // [0 -> 1]

    // Drag test(z,V_rocket,U_airbrake);

    // cout << test.get_Cd() << endl;



    //Dynamics model testing
    float theta_0 = 13.0;
    unordered_map<int, float> theta_map = pitchanglevector(theta_0);


    // auto ret = pitchanglevector(theta_0);
    // vector<int> theta_region = ret.first;
    // vector<float> theta_vector = ret.second;

    //Initial Conditions
    float x = 0.0;
	float z = 753;
	float x_dot = 58.38;
	float z_dot = 325;
    float U_airbrake = 0;

    
    // time step info
    float t = 0;
    float dt = 0.1;

    // tracking how many times its been integrated
    int num_integrated = 0;
    int i = 0;
    float Pwm_home_value = 345;      //in main.cpp this will be from pad.get_Pwm_home_value() getter
    float Pwm_max_value = 625;     //in main.cpp this will be from pad.get_Pwm_max_value() getter
    dynamics_model test(theta_map);
    test.init_model();
    controller airbrake;                //creates controller instance, probably don't name this test in main.cpp lol
    airbrake.init_controller(Pwm_home_value, Pwm_max_value);        //initializes the controller, this should only run once, probably at the end of the Launch Detected status

    auto t_start = chrono::high_resolution_clock::now();
    while (z_dot > 0)
    {
        auto cur = chrono::high_resolution_clock::now();
        // dynamics_model test;
        test.init_model();
        test.dynamics(t,x,z,x_dot,z_dot,dt,U_airbrake);
        cout << "Apogee expected:\t" <<test.get_apogee_expected() << endl;
        // cout << "New initial x:\t" << test.get_xinit() << endl;
        // cout << "New initial altitude:\t" << test.get_zinit() << endl;
        // cout << "New initial xdot:\t" << test.get_xdotinit() << endl;
        //cout << "New initial zdot:\t" << test.get_zdotinit() << endl;
        // cout << "Final zdot" << test.get_zdotfinal() << endl;

        float Pwm_home_value = 345;      //in main.cpp this will be from pad.get_Pwm_home_value() getter
        float Pwm_max_value = 625;     //in main.cpp this will be from pad.get_Pwm_max_value() getter 
        // controller airbrake;                //creates controller instance, probably don't name this test in main.cpp lol
        float Mach = 0.6;
        float altitude = 1000;
        // airbrake.init_controller(Pwm_home_value, Pwm_max_value);        //initializes the controller, this should only run once, probably at the end of the Launch Detected status
        float output = airbrake.controller_loop(test.get_apogee_expected(), Mach, altitude);        //method that finds the airbrake output in [0->1]
        cout << "Final controller output:" << "\t" << airbrake.get_airbrake_output() << endl;       //Prints airbrake controller output in PWM
        cout << "U_airbrake:\t" << output << endl;

        x = test.get_xinit();
        z = test.get_zinit();
        cout << "Altitude:\t" << z << endl;
        x_dot = test.get_xdotinit();
        z_dot = test.get_zdotinit();
        U_airbrake = output;
        i++;
        // t = t+dt;
        cout << i << endl;
        while (chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - cur).count() < 100);
    }

    cout << chrono::duration<double>(chrono::high_resolution_clock::now() - t_start).count() << endl;       //displays total runtime
    return 0;
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



// pair<vector<int>, vector<float>> pitchanglevector(float theta_0)
// {
//     const static vector<float> m_theta{ 0.000525089, 0.000690884, 0.001009584, 0.001398228, 0.001801924 };    //Slopes for linear region, determined in excel

//     vector<int> theta_region(8501);     //Size of theta region

//     for (size_t i = 0; i < theta_region.size(); i++)        //Sets theta region from altitude of 2500 ft o 11k feet
//     {
//         theta_region[i] = i + 2500;
//     }

//     vector<float> theta_vector(theta_region.size());       //initializes theta vector, same size as theta region

//     //Linear fit region, 2.5k ft to 7k ft
//     float b;

//     if (theta_0 <= 7)        //All of the if statements for theta_0
//     {
//         b = theta_0 - m_theta[0] * 2500;
//         for (int i = 0; i < 4501; i++)
//         {
//             theta_vector[i] = m_theta[0] * theta_region[i] + b;
//         }
//     }
//     else if (theta_0 < 7 && theta_0 < 10)
//     {
//         b = theta_0 - m_theta[1] * 2500;
//         for (int i = 0; i < 4501; i++)
//         {
//             theta_vector[i] = m_theta[1] * theta_region[i] + b;
//         }
//     }
//     else if (theta_0 >= 10 && theta_0 < 14)
//     {
//         b = theta_0 - m_theta[2] * 2500;
//         for (int i = 0; i < 4501; i++)
//         {
//             theta_vector[i] = m_theta[2] * theta_region[i] + b;
//         }
//     }
//     else if (theta_0 >= 14 && theta_0 < 19)
//     {
//         b = theta_0 - m_theta[3] * 2500;
//         for (int i = 0; i < 4501; i++)
//         {
//             theta_vector[i] = m_theta[3] * theta_region[i] + b;
//         }
//     }
//     else
//     {
//         b = theta_0 - m_theta[4] * 2500;
//         for (int i = 0; i < 4501; i++)
//         {
//             theta_vector[i] = m_theta[4] * theta_region[i] + b;
//         }
//     }
//     //End of Linear fit region, ends at index 4500 at an altitude of 7k feet

//     //Start of the Quadratic fit region, 7k ft to 10k ft
//     vector<float> a_theta{ 8.26652482191255e-7, 1.03558936423213e-6, 1.53275631191493e-6, 2.17922684530253e-6, 2.92066636707301e-6 };

//     int h_theta = 0;        //Parabola parameter for quadratic region
//     float k_theta = theta_vector[4500];        //Initial value of quadratic region

//     if (theta_0 < 7)     //if statements for the different initial thetas
//     {
//         for (int i = 4501; i < 7501; i++)
//         {
//             theta_vector[i] = a_theta[0] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
//         }
//     }
//     else if (theta_0 < 7 && theta_0 < 10)
//     {
//         for (int i = 4501; i < 7501; i++)
//         {
//             theta_vector[i] = a_theta[1] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
//         }
//     }
//     else if (theta_0 >= 10 && theta_0 < 14)
//     {
//         for (int i = 4501; i < 7501; i++)
//         {
//             theta_vector[i] = a_theta[2] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
//         }
//     }
//     else if (theta_0 >= 14 && theta_0 < 19)
//     {
//         for (int i = 4501; i < 7501; i++)
//         {
//             theta_vector[i] = a_theta[3] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
//         }
//     }
//     else
//     {
//         for (int i = 4501; i < 7501; i++)
//         {
//             theta_vector[i] = a_theta[4] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
//         }
//     }
//     //End of Quadratic fit region, ends at index 7500 at an altitude of 10k feet

//     //Region after Quadratic region, increase linearly until 90 degrees at a steep slope
//     float inc = 0.1;        //increment for the linear section past the quadratic region
//     vector<float> int_vec(1000);      //interval vector initialization

//     for (size_t i = 0; i < int_vec.size(); i++)     //interval vector definition, 1:1:1000
//     {
//         int_vec[i] = i + 1.0;
//     }

//     for (size_t i = 7501; i < theta_vector.size(); i++)     //adds the last linear section past quadratic region
//     {
//         theta_vector[i] = theta_vector[7500] + inc * int_vec[i - 7501];
//     }

//     return {theta_region,theta_vector};
// }