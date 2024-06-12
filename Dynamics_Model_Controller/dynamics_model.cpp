#include <cmath>
#include <math.h>
#include <algorithm>
#include <vector>
#include "dynamics_model.h"
#include "drag.h"
#include <iostream>

using namespace std;

//defines
#define g 9.81
#define m 32.6     //DEPENDENT      --
// #define theta_0 13
#define m_to_ft 3.28084
#define pi 3.14159265


dynamics_model::dynamics_model(unordered_map<int, float> theta_map_in)
{
    theta_map = theta_map_in;
}

void dynamics_model::init_model()
{
    this->model_params.t = 0.0;
    this->model_params.num_integrated = 0;
    // this->model_params.xinit = 0.0;
    // this->model_params.zinit = 0.0;
    // this->model_params.xdotinit = 0.0;
    // this->model_params.zdotinit = 0.0;
}

float dynamics_model::dynamics(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, float &U_airbrake)
{
    while (z_dot > 0) {

        rk4_integrate(t, x, z, x_dot, z_dot, dt, U_airbrake);
        
        t += dt;
        this->model_params.t = t;

        this->model_params.num_integrated += 1;
        if (get_num_integrated() == 1)
        {
            U_airbrake = 0;
            this->model_params.xinit = x;
            this->model_params.zinit = z;
            this->model_params.xdotinit = x_dot;
            this->model_params.zdotinit = z_dot;
        }
	}

    this->model_params.xfinal = x;
    this->model_params.apogee_expected = z;
    this->model_params.xdotfinal = x_dot;
    this->model_params.zdotfinal = z_dot;
    return z;
}

void dynamics_model::rk4_integrate(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, float U_airbrake)
{
    vector<float> k1(4), k2(4), k3(4), k4(4);

    model(t, x, z, x_dot, z_dot, k1, U_airbrake);
    model(t+0.5*dt, x + 0.5 * dt * k1[0], z+0.5*dt*k1[1], x_dot+0.5*dt*k1[2], z_dot + 0.5*dt*k1[3], k2, U_airbrake);
    model(t+0.5*dt, x + 0.5 * dt * k2[0], z + 0.5 * dt * k2[1], x_dot + 0.5 * dt * k2[2], z_dot + 0.5 * dt * k2[3], k3, U_airbrake);
    model(t+dt, x + dt * k3[0], z + dt * k3[1], x_dot + dt * k3[2], z_dot + dt * k3[3], k4, U_airbrake);

    x += dt * 0.1666 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    z += dt * 0.1666 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    x_dot += dt * 0.1666 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    z_dot += dt * 0.1666 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
}

void dynamics_model::model(float t, float x, float z, float x_dot, float z_dot, vector<float> &output, float U_airbrake)
{
    float pitch_angle = Calc_pitch_angle(z * m_to_ft);

	//cout << "Pitch angle (deg): " << pitch_angle*180/pi << endl;

	float rho = Calc_rho(z);

	//cout << "Rho: " << rho<< endl;

	float v_rocket = Calc_v_rocket(z_dot, x_dot);

	//cout << "V_rocket: " << v_rocket << endl;

	//float U_airbrake = Calc_U_airbrake(t);  // dont think we need this anymore
	float cd = Calc_cd(z*m_to_ft, v_rocket*m_to_ft, U_airbrake);

	// cout << "cd: " << cd << endl;
	//cout << endl;

	float area = Calc_area(U_airbrake);

	float drag = Calc_drag(rho, pow(v_rocket, 2), cd, area);

	float x_float_dot = Calc_x_float_dot(pitch_angle, drag);
	float z_float_dot = Calc_z_float_dot(pitch_angle, drag);

	// Runge Kutta on x_dot, z_dot, z_float_dot, x_float_dot?

	output[0] = x_dot;
	output[1] = z_dot;
	output[2] = x_float_dot;
	output[3] = z_float_dot;
}

float dynamics_model::get_t()
{
    return this->model_params.t;
}

float dynamics_model::get_apogee_expected()
{
    return this->model_params.apogee_expected;
}

float dynamics_model::Calc_pitch_angle(float z)
{
    
    // doesnt check to see if the altitude exists but dont think i need that if the range is right
    int altitude_index = static_cast<int>(round(z)); //Finds the index for the current altitude
    if (altitude_index < 2500) {
        altitude_index = 2500;
    }
    else if (altitude_index > 11000) {
        altitude_index = 11000;
    }

    float theta_at_altitude = theta_map[altitude_index];


    // cout << altitude_index << endl;
    //float theta_at_altitude = theta_map[altitude_index];        //Finds the theta angle at the current altitude, THIS IS THE OUTPUT

    if (theta_at_altitude > 90)      //Sets any theta values above 90 degrees to 90
    {
        theta_at_altitude = 90;
    }

    //cout << altitude_index << "\t" << theta_at_altitude << endl;    //for testing with the matlab function

    // return 0;

    // Calc_x_float_dot uses sin() which takes in radians so this is converted
	float radian_theta = theta_at_altitude * pi * 0.00556;

	return radian_theta;
}

float dynamics_model::Calc_rho(float z)
{
    // lapse rate model goes here
	// 
	//float z = 3002.3;     //Altitude [m], THIS IS THE INPUT

	float T0 = 297.6;        //Temperature at ground level [K], DEPENDENT       --
	float L = 0.0065;      //Lapse rate 

	float T = T0 - L * z;        //Temperature at current altitude [K]

	float rho0 = 1.225;     //Air density at ground level, THIS NEEDS TO CHANGE BASED ON FLORIDA OR SPACEPORT LAUNCH SITE, DEPENDENT        --
	// float g = 9.81;        //Gravity [m/s^2]
	float R = 287.0531;    //Universal gas constant for air

	float rho = rho0 * pow(T / T0, (((g / (L * R))) - 1));     //Air density at altitude [kg/m^3], THIS IS THE OUTPUT

	// cout << rho << endl;

	//return 0;

	return rho;
}

float dynamics_model::Calc_v_rocket(float x_dot, float z_dot)
{
    return sqrt(pow(x_dot, 2) + pow(z_dot,2));
}

// float dynamics_model::Calc_U_airbrake(float t)
// {

// }

float dynamics_model::Calc_cd(float z, float v_rocket, float U_airbrake)
{
    // Coefficient of Drag function goes here:

	Drag d(z, v_rocket, U_airbrake);
	
	return d.get_Cd();
}

float dynamics_model::Calc_area(float U_airbrake)
{
    // Cross sectional area function goes here
	const float d_rocket = 6.14 * 0.0254;    //diameter of rocket in meters, 6.14 in = 0.1560 m
	const float A_rocket = (0.25) * (pi)*pow(d_rocket, 2); // my own version of M_PI

	// i have this as the input
	// float U_airbrake = 0.25;       //Input of the airbrake, this is from 0 -> 1, THIS WILL BE THE INPUT 

	float Servo_angle = 105.0 * U_airbrake;        //Servo angle in degrees

	float A_airbrake;

	if (U_airbrake == 0)
	{
		A_airbrake = 0;
	}
	else
	{
		//Max cross-sectional area of airbrake during extension in [m^2]
		A_airbrake = abs(0.00064516 * (8.48389446479259e-8 * pow(Servo_angle, 4) - 0.0000348194172718423 * pow(Servo_angle, 3) + 0.00388560760536394 * pow(Servo_angle, 2) - 0.0629348277080075 * Servo_angle + 0.148040926341214));
	}

	float A_cross = A_rocket + A_airbrake;     //Cross sectional area of the rocket, THIS IS THE OUTPUT

	// replaced M_PI
	// cout << A_cross << "\t" << pi << endl;

	//return 0;

	
	return A_cross;
}

float dynamics_model::Calc_drag(float rho, float v_rocket_squared, float cd, float area)
{
    return rho * v_rocket_squared * cd * .5 * area;
}

float dynamics_model::Calc_x_float_dot(float theta, float drag)
{
    float x_float_dot = (-1.0 / m) * sin(theta) * drag;

	return x_float_dot;
}

float dynamics_model::Calc_z_float_dot(float theta, float drag)
{
    float z_float_dot = (-1.0 / m) * ((drag*cos(theta)) + (m*g));

	return z_float_dot;
}

float dynamics_model::get_xinit()      //for debugging
{
    return this->model_params.xinit;
}

float dynamics_model::get_zinit()      //for debugging
{
    return this->model_params.zinit;       
}

float dynamics_model::get_xdotinit()   //for debugging
{
    return this->model_params.xdotinit;
}

float dynamics_model::get_zdotinit()   //for debugging
{
    return this->model_params.zdotinit;
}

float dynamics_model::get_thetainit()  //for debugging
{
    return this->model_params.thetainit;
}

float dynamics_model::get_num_integrated() //for debugging
{
    return this->model_params.num_integrated;
}

float dynamics_model::get_xfinal()     //for debugging
{
    return this->model_params.xfinal;
}

float dynamics_model::get_xdotfinal()  //for debugging
{
    return this->model_params.xdotfinal;
}

float dynamics_model::get_zdotfinal()  //for debugging
{
    return this->model_params.zdotfinal;
}

float dynamics_model::get_thetafinal() //for debugging
{
    return this->model_params.thetafinal;
}



// #include <cmath>
// #include <math.h>
// #include <algorithm>
// #include <vector>
// #include "dynamics_model.h"
// #include "Drag.h"
// #include <iostream>

// using namespace std;

// //defines
// #define g 9.81
// #define m 33.94
// #define theta_0 13
// #define m_to_ft 3.28084
// #define pi 3.14159265


// dynamics_model::dynamics_model()
// {

// }

// void dynamics_model::init_model()
// {
//     this->model_params.t = 0.0;
//     this->model_params.num_integrated = 0;
//     // this->model_params.xinit = 0.0;
//     // this->model_params.zinit = 0.0;
//     // this->model_params.xdotinit = 0.0;
//     // this->model_params.zdotinit = 0.0;
// }

// float dynamics_model::dynamics(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, vector<int> theta_region, vector<float> theta_vector, float &U_airbrake)
// {
//     while (z_dot > 0) {
//     //for (int i = 0; i < 200; i++){

//         rk4_integrate(t, x, z, x_dot, z_dot, dt, theta_region, theta_vector, U_airbrake);

//         t += dt;
//         this->model_params.t = t;

//         this->model_params.num_integrated += 1;
//         if (get_num_integrated() == 1)
//         {
//             U_airbrake = 0;
//             this->model_params.xinit = x;
//             this->model_params.zinit = z;
//             this->model_params.xdotinit = x_dot;
//             this->model_params.zdotinit = z_dot;
//         }
// 		// cout << "z_dot:\t" << z_dot << endl;
// 	}

//     this->model_params.xfinal = x;
//     this->model_params.apogee_expected = z;
//     this->model_params.xdotfinal = x_dot;
//     this->model_params.zdotfinal = z_dot;
//     return z;
// }

// void dynamics_model::rk4_integrate(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, vector<int> theta_region, vector<float> theta_vector, float U_airbrake)
// {
//     vector<float> k1(4), k2(4), k3(4), k4(4);

//     model(t, x, z, x_dot, z_dot, k1, theta_region, theta_vector, U_airbrake);
//     model(t+0.5*dt, x + 0.5 * dt * k1[0], z+0.5*dt*k1[1], x_dot+0.5*dt*k1[2], z_dot + 0.5*dt*k1[3], k2, theta_region, theta_vector, U_airbrake);
//     model(t+0.5*dt, x + 0.5 * dt * k2[0], z + 0.5 * dt * k2[1], x_dot + 0.5 * dt * k2[2], z_dot + 0.5 * dt * k2[3], k3, theta_region, theta_vector, U_airbrake);
//     model(t+dt, x + dt * k3[0], z + dt * k3[1], x_dot + dt * k3[2], z_dot + dt * k3[3], k4, theta_region, theta_vector, U_airbrake);

//     x += dt * 0.1666 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
//     z += dt * 0.1666 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
//     x_dot += dt * 0.1666 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
//     z_dot += dt * 0.1666 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
// }

// void dynamics_model::model(float t, float x, float z, float x_dot, float z_dot, vector<float> &output, vector<int> theta_region, vector<float> theta_vector, float U_airbrake)
// {
//     float pitch_angle = Calc_pitch_angle(z * m_to_ft, theta_region, theta_vector);

// 	//cout << "Pitch angle (deg): " << pitch_angle*180/pi << endl;

// 	float rho = Calc_rho(z);

// 	//cout << "Rho: " << rho<< endl;

// 	float v_rocket = Calc_v_rocket(z_dot, x_dot);

// 	//cout << "V_rocket: " << v_rocket << endl;

// 	//float U_airbrake = Calc_U_airbrake(t);  // dont think we need this anymore
// 	float cd = Calc_cd(z*m_to_ft, v_rocket*m_to_ft, U_airbrake);

// 	// cout << "cd: " << cd << endl;
// 	//cout << endl;

// 	float area = Calc_area(U_airbrake);

// 	float drag = Calc_drag(rho, pow(v_rocket, 2), cd, area);

// 	float x_float_dot = Calc_x_float_dot(pitch_angle, drag);
// 	float z_float_dot = Calc_z_float_dot(pitch_angle, drag);

// 	// Runge Kutta on x_dot, z_dot, z_float_dot, x_float_dot?

// 	output[0] = x_dot;
// 	output[1] = z_dot;
// 	output[2] = x_float_dot;
// 	output[3] = z_float_dot;
// }

// float dynamics_model::get_t()
// {
//     return this->model_params.t;
// }

// float dynamics_model::get_apogee_expected()
// {
//     return this->model_params.apogee_expected;
// }

// float dynamics_model::Calc_pitch_angle(float z, vector<int> theta_region, vector<float> theta_vector)
// {
//     vector<float> altitude_error(theta_region.size());     //Initialization of altitude error, difference between theta_region and altitude

//     for (size_t i = 0; i < theta_region.size(); i++)        //math for altitude error
//     {
//         altitude_error[i] = abs(theta_region[i] - z);
//     }

//     int altitude_index = distance(altitude_error.begin(), min_element(altitude_error.begin(), altitude_error.end()));   //Finds the index for the current altitude
//     float theta_at_altitude = theta_vector[altitude_index];        //Finds the theta angle at the current altitude, THIS IS THE OUTPUT

//     if (theta_at_altitude > 90)      //Sets any theta values above 90 degrees to 90
//     {
//         theta_at_altitude = 90;
//     }

//     //cout << altitude_index << "\t" << theta_at_altitude << endl;    //for testing with the matlab function

//     // return 0;

//     // Calc_x_float_dot uses sin() which takes in radians so this is converted
// 	float radian_theta = theta_at_altitude * pi * 0.00556;

// 	return radian_theta;
// }

// float dynamics_model::Calc_rho(float z)
// {
//     // lapse rate model goes here
// 	// 
// 	//float z = 3002.3;     //Altitude [m], THIS IS THE INPUT

// 	float T0 = 310;        //Temperature at ground level [K]
// 	float L = 0.0065;      //Lapse rate 

// 	float T = T0 - L * z;        //Temperature at current altitude [K]

// 	float rho0 = 1.13;     //Air density at ground level, THIS NEEDS TO CHANGE BASED ON FLORIDA OR SPACEPORT LAUNCH SITE
// 	// float g = 9.81;        //Gravity [m/s^2]
// 	float R = 287.0531;    //Universal gas constant for air

// 	float rho = rho0 * pow(T / T0, (((g / (L * R))) - 1));     //Air density at altitude [kg/m^3], THIS IS THE OUTPUT

// 	// cout << rho << endl;

// 	//return 0;

// 	return rho;
// }

// float dynamics_model::Calc_v_rocket(float x_dot, float z_dot)
// {
//     return sqrt(pow(x_dot, 2) + pow(z_dot,2));
// }

// // float dynamics_model::Calc_U_airbrake(float t)
// // {

// // }

// float dynamics_model::Calc_cd(float z, float v_rocket, float U_airbrake)
// {
//     // Coefficient of Drag function goes here:

// 	Drag d(z, v_rocket, U_airbrake);
	
// 	return d.get_Cd();
// }

// float dynamics_model::Calc_area(float U_airbrake)
// {
//     // Cross sectional area function goes here
// 	const float d_rocket = 6.14 * 0.0254;    //diameter of rocket in meters, 6.14 in = 0.1560 m
// 	const float A_rocket = (0.25) * (pi)*pow(d_rocket, 2); // my own version of M_PI

// 	// i have this as the input
// 	// float U_airbrake = 0.25;       //Input of the airbrake, this is from 0 -> 1, THIS WILL BE THE INPUT 

// 	float Servo_angle = 105 * U_airbrake;        //Servo angle in degrees

// 	float A_airbrake;

// 	if (U_airbrake == 0)
// 	{
// 		A_airbrake = 0;
// 	}
// 	else
// 	{
// 		//Max cross-sectional area of airbrake during extension in [m^2]
// 		A_airbrake = 0.00064516 * (8.48389446479259e-8 * pow(Servo_angle, 4) - 0.0000348194172718423 * pow(Servo_angle, 3) + 0.00388560760536394 * pow(Servo_angle, 2) - 0.0629348277080075 * Servo_angle + 0.148040926341214);
// 	}

// 	float A_cross = A_rocket + A_airbrake;     //Cross sectional area of the rocket, THIS IS THE OUTPUT

// 	// replaced M_PI
// 	// cout << A_cross << "\t" << pi << endl;

// 	//return 0;

	
// 	return A_cross;
// }

// float dynamics_model::Calc_drag(float rho, float v_rocket_squared, float cd, float area)
// {
//     return rho * v_rocket_squared * cd * .5 * area;
// }

// float dynamics_model::Calc_x_float_dot(float theta, float drag)
// {
//     float x_float_dot = (-1 / m) * sin(theta) * drag;

// 	return x_float_dot;
// }

// float dynamics_model::Calc_z_float_dot(float theta, float drag)
// {
//     float z_float_dot = (-1 / m) * ((drag*cos(theta)) + (m*g));

// 	return z_float_dot;
// }

// float dynamics_model::get_xinit()      //for debugging
// {
//     return this->model_params.xinit;
// }

// float dynamics_model::get_zinit()      //for debugging
// {
//     return this->model_params.zinit;       
// }

// float dynamics_model::get_xdotinit()   //for debugging
// {
//     return this->model_params.xdotinit;
// }

// float dynamics_model::get_zdotinit()   //for debugging
// {
//     return this->model_params.zdotinit;
// }

// float dynamics_model::get_thetainit()  //for debugging
// {
//     return this->model_params.thetainit;
// }

// float dynamics_model::get_num_integrated() //for debugging
// {
//     return this->model_params.num_integrated;
// }

// float dynamics_model::get_xfinal()     //for debugging
// {
//     return this->model_params.xfinal;
// }

// float dynamics_model::get_xdotfinal()  //for debugging
// {
//     return this->model_params.xdotfinal;
// }

// float dynamics_model::get_zdotfinal()  //for debugging
// {
//     return this->model_params.zdotfinal;
// }

// float dynamics_model::get_thetafinal() //for debugging
// {
//     return this->model_params.thetafinal;
// }
