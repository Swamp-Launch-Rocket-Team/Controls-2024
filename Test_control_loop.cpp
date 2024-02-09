#include <iostream>
#include "Dynamics_Model_Controller/controller.h"
#include "Dynamics_Model_Controller/PI.h"
#include <cmath>
#include <math.h>
#include <algorithm>
#include <vector>

using namespace std;

#define g 9.81
#define m 33.94
#define theta_0 13
#define m_to_ft 3.28084
#define U 0
#define pi 3.14159265

class Rocket {
public:
    Rocket();
protected:
    // Protuberances excluding the airbrake
    float Spro_bracket_base,// wetted area of fin bracket base
        Spro_bracket_height,// wetted area of fin bracket height
        Spro_rail_button;//wetted area of rail buttons

// Individual wetted areas[in , 2]
    float 	S_nosecone,
        S_forw_airf,
        S_main_airf,
        S_drogue_airf,
        S_aft_airf;

    // Rocket Parameters
    float d,// max diameter of rocket[in]
        L,// distance from nosecone tip to bottom of aft, [in]
        Sb,// body wetted area of airframesand nosecone[in , 2]
        Cr,// root chord of fin[in]
        Ct,// tip chord of fin[in]
        t,// thickness of fins[in];
        Xtc,// distance from leading edge to max thickness[in]
        Sf,// wetted area of a single fin[in , 2]
        Sr,// wetted area of entire body, fins, and purtuberances, this is not counting the airbrake, [in , 2]
        db,// diameter at base of rocket[in]
        aa,//constant for kinematic viscosity
        L0;// distance from nosecone - body joint to end of aft[in]
    int Nf = 4;// number of fins

    // rail buttons
    float a_railbutton1,// distance from nosecone tip to rail button 1[in]
        Lp_railbutton1, //// length of rail button 1[in]
        A_railbutton1,// max cross - sectional area of rail button1[in , 2]
        a_railbutton2,
        Lp_railbutton2,
        A_railbutton2;

    // Fin Brackets, 4 sets total, 8 individial brackets
    float a_bracket,// distance from nosecone tip to fin brackets[in]
        Lp_bracket,// length of fin brackets[in]
        A_bracket_base,// max cross - sectional area of fin bracket base[in , 2]
        A_bracket_height;// max cross - sectional area of fin bracket height[in]

    float a_airbrake,// distance from nosecone tip to airbrake[in]
        Lp_airbrake;// length of airbrake, aka flap thickness[in]
};

class Drag : public  Rocket {
public:
    Drag();
    Drag(float, float, float);
    float getCd() const;
    float getCd(float, float, float);
    float getM() const;
    float getM(float, float, float);
private:
    float Cd, M, h, a, visc, Servo_angle;
    // removed pi cuz i have already
    float frictionDrag();
    float proturbenceFdrag(float);
    float bodyFdrag();
    float finFdrag();
    float smallDragCalcs();
};

float Calc_pitch_angle(float z, vector<int> theta_region, vector<float> theta_vector) {
	
	// // function here:

    // float theta_0 = 13;      //Theta value, this will be an input into this function from the IMU

    // vector<float> m_theta{ 0.000525089, 0.000690884, 0.001009584, 0.001398228, 0.001801924 };    //Slopes for linear region, determined in excel

    // vector<int> theta_region(8501);     //Size of theta region

    // for (size_t i = 0; i < theta_region.size(); i++)        //Sets theta region from altitude of 2500 ft o 11k feet
    // {
    //     theta_region[i] = i + 2500;
    // }

    // vector<float> theta_vector(theta_region.size());       //initializes theta vector, same size as theta region

    // //Linear fit region, 2.5k ft to 7k ft
    // float b;

    // if (theta_0 <= 7)        //All of the if statements for theta_0
    // {
    //     b = theta_0 - m_theta[0] * 2500;
    //     for (int i = 0; i < 4501; i++)
    //     {
    //         theta_vector[i] = m_theta[0] * theta_region[i] + b;
    //     }
    // }
    // else if (theta_0 < 7 && theta_0 < 10)
    // {
    //     b = theta_0 - m_theta[1] * 2500;
    //     for (int i = 0; i < 4501; i++)
    //     {
    //         theta_vector[i] = m_theta[1] * theta_region[i] + b;
    //     }
    // }
    // else if (theta_0 >= 10 && theta_0 < 14)
    // {
    //     b = theta_0 - m_theta[2] * 2500;
    //     for (int i = 0; i < 4501; i++)
    //     {
    //         theta_vector[i] = m_theta[2] * theta_region[i] + b;
    //     }
    // }
    // else if (theta_0 >= 14 && theta_0 < 19)
    // {
    //     b = theta_0 - m_theta[3] * 2500;
    //     for (int i = 0; i < 4501; i++)
    //     {
    //         theta_vector[i] = m_theta[3] * theta_region[i] + b;
    //     }
    // }
    // else
    // {
    //     b = theta_0 - m_theta[4] * 2500;
    //     for (int i = 0; i < 4501; i++)
    //     {
    //         theta_vector[i] = m_theta[4] * theta_region[i] + b;
    //     }
    // }
    // //End of Linear fit region, ends at index 4500 at an altitude of 7k feet

    // //Start of the Quadratic fit region, 7k ft to 10k ft
    // vector<float> a_theta{ 8.26652482191255e-7, 1.03558936423213e-6, 1.53275631191493e-6, 2.17922684530253e-6, 2.92066636707301e-6 };

    // int h_theta = 0;        //Parabola parameter for quadratic region
    // float k_theta = theta_vector[4500];        //Initial value of quadratic region

    // if (theta_0 < 7)     //if statements for the different initial thetas
    // {
    //     for (int i = 4501; i < 7501; i++)
    //     {
    //         theta_vector[i] = a_theta[0] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
    //     }
    // }
    // else if (theta_0 < 7 && theta_0 < 10)
    // {
    //     for (int i = 4501; i < 7501; i++)
    //     {
    //         theta_vector[i] = a_theta[1] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
    //     }
    // }
    // else if (theta_0 >= 10 && theta_0 < 14)
    // {
    //     for (int i = 4501; i < 7501; i++)
    //     {
    //         theta_vector[i] = a_theta[2] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
    //     }
    // }
    // else if (theta_0 >= 14 && theta_0 < 19)
    // {
    //     for (int i = 4501; i < 7501; i++)
    //     {
    //         theta_vector[i] = a_theta[3] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
    //     }
    // }
    // else
    // {
    //     for (int i = 4501; i < 7501; i++)
    //     {
    //         theta_vector[i] = a_theta[4] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
    //     }
    // }
    // //End of Quadratic fit region, ends at index 7500 at an altitude of 10k feet

    // //Region after Quadratic region, increase linearly until 90 degrees at a steep slope
    // float inc = 0.1;        //increment for the linear section past the quadratic region
    // vector<float> int_vec(1000);      //interval vector initialization

    // for (size_t i = 0; i < int_vec.size(); i++)     //interval vector definition, 1:1:1000
    // {
    //     int_vec[i] = i + 1.0;
    // }

    // for (size_t i = 7501; i < theta_vector.size(); i++)     //adds the last linear section past quadratic region
    // {
    //     theta_vector[i] = theta_vector[7500] + inc * int_vec[i - 7501];
    // }
    // //End of last linear increase region

    // //Finding the angle theta for a given altitude z

    // //float z = 9432;    //Altitude in feet, THIS WILL BE AN INPUT FROM THE ALTIMETER

    vector<float> altitude_error(theta_region.size());     //Initialization of altitude error, difference between theta_region and altitude

    for (size_t i = 0; i < theta_region.size(); i++)        //math for altitude error
    {
        altitude_error[i] = abs(theta_region[i] - z);
    }

    int altitude_index = distance(altitude_error.begin(), min_element(altitude_error.begin(), altitude_error.end()));   //Finds the index for the current altitude
    float theta_at_altitude = theta_vector[altitude_index];        //Finds the theta angle at the current altitude, THIS IS THE OUTPUT

    if (theta_at_altitude > 90)      //Sets any theta values above 90 degrees to 90
    {
        theta_at_altitude = 90;
    }

    //cout << altitude_index << "\t" << theta_at_altitude << endl;    //for testing with the matlab function

    // return 0;

    // Calc_x_float_dot uses sin() which takes in radians so this is converted
	float radian_theta = theta_at_altitude * pi / 180;

	return radian_theta;
}

float Calc_rho(float z) {
	// lapse rate model goes here
	// 
	//float z = 3002.3;     //Altitude [m], THIS IS THE INPUT

	float T0 = 310;        //Temperature at ground level [K]
	float L = 0.0065;      //Lapse rate 

	float T = T0 - L * z;        //Temperature at current altitude [K]

	float rho0 = 1.13;     //Air density at ground level, THIS NEEDS TO CHANGE BASED ON FLORIDA OR SPACEPORT LAUNCH SITE
	// float g = 9.81;        //Gravity [m/s^2]
	float R = 287.0531;    //Universal gas constant for air

	float rho = rho0 * pow(T / T0, (((g / (L * R))) - 1));     //Air density at altitude [kg/m^3], THIS IS THE OUTPUT

	// cout << rho << endl;

	//return 0;

	return rho;
}

float Calc_v_rocket(float x_dot, float z_dot) {
	return sqrt(pow(x_dot, 2) + pow(z_dot,2));
}

float Calc_U_airbrake(float t) {

	// Not sure if U is supposed to change at all but this does the saturation
	if(t > .1){
		return 0;
	}
	else {
		return U;
	}
}

float Calc_cd(float z, float v_rocket, float U_airbrake) {
	// Coefficient of Drag function goes here:

	Drag d(z, v_rocket, U_airbrake);
	
	return d.getCd();
}

float Calc_area(float U_airbrake) {
	// Cross sectional area function goes here
	const float d_rocket = 6.14 * 0.0254;    //diameter of rocket in meters, 6.14 in = 0.1560 m
	const float A_rocket = (0.25) * (pi)*pow(d_rocket, 2); // my own version of M_PI

	// i have this as the input
	// float U_airbrake = 0.25;       //Input of the airbrake, this is from 0 -> 1, THIS WILL BE THE INPUT 

	float Servo_angle = 105 * U_airbrake;        //Servo angle in degrees

	float A_airbrake;

	if (U_airbrake == 0)
	{
		A_airbrake = 0;
	}
	else
	{
		//Max cross-sectional area of airbrake during extension in [m^2]
		A_airbrake = 0.00064516 * (8.48389446479259e-8 * pow(Servo_angle, 4) - 0.0000348194172718423 * pow(Servo_angle, 3) + 0.00388560760536394 * pow(Servo_angle, 2) - 0.0629348277080075 * Servo_angle + 0.148040926341214);
	}

	float A_cross = A_rocket + A_airbrake;     //Cross sectional area of the rocket, THIS IS THE OUTPUT

	// replaced M_PI
	// cout << A_cross << "\t" << pi << endl;

	//return 0;

	
	return A_cross;
}

float Calc_drag(float rho, float v_rocket_squared, float cd, float area) {
	return rho * v_rocket_squared * cd * .5 * area;
}

float Calc_x_float_dot(float theta, float drag) {
	
	// not sure if sin works here
	float x_float_dot = (-1 / m) * sin(theta) * drag;

	return x_float_dot;
}

float Calc_z_float_dot(float theta, float drag) {

	float z_float_dot = (-1 / m) * ((drag*cos(theta)) + (m*g));

	return z_float_dot;
}

void dynamicsModel(float t, float x, float z, float x_dot, float z_dot, vector<float> &output, vector<int> theta_region, vector<float> theta_vector) {

	// x = x
	// z_dot = z_dot
	// x_dot = x_dot

	float pitch_angle = Calc_pitch_angle(z * m_to_ft, theta_region, theta_vector);

	//cout << "Pitch angle (deg): " << pitch_angle*180/pi << endl;

	float rho = Calc_rho(z);

	//cout << "Rho: " << rho<< endl;

	float v_rocket = Calc_v_rocket(z_dot, x_dot);

	//cout << "V_rocket: " << v_rocket << endl;

	float U_airbrake = Calc_U_airbrake(t);  // if its not 0 then its just a value
	float cd = Calc_cd(z*m_to_ft, v_rocket*m_to_ft, U_airbrake);

	//cout << "cd: " << cd << endl;
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

void rk4_integrate(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, vector<int> theta_region, vector<float> theta_vector) {

    vector<float> k1(4), k2(4), k3(4), k4(4);

    dynamicsModel(t, x, z, x_dot, z_dot, k1, theta_region, theta_vector);
    dynamicsModel(t+0.5*dt, x + 0.5 * dt * k1[0], z+0.5*dt*k1[1], x_dot+0.5*dt*k1[2], z_dot + 0.5*dt*k1[3], k2, theta_region, theta_vector);
    dynamicsModel(t+0.5*dt, x + 0.5 * dt * k2[0], z + 0.5 * dt * k2[1], x_dot + 0.5 * dt * k2[2], z_dot + 0.5 * dt * k2[3], k3, theta_region, theta_vector);
    dynamicsModel(t+dt, x + dt * k3[0], z + dt * k3[1], x_dot + dt * k3[2], z_dot + dt * k3[3], k4, theta_region, theta_vector);

    x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    z += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    x_dot += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    z_dot += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);

}

int main()
{
    // function here:

    // float theta_0 = 13;      //Theta value, this will be an input into this function from the IMU

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
    //End of last linear increase region

    //Finding the angle theta for a given altitude z

    //float z = 9432;    //Altitude in feet, THIS WILL BE AN INPUT FROM THE ALTIMETER

	// Initial Conditions
	float x = 0.0;
	float z = 753;
	float x_dot = 58.38;
	float z_dot = 285;
    
    // time step info
    float t = 0;
    float dt = 0.1;

    // tracking how many times its been integrated
    int num_integrated = 1;

    // until z_dot is negative
	while (z_dot > 0) {
    //for (int i = 0; i < 200; i++){

        rk4_integrate(t, x, z, x_dot, z_dot, dt, theta_region, theta_vector);

        t += dt;

        num_integrated += 1;
		// cout << "z_dot:\t" << z_dot << endl;
	}

    cout << "Time: " << t << endl;
	cout << "Number of times integrated: " << num_integrated << endl;
	cout << "x: " << x << endl;
	cout << "z: " << z << endl;
	cout << "x_dot: " << x_dot << endl;
	cout << "z_dot: " << z_dot << endl;

    //Controller Part
    float Pwm_home_value = 35;      //in main.cpp this will be from pad.get_Pwm_home_value() getter
    float Pwm_max_value = 1250;     //in main.cpp this will be from pad.get_Pwm_max_value() getter 
    float Mach = 0.6;              //this will be calculated and passed into here
    float altitude = 1000;         //this is from the altimeter
    controller test;                //creates controller instance, probably don't name this test in main.cpp lol
    test.init_controller(Pwm_home_value, Pwm_max_value);        //initializes the controller, this should only run once, probably at the end of the Launch Detected status
    test.controller_loop(z, Mach, altitude);        //method that finds the airbrake output in PWM signal
    cout << "Final controller output:" << "\t" << test.get_airbrake_output() << endl;       //Prints airbrake controller output


    return 0;
}







//class def
Rocket::Rocket()
    :Spro_bracket_base(10.9), Spro_bracket_height(8.95), Spro_rail_button(1.95),
    S_nosecone(302.9), S_forw_airf(694.4), S_main_airf(559.9), S_drogue_airf(385.8),
    S_aft_airf(945.2), d(6.14), L(160), Cr(13), Ct(11), t(.18), Xtc(13), Nf(4),
    Sf(131.64), db(6.14), L0(129), aa(0.00002503), a_railbutton1(96), A_railbutton1(0.317), Lp_railbutton1(1.26),
    a_railbutton2(142), A_railbutton2(0.317), Lp_railbutton2(1.26),
    a_bracket(96), Lp_bracket(1.26), A_bracket_base(0.148), A_bracket_height(0.165),
    a_airbrake(112), Lp_airbrake(0.19)
{
    Sb = S_nosecone + S_forw_airf + S_main_airf + S_drogue_airf + S_aft_airf;// body wetted area of airframesand nosecone[in , 2]
    Sr = Sb + Nf * Sf + 8 * Spro_bracket_base + 8 * Spro_bracket_height + 2 * Spro_rail_button;// wetted area of entire body, fins, and purtuberances, this is not counting the airbrake, [in , 2]
}

Drag::Drag()
	:Rocket(), Cd(0), M(0), h(4595), a(0), visc(0), Servo_angle(0)
{}

Drag::Drag(float z, float V_rocket, float U_airbrake)
	: Rocket()
{
	h = 4595 + z; // altitude above ground feet
	a = -0.004 * h + 1116.45; // speed of sound approx ft/s
	M = V_rocket / a; // local mach number
	visc = 0.000157 * exp(aa * h);// kinematic viscosity ft2/s
	Servo_angle = (105) * U_airbrake;
	//// Total Frictionand Interference Drag Coefficient
	Cd = bodyFdrag() + 1.04 * finFdrag() + smallDragCalcs() + proturbenceFdrag(U_airbrake);
	// Total friction drag coefficient
}

float Drag::getCd() const {
	return Cd;
}

float Drag::getM() const {
	return M;
}

float Drag::bodyFdrag() {
	float Rn_star_body = (a * M * L / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Reynolds number,,
	float	Cf_star_body = 0.037036 * pow((Rn_star_body), (-0.155079));// Incompressible skin friction coefficient
	float	Cf_body = Cf_star_body * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient,,,
	float k_body = 0.0012;// surface roughness coefficient
	float Cf_star_term_body = 1 / (pow(1.89 + 1.62 * (log10(L / k_body)), 2.5));// Incompressible skin friction coefficient w / roughness
	float Cf_term_body = Cf_star_term_body / (1 + 0.2044 * pow(M, 2));// Compressible skin friction coefficient w / roughness
	float Cf_final_body;
	if (Cf_body >= Cf_term_body) {
		Cf_final_body = Cf_body;
	}
	else {
		Cf_final_body = Cf_term_body;
	}

	float Cd_f_body = Cf_final_body * (1 + (60 / (pow((L / d), 3))) + 0.0025 * (L / d)) * (4 * Sb / (pi * pow(d, 2)));
	// Body drag coefficient due to friction
	return Cd_f_body;
}

float Drag::finFdrag() {
	// same speed of sound a, and kinematic viscosity visc
	float Rn_star_fin = (a * M * Cr / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Re number for fin,,,
	float Cf_star_fin = 0.037036 * pow(Rn_star_fin, -0.155079);// Incompressible skin friction coefficient for fin
	float Cf_fin = Cf_star_fin * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient of fin,,,
	float k_fin = 0.0012;
	float Cf_star_term_fin = 1 / (pow((1.89 + 1.62 * log10(Cr / k_fin)), 2.5));// Incompressible skin friction coefficient w / roughness
	float Cf_term_fin = Cf_star_term_fin / (1 + 0.2044 * pow(M, 2));// Compressible skin friction coefficient w / roughness
	float Cf_final_fin;
	if (Cf_fin >= Cf_term_fin) {
		Cf_final_fin = Cf_fin;
	}
	else {
		Cf_final_fin = Cf_term_fin;
	}
	float Rn_fin = (a * M * Cr) / (12 * visc); //Incompressible Re
	float lamda_fin = Ct / Cr;// Ratio of fin tip chord to root chord
	float Cf_lamda_fin;

	if (lamda_fin == 0) {
		Cf_lamda_fin = Cf_final_fin * (1 + (0.5646 / (log10(Rn_fin))));
	}
	else {
		Cf_lamda_fin = Cf_final_fin * ((pow((log10(Rn_fin)), 2.6)) / (pow(lamda_fin, 2) - 1)) * ((pow(lamda_fin, 2)) / (pow((log10(Rn_fin * lamda_fin)), 2.6)) - 1 / (pow((log10(Rn_fin)), 2.6)) + 0.5646 * (((pow(lamda_fin, 2)) / (pow((log10(Rn_fin * lamda_fin)), 3.6))) - 1 / (pow((log10(Rn_fin)), 3.6))));
	}

	float Cd_f_fins = Cf_lamda_fin * (1 + 60 * pow((t / Cr), 4) + 0.8 * (1 + 5 * pow((Xtc / Cr), 2)) * (t / Cr)) * (4.0 * (float)Nf * Sf) / (pi * pow(d, 2));
	//	 Fin drag cofficient due to friction for all 4 fins
	return Cd_f_fins;
}

float Drag::proturbenceFdrag(float U_airbrake) {
	float Rn_star_railbutton1 = ((a * M * Lp_railbutton1) / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Reynolds number of rail button1,,
	float Cf_star_railbutton1 = 0.037036 * pow((Rn_star_railbutton1), (-0.155079));// Incompressible skin friction coefficient for rail button1
	float Cf_railbutton1 = Cf_star_railbutton1 * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient of rail button1,,,
	float k_railbutton1 = 0.0001;
	float Cf_star_term_railbutton1 = 1 / (pow((1.89 + 1.62 * log10(Lp_railbutton1 / k_railbutton1)), 2.5));// Incompressible skin friction coefficient w / roughness
	float Cf_term_railbutton1 = Cf_star_term_railbutton1 / (1 + 0.2044 * pow(M, 2));
	float Cf_final_railbutton1;
	if (Cf_railbutton1 >= Cf_term_railbutton1) Cf_final_railbutton1 = Cf_railbutton1;
	else Cf_final_railbutton1 = Cf_term_railbutton1;

	float	Cf_pro_railbutton1 = 0.8151 * Cf_final_railbutton1 * (pow((a_railbutton1 / Lp_railbutton1), -0.1243));
	float Cd_pro_railbutton1 = Cf_pro_railbutton1 * (1 + 1.798 * (pow((sqrt(A_railbutton1) / Lp_railbutton1), 1.5))) * (4 * Spro_rail_button) / (pi * pow(d, 2));
	// Drag coefficient of rail button 1 protuberance

	float Rn_star_railbutton2 = ((a * M * Lp_railbutton2) / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Reynolds number of rail button2,,
	float	Cf_star_railbutton2 = 0.037036 * pow((Rn_star_railbutton2), (-0.155079));// Incompressible skin friction coefficient for rail button2
	float	Cf_railbutton2 = Cf_star_railbutton2 * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient of rail button1,,,
	float	k_railbutton2 = 0.0001;
	float Cf_star_term_railbutton2 = 1 / (pow((1.89 + 1.62 * log10(Lp_railbutton2 / k_railbutton2)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_railbutton2 = Cf_star_term_railbutton2 / (1 + 0.2044 * pow(M, 2));
	float Cf_final_railbutton2;
	if (Cf_railbutton2 >= Cf_term_railbutton2) Cf_final_railbutton2 = Cf_railbutton2;
	else Cf_final_railbutton2 = Cf_term_railbutton2;

	float Cf_pro_railbutton2 = 0.8151 * Cf_final_railbutton2 * (pow((a_railbutton2 / Lp_railbutton2), -0.1243));
	float Cd_pro_railbutton2 = Cf_pro_railbutton2 * (1 + 1.798 * (pow((sqrt(A_railbutton2) / Lp_railbutton2), 1.5))) * (4 * Spro_rail_button) / (pi * pow(d, 2));
	// Drag coefficient of rail button 2 protuberance

		// same speed of sound a and viscosity visc as before
	float	Rn_star_bracketbase = ((a * M * Lp_bracket) / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Reynolds number of bracket base,,
	float	Cf_star_bracketbase = 0.037036 * pow((Rn_star_bracketbase), (-0.155079));// Incompressible skin friction coefficient for bracket base
	float	Cf_bracketbase = Cf_star_bracketbase * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient of bracket base,,,
	float	k_bracketbase = 0.0002;
	float Cf_star_term_bracketbase = 1 / (pow((1.89 + 1.62 * log10(Lp_bracket / k_bracketbase)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_bracketbase = Cf_star_term_bracketbase / (1 + 0.2044 * pow(M, 2));

	float Cf_final_bracketbase;
	if (Cf_bracketbase >= Cf_term_bracketbase) Cf_final_bracketbase = Cf_bracketbase;
	else Cf_final_bracketbase = Cf_term_bracketbase;

	float	Cf_pro_bracketbase = 0.8151 * Cf_final_bracketbase * (pow((a_bracket / Lp_bracket), -0.1243));
	float Cd_pro_bracketbase = 8 * Cf_pro_bracketbase * (1 + 1.798 * (pow((sqrt(A_bracket_base) / Lp_bracket), 1.5))) * (4 * Spro_bracket_base) / (pi * pow(d, 2));
	// Drag coefficient of 8 fin bracket bases

		// same speed of sound a and viscosity visc as before
	float	Rn_star_bracketheight = ((a * M * Lp_bracket) / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Reynolds number of bracket height,,
	float	Cf_star_bracketheight = 0.037036 * pow((Rn_star_bracketheight), (-0.155079));// Incompressible skin friction coefficient for bracket height
	float	Cf_bracketheight = Cf_star_bracketheight * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient of bracket height,,,
	float	k_bracketheight = 0.0002;
	float Cf_star_term_bracketheight = 1 / (pow((1.89 + 1.62 * log10(Lp_bracket / k_bracketheight)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_bracketheight = Cf_star_term_bracketheight / (1 + 0.2044 * pow(M, 2));

	float Cf_final_bracketheight;
	if (Cf_bracketheight >= Cf_term_bracketheight) Cf_final_bracketheight = Cf_bracketheight;
	else Cf_final_bracketheight = Cf_term_bracketheight;

	float	Cf_pro_bracketheight = 0.8151 * Cf_final_bracketheight * (pow((a_bracket / Lp_bracket), -0.1243));
	float Cd_pro_bracketheight = 8 * Cf_pro_bracketheight * (1 + 1.798 * (pow((sqrt(A_bracket_height) / Lp_bracket), 1.5))) * (4 * Spro_bracket_height) / (pi * pow(d, 2));
	// Drag coefficient of 8 fin bracket height

	float A_airbrake, Spro_airbrake;

	if (U_airbrake == 0) {
		A_airbrake = 0;
		Spro_airbrake = 0;
	}
	else {
		A_airbrake = 8.48389446479259e-08 * pow(Servo_angle, 4) - 0.0000348194172718423 * pow(Servo_angle, 3) + 0.00388560760536394 * pow(Servo_angle, 2) - 0.0629348277080075 * Servo_angle + 0.148040926341214;// max cross - sectional area of airbrake during extension[in , 2]
		Spro_airbrake = 1.52223912727639e-07 * pow(Servo_angle, 4) - 0.0000493635877996522 * pow(Servo_angle, 3) + 0.00469232129431139 * pow(Servo_angle, 2) - 0.0458806779765204 * Servo_angle + 0.0331759530791818;// wetted surface area of airbrake at extension[in , 2]
	}
	// same speed of sound aand viscosity visc as before
	float Rn_star_airbrake = ((a * M * Lp_airbrake) / (12 * visc)) * (1 + 0.0283 * M - 0.043 * pow(M, 2) + 0.2107 * pow(M, 3) - 0.03829 * pow(M, 4) + 0.002709 * pow(M, 5));
	// Compressible Reynolds number of airbrake,,
	float	Cf_star_airbrake = 0.037036 * pow((Rn_star_airbrake), (-0.155079));// Incompressible skin friction coefficient for airbrake
	float	Cf_airbrake = Cf_star_airbrake * (1 + 0.00798 * M - 0.1813 * pow(M, 2) + 0.0632 * pow(M, 3) - 0.00933 * pow(M, 4) + 0.000549 * pow(M, 5));
	// Compressible skin friction coefficient of airbrake,,,
	float	k_airbrake = 0.0012;
	float Cf_star_term_airbrake = 1 / (pow((1.89 + 1.62 * log10(Lp_airbrake / k_airbrake)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_airbrake = Cf_star_term_airbrake / (1 + 0.2044 * pow(M, 2));

	float Cf_final_airbrake;
	if (Cf_airbrake >= Cf_term_airbrake) {
		Cf_final_airbrake = Cf_airbrake;
	}
	else Cf_final_airbrake = Cf_term_airbrake;

	float	Cf_pro_airbrake = 0.8151 * Cf_final_airbrake * (pow((a_airbrake / Lp_airbrake), -0.1243));
	float Cd_pro_airbrake = Cf_pro_airbrake * (1 + 1.798 * (pow((sqrt(A_airbrake) / Lp_airbrake), 1.5))) * (4 * Spro_airbrake) / (pi * pow(d, 2));
	// Drag coefficient of airbrake at extension,,


	float Cd_pro_total = Cd_pro_railbutton1 + Cd_pro_railbutton2 + Cd_pro_bracketbase + Cd_pro_bracketheight + Cd_pro_airbrake;
	// Drag coefficient of all of the protuberances
		// same speed of sound aand viscosity visc as before
	float Sr_total = Sr + Spro_airbrake;// add the wetted area of airbrake to rocket wetted area
	float Ke = 0.00038;// coefficient of excrescencies for M < 0.78
	float Cde = Ke * 4 * Sr_total / (pi * pow(d, 2));// change in drag coeff due to excrescencies
	float Kf = 1.04;// mutual interference factor of fins& rail button w / body

	return Kf * Cd_pro_total + Cde;
}

float Drag::smallDragCalcs() {
	// Drag Due to Excrescencies


		//// Base Drag Coefficient
		// Kb = 0.0274 * atan((L0 / d) + 0.0116);// const of proportionality
		// Cd_base = Kb / sqrt(Cd_f);// Base coefficient of Drag
	float Cd_base = 0.12 + 0.13 * pow(M, 2);// From OpenRocket resource, use this one!!!

	//// Fin Pressure Drag, use this
	float Cd_press_fins = pow((1 - pow(M, 2)), (-0.3)) - 1;// This was taken from the OpenRocket resource, adjusted for rectangular cross - section
	return Cd_base + Cd_press_fins;
}