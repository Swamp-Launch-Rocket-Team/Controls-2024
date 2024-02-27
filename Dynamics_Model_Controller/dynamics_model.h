#ifndef DYNAMICS_MODEL_H
#define DYNAMICS_MODEL_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <vector>
#include <unordered_map>

using namespace std;

class dynamics_model
{
    private:
    struct model_params
    {
        //vector<float> k1(4), k2(4), k3(4), k4(4);
        float dt = 0.1;
        float t = 0;
        float xinit = 0;        //for debugging
        float zinit = 0;        //for debugging
        float xdotinit = 0;     //for debugging
        float zdotinit = 0;     //for debugging
        float thetainit = 0;    //for debugging
        int num_integrated = 0; //for debugging
        float xfinal = 0;       //for debugging
        float apogee_expected = 0;
        float xdotfinal = 0;    //for debugging
        float zdotfinal = 0;    //for debugging
        float thetafinal = 0;   //for debugging
    } model_params;
        unordered_map<int, float> theta_map;

    public:
        dynamics_model(unordered_map<int, float> theta_map_in);
        void init_model();
        float dynamics(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, float &U_airbrake);
        void rk4_integrate(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, float U_airbrake);
        void model(float t, float x, float z, float x_dot, float z_dot, vector<float> &output, float U_airbrake);
        float get_t();
        float get_apogee_expected();
        float Calc_pitch_angle(float z);
        float Calc_rho(float z);
        float Calc_v_rocket(float x_dot, float z_dot);
        //float Calc_U_airbrake(float t);
        float Calc_cd(float z, float v_rocket, float U_airbrake);
        float Calc_area(float U_airbrake);
        float Calc_drag(float rho, float v_rocket_squared, float cd, float area);
        float Calc_x_float_dot(float theta, float drag);
        float Calc_z_float_dot(float theta, float drag);
        float get_xinit();      //for debugging
        float get_zinit();      //for debugging
        float get_xdotinit();   //for debugging
        float get_zdotinit();   //for debugging
        float get_thetainit();  //for debugging
        float get_num_integrated(); //for debugging
        float get_xfinal();     //for debugging
        float get_xdotfinal();  //for debugging
        float get_zdotfinal();  //for debugging
        float get_thetafinal(); //for debugging
};

#endif




// #ifndef DYNAMICS_MODEL_H
// #define DYNAMICS_MODEL_H

// #include <iostream>
// #include <cmath>
// #include <math.h>
// #include <algorithm>
// #include <vector>

// using namespace std;

// class dynamics_model
// {
//     private:
//     struct model_params
//     {
//         //vector<float> k1(4), k2(4), k3(4), k4(4);
//         float dt = 0.1;
//         float t = 0;
//         float xinit = 0;        //for debugging
//         float zinit = 0;        //for debugging
//         float xdotinit = 0;     //for debugging
//         float zdotinit = 0;     //for debugging
//         float thetainit = 0;    //for debugging
//         int num_integrated = 0; //for debugging
//         float xfinal = 0;       //for debugging
//         float apogee_expected = 0;
//         float xdotfinal = 0;    //for debugging
//         float zdotfinal = 0;    //for debugging
//         float thetafinal = 0;   //for debugging
//     } model_params;

//     public:
//         dynamics_model();
//         void init_model();
//         float dynamics(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, vector<int> theta_region, vector<float> theta_vector, float &U_airbrake);
//         void rk4_integrate(float t, float &x, float &z, float &x_dot, float &z_dot, float dt, vector<int> theta_region, vector<float> theta_vector, float U_airbrake);
//         void model(float t, float x, float z, float x_dot, float z_dot, vector<float> &output, vector<int> theta_region, vector<float> theta_vector, float U_airbrake);
//         float get_t();
//         float get_apogee_expected();
//         float Calc_pitch_angle(float z, vector<int> theta_region, vector<float> theta_vector);
//         float Calc_rho(float z);
//         float Calc_v_rocket(float x_dot, float z_dot);
//         //float Calc_U_airbrake(float t);
//         float Calc_cd(float z, float v_rocket, float U_airbrake);
//         float Calc_area(float U_airbrake);
//         float Calc_drag(float rho, float v_rocket_squared, float cd, float area);
//         float Calc_x_float_dot(float theta, float drag);
//         float Calc_z_float_dot(float theta, float drag);
//         float get_xinit();      //for debugging
//         float get_zinit();      //for debugging
//         float get_xdotinit();   //for debugging
//         float get_zdotinit();   //for debugging
//         float get_thetainit();  //for debugging
//         float get_num_integrated(); //for debugging
//         float get_xfinal();     //for debugging
//         float get_xdotfinal();  //for debugging
//         float get_zdotfinal();  //for debugging
//         float get_thetafinal(); //for debugging
// };

// #endif

