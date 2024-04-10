#ifndef DRAG_H
#define DRAG_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <vector>

using namespace std;

class Drag
{
    private:
    struct drag_parameters
    {
        const float Spro_bracket_base = 10.9,// wetted area of fin bracket base, 
            Spro_bracket_height = 8.95,// wetted area of fin bracket height, 
            Spro_rail_button = 1.95;//wetted area of rail buttons, 

        // Individual wetted areas[in , 2]
        const float 	S_nosecone = 302.9, //DEPENDENT
            S_forw_airf = 694.4,            //DEPENDENT
            S_main_airf = 559.9,            //DEPENDENT
            S_drogue_airf = 385.8,          //DEPENDENT
            S_aft_airf = 945.2;             //DEPENDENT

        // Rocket Parameters
        const float d = 6.14,// max diameter of rocket[in]
            L = 164.5,// distance from nosecone tip to bottom of aft, [in], DEPENDENT
            Sb = 2888.2,// body wetted area of airframesand nosecone[in , 2], DEPENDENT
            Cr = 13.0,// root chord of fin[in], DEPENDENT
            Ct = 11.0,// tip chord of fin[in], DEPENDENT
            t = 0.18,// thickness of fins[in];, DEPENDENT
            Xtc = 13.0,// distance from leading edge to max thickness[in], DEPENDENT
            Sf = 131.64,// wetted area of a single fin[in , 2], DEPENDENT
            Sr = 3577.5,// wetted area of entire body, fins, and purtuberances, this is not counting the airbrake, [in , 2], DEPENDENT
            db = 6.14,// diameter at base of rocket[in], 
            aa = 0.00002503,//constant for kinematic viscosity
            L0 = 137.4;// distance from nosecone - body joint to end of aft[in], DEPENDENT
        const int Nf = 4.0;// number of fins

        // rail buttons
        const float a_railbutton1 = 103.0,// distance from nosecone tip to rail button 1[in], DEPENDENT
            Lp_railbutton1 = 1.26, //// length of rail button 1[in]
            A_railbutton1 = 0.317,// max cross - sectional area of rail button1[in , 2], 
            a_railbutton2 = 145.0,        //DEPENDENT
            Lp_railbutton2 = 1.26,
            A_railbutton2 = 0.317;

        // Fin Brackets, 4 sets total, 8 individial brackets
        const float a_bracket = 144.0,// distance from nosecone tip to fin brackets[in], DEPENDENT
            Lp_bracket = 12.6,// length of fin brackets[in]
            A_bracket_base = 0.148,// max cross - sectional area of fin bracket base[in , 2]
            A_bracket_height = 0.165;// max cross - sectional area of fin bracket height[in]

        const float a_airbrake = 111.0,// distance from nosecone tip to airbrake[in], DEPENDENT
            Lp_airbrake = 0.19;// length of airbrake, aka flap thickness[in]

        float h;
        float a;
        float bodyFdrag;
        float finFdrag;
        float smallDragCalcs;
        float proturbenceFdrag;
        float M;
        float Servo_angle;
        float Cd;
    } drag_parameters;

    public:
        Drag();
        Drag(float z, float V_rocket, float U_airbrake);        //z is in units of feet, V_rocket in ft/s, U_airbrake [0 -> 1]!!!!!!!
        float get_bodyFdrag();
        float get_finFdrag();
        float get_smallDragCalcs();
        float get_proturbenceFdrag();
        float get_Cd();
        float get_M();
        float get_Servo_angle();
};

#endif