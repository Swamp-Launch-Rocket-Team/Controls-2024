#include <iostream>
#include <cmath>
#include "drag.h"

using namespace std;

#define pi 3.14159265

Drag::Drag()
{

}

Drag::Drag(float z, float V_rocket, float U_airbrake)
{
    this->drag_parameters.h = 13.0 + z; // altitude above ground feet, first value is DEPENDENT on the launch site
	this->drag_parameters.a = -0.004 * drag_parameters.h + 1116.45; // speed of sound approx ft/s
	this->drag_parameters.M = V_rocket / drag_parameters.a; // local mach number
	float visc = 0.000157 * exp(drag_parameters.aa * drag_parameters.h);// kinematic viscosity ft2/s
	this->drag_parameters.Servo_angle = (105.0) * U_airbrake;




    //Body Friction Drag Calc
    float Rn_star_body = (drag_parameters.a * drag_parameters.M * drag_parameters.L / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Reynolds number,,
	float	Cf_star_body = 0.037036 * pow((Rn_star_body), (-0.155079));// Incompressible skin friction coefficient
	float	Cf_body = Cf_star_body * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient,,,
	float k_body = 0.0012;// surface roughness coefficient
	float Cf_star_term_body = 1.0 / (pow(1.89 + 1.62 * (log10(drag_parameters.L / k_body)), 2.5));// Incompressible skin friction coefficient w / roughness
	float Cf_term_body = Cf_star_term_body / (1 + 0.2044 * pow(drag_parameters.M, 2));// Compressible skin friction coefficient w / roughness
	float Cf_final_body;
	if (Cf_body >= Cf_term_body) {
		Cf_final_body = Cf_body;
	}
	else {
		Cf_final_body = Cf_term_body;
	}

	float Cd_f_body = Cf_final_body * (1 + (60.0 / (pow((drag_parameters.L / drag_parameters.d), 3))) + 0.0025 * (drag_parameters.L / drag_parameters.d)) * (4.0 * drag_parameters.Sb / (pi * pow(drag_parameters.d, 2)));
	// Body drag coefficient due to friction
	this->drag_parameters.bodyFdrag = Cd_f_body;





    //Fin Friction Drag Calc
    float Rn_star_fin = (drag_parameters.a * drag_parameters.M * drag_parameters.Cr / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Re number for fin,,,
	float Cf_star_fin = 0.037036 * pow(Rn_star_fin, -0.155079);// Incompressible skin friction coefficient for fin
	float Cf_fin = Cf_star_fin * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient of fin,,,
	float k_fin = 0.0012;
	float Cf_star_term_fin = 1.0 / (pow((1.89 + 1.62 * log10(drag_parameters.Cr / k_fin)), 2.5));// Incompressible skin friction coefficient w / roughness
	float Cf_term_fin = Cf_star_term_fin / (1 + 0.2044 * pow(drag_parameters.M, 2));// Compressible skin friction coefficient w / roughness
	float Cf_final_fin;
	if (Cf_fin >= Cf_term_fin) {
		Cf_final_fin = Cf_fin;
	}
	else {
		Cf_final_fin = Cf_term_fin;
	}
	float Rn_fin = (drag_parameters.a * drag_parameters.M * drag_parameters.Cr) / (12.0 * visc); //Incompressible Re
	float lamda_fin = drag_parameters.Ct / drag_parameters.Cr;// Ratio of fin tip chord to root chord
	float Cf_lamda_fin;

	if (lamda_fin == 0) {
		Cf_lamda_fin = Cf_final_fin * (1 + (0.5646 / (log10(Rn_fin))));
	}
	else {
		Cf_lamda_fin = Cf_final_fin * ((pow((log10(Rn_fin)), 2.6)) / (pow(lamda_fin, 2) - 1)) * ((pow(lamda_fin, 2)) / (pow((log10(Rn_fin * lamda_fin)), 2.6)) - 1.0 / (pow((log10(Rn_fin)), 2.6)) + 0.5646 * (((pow(lamda_fin, 2)) / (pow((log10(Rn_fin * lamda_fin)), 3.6))) - 1.0 / (pow((log10(Rn_fin)), 3.6))));
	}

	float Cd_f_fins = Cf_lamda_fin * (1 + 60.0 * pow((drag_parameters.t / drag_parameters.Cr), 4) + 0.8 * (1 + 5.0 * pow((drag_parameters.Xtc / drag_parameters.Cr), 2)) * (drag_parameters.t / drag_parameters.Cr)) * (4.0 * (float)drag_parameters.Nf * drag_parameters.Sf) / (pi * pow(drag_parameters.d, 2));
	//	 Fin drag cofficient due to friction for all 4 fins
	this->drag_parameters.finFdrag  = Cd_f_fins;





    //Protuberance Drag Calc
    float Rn_star_railbutton1 = ((drag_parameters.a * drag_parameters.M * drag_parameters.Lp_railbutton1) / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Reynolds number of rail button1,,
	float Cf_star_railbutton1 = 0.037036 * pow((Rn_star_railbutton1), (-0.155079));// Incompressible skin friction coefficient for rail button1
	float Cf_railbutton1 = Cf_star_railbutton1 * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient of rail button1,,,
	float k_railbutton1 = 0.0001;
	float Cf_star_term_railbutton1 = 1.0 / (pow((1.89 + 1.62 * log10(drag_parameters.Lp_railbutton1 / k_railbutton1)), 2.5));// Incompressible skin friction coefficient w / roughness
	float Cf_term_railbutton1 = Cf_star_term_railbutton1 / (1 + 0.2044 * pow(drag_parameters.M, 2));
	float Cf_final_railbutton1;
	if (Cf_railbutton1 >= Cf_term_railbutton1) Cf_final_railbutton1 = Cf_railbutton1;
	else Cf_final_railbutton1 = Cf_term_railbutton1;

	float	Cf_pro_railbutton1 = 0.8151 * Cf_final_railbutton1 * (pow((drag_parameters.a_railbutton1 / drag_parameters.Lp_railbutton1), -0.1243));
	float Cd_pro_railbutton1 = Cf_pro_railbutton1 * (1 + 1.798 * (pow((sqrt(drag_parameters.A_railbutton1) / drag_parameters.Lp_railbutton1), 1.5))) * (4.0 * drag_parameters.Spro_rail_button) / (pi * pow(drag_parameters.d, 2));
	// Drag coefficient of rail button 1 protuberance

	float Rn_star_railbutton2 = ((drag_parameters.a * drag_parameters.M * drag_parameters.Lp_railbutton2) / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Reynolds number of rail button2,,
	float	Cf_star_railbutton2 = 0.037036 * pow((Rn_star_railbutton2), (-0.155079));// Incompressible skin friction coefficient for rail button2
	float	Cf_railbutton2 = Cf_star_railbutton2 * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient of rail button1,,,
	float	k_railbutton2 = 0.0001;
	float Cf_star_term_railbutton2 = 1.0 / (pow((1.89 + 1.62 * log10(drag_parameters.Lp_railbutton2 / k_railbutton2)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_railbutton2 = Cf_star_term_railbutton2 / (1 + 0.2044 * pow(drag_parameters.M, 2));
	float Cf_final_railbutton2;
	if (Cf_railbutton2 >= Cf_term_railbutton2) Cf_final_railbutton2 = Cf_railbutton2;
	else Cf_final_railbutton2 = Cf_term_railbutton2;

	float Cf_pro_railbutton2 = 0.8151 * Cf_final_railbutton2 * (pow((drag_parameters.a_railbutton2 / drag_parameters.Lp_railbutton2), -0.1243));
	float Cd_pro_railbutton2 = Cf_pro_railbutton2 * (1 + 1.798 * (pow((sqrt(drag_parameters.A_railbutton2) / drag_parameters.Lp_railbutton2), 1.5))) * (4.0 * drag_parameters.Spro_rail_button) / (pi * pow(drag_parameters.d, 2));
	// Drag coefficient of rail button 2 protuberance

		// same speed of sound a and viscosity visc as before
	float	Rn_star_bracketbase = ((drag_parameters.a * drag_parameters.M * drag_parameters.Lp_bracket) / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Reynolds number of bracket base,,
	float	Cf_star_bracketbase = 0.037036 * pow((Rn_star_bracketbase), (-0.155079));// Incompressible skin friction coefficient for bracket base
	float	Cf_bracketbase = Cf_star_bracketbase * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient of bracket base,,,
	float	k_bracketbase = 0.0002;
	float Cf_star_term_bracketbase = 1.0 / (pow((1.89 + 1.62 * log10(drag_parameters.Lp_bracket / k_bracketbase)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_bracketbase = Cf_star_term_bracketbase / (1 + 0.2044 * pow(drag_parameters.M, 2));

	float Cf_final_bracketbase;
	if (Cf_bracketbase >= Cf_term_bracketbase) Cf_final_bracketbase = Cf_bracketbase;
	else Cf_final_bracketbase = Cf_term_bracketbase;

	float	Cf_pro_bracketbase = 0.8151 * Cf_final_bracketbase * (pow((drag_parameters.a_bracket / drag_parameters.Lp_bracket), -0.1243));
	float Cd_pro_bracketbase = 8.0 * Cf_pro_bracketbase * (1 + 1.798 * (pow((sqrt(drag_parameters.A_bracket_base) / drag_parameters.Lp_bracket), 1.5))) * (4.0 * drag_parameters.Spro_bracket_base) / (pi * pow(drag_parameters.d, 2));
	// Drag coefficient of 8 fin bracket bases

		// same speed of sound a and viscosity visc as before
	float	Rn_star_bracketheight = ((drag_parameters.a * drag_parameters.M * drag_parameters.Lp_bracket) / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Reynolds number of bracket height,,
	float	Cf_star_bracketheight = 0.037036 * pow((Rn_star_bracketheight), (-0.155079));// Incompressible skin friction coefficient for bracket height
	float	Cf_bracketheight = Cf_star_bracketheight * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient of bracket height,,,
	float	k_bracketheight = 0.0002;
	float Cf_star_term_bracketheight = 1.0 / (pow((1.89 + 1.62 * log10(drag_parameters.Lp_bracket / k_bracketheight)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_bracketheight = Cf_star_term_bracketheight / (1 + 0.2044 * pow(drag_parameters.M, 2));

	float Cf_final_bracketheight;
	if (Cf_bracketheight >= Cf_term_bracketheight) Cf_final_bracketheight = Cf_bracketheight;
	else Cf_final_bracketheight = Cf_term_bracketheight;

	float	Cf_pro_bracketheight = 0.8151 * Cf_final_bracketheight * (pow((drag_parameters.a_bracket / drag_parameters.Lp_bracket), -0.1243));
	float Cd_pro_bracketheight = 8.0 * Cf_pro_bracketheight * (1 + 1.798 * (pow((sqrt(drag_parameters.A_bracket_height) / drag_parameters.Lp_bracket), 1.5))) * (4.0 * drag_parameters.Spro_bracket_height) / (pi * pow(drag_parameters.d, 2));
	// Drag coefficient of 8 fin bracket height

	float A_airbrake, Spro_airbrake;

	if (U_airbrake == 0) {
		A_airbrake = 0.0;
		Spro_airbrake = 0.0;
	}
	else {
		A_airbrake = abs(8.48389446479259e-08 * pow(drag_parameters.Servo_angle, 4) - 0.0000348194172718423 * pow(drag_parameters.Servo_angle, 3) + 0.00388560760536394 * pow(drag_parameters.Servo_angle, 2) - 0.0629348277080075 * drag_parameters.Servo_angle + 0.148040926341214);// max cross - sectional area of airbrake during extension[in , 2]
		Spro_airbrake = abs(1.52223912727639e-07 * pow(drag_parameters.Servo_angle, 4) - 0.0000493635877996522 * pow(drag_parameters.Servo_angle, 3) + 0.00469232129431139 * pow(drag_parameters.Servo_angle, 2) - 0.0458806779765204 * drag_parameters.Servo_angle + 0.0331759530791818);// wetted surface area of airbrake at extension[in , 2]
	}
    //cout << "Spro_airbrake:\t" << Spro_airbrake << endl;
	// same speed of sound aand viscosity visc as before
	float Rn_star_airbrake = ((drag_parameters.a * drag_parameters.M * drag_parameters.Lp_airbrake) / (12.0 * visc)) * (1 + 0.0283 * drag_parameters.M - 0.043 * pow(drag_parameters.M, 2) + 0.2107 * pow(drag_parameters.M, 3) - 0.03829 * pow(drag_parameters.M, 4) + 0.002709 * pow(drag_parameters.M, 5));
	// Compressible Reynolds number of airbrake,,
	float	Cf_star_airbrake = 0.037036 * pow((Rn_star_airbrake), (-0.155079));// Incompressible skin friction coefficient for airbrake
	float	Cf_airbrake = Cf_star_airbrake * (1 + 0.00798 * drag_parameters.M - 0.1813 * pow(drag_parameters.M, 2) + 0.0632 * pow(drag_parameters.M, 3) - 0.00933 * pow(drag_parameters.M, 4) + 0.000549 * pow(drag_parameters.M, 5));
	// Compressible skin friction coefficient of airbrake,,,
	float	k_airbrake = 0.0012;
	float Cf_star_term_airbrake = 1.0 / (pow((1.89 + 1.62 * log10(drag_parameters.Lp_airbrake / k_airbrake)), 2.5));// Incompressible skin friction coefficient w / roughness
	float	Cf_term_airbrake = Cf_star_term_airbrake / (1 + 0.2044 * pow(drag_parameters.M, 2));

	float Cf_final_airbrake;
	if (Cf_airbrake >= Cf_term_airbrake) {
		Cf_final_airbrake = Cf_airbrake;
	}
	else Cf_final_airbrake = Cf_term_airbrake;

	float	Cf_pro_airbrake = 0.8151 * Cf_final_airbrake * (pow((drag_parameters.a_airbrake / drag_parameters.Lp_airbrake), -0.1243));
	float Cd_pro_airbrake = Cf_pro_airbrake * (1 + 1.798 * (pow((sqrt(A_airbrake) / drag_parameters.Lp_airbrake), 1.5))) * (4.0 * Spro_airbrake) / (pi * pow(drag_parameters.d, 2));
	// Drag coefficient of airbrake at extension,,


	float Cd_pro_total = Cd_pro_railbutton1 + Cd_pro_railbutton2 + Cd_pro_bracketbase + Cd_pro_bracketheight + Cd_pro_airbrake;
    //cout << "Cd_pro_total\t" << Cd_pro_total << endl;
	// Drag coefficient of all of the protuberances
		// same speed of sound aand viscosity visc as before
	float Sr_total = drag_parameters.Sr + Spro_airbrake;// add the wetted area of airbrake to rocket wetted area
    //cout << "Sr:\t" << drag_parameters.Sr << endl;
    //cout << "Sr_total:\t" << Sr_total << endl;
	float Ke = 0.00038;// coefficient of excrescencies for M < 0.78
	float Cde = Ke * 4.0 * Sr_total / (pi * pow(drag_parameters.d, 2));// change in drag coeff due to excrescencies
    //cout << "Cde\t" << Cde << endl;
	float Kf = 1.04;// mutual interference factor of fins& rail button w / body

	this->drag_parameters.proturbenceFdrag = Kf * Cd_pro_total + Cde;





    //Small Drag Calcs
    float Cd_base = 0.12 + 0.13 * pow(drag_parameters.M, 2);// From OpenRocket resource, use this one!!!

	//// Fin Pressure Drag, use this
	float Cd_press_fins = pow((1 - pow(drag_parameters.M, 2)), (-0.2)) - 1;// This was taken from the OpenRocket resource, adjusted for rectangular cross - section
	this->drag_parameters.smallDragCalcs = Cd_base + Cd_press_fins;





    //Total Cd Calc
    this->drag_parameters.Cd = drag_parameters.bodyFdrag + 1.04*drag_parameters.finFdrag + drag_parameters.smallDragCalcs + drag_parameters.proturbenceFdrag;
}

float Drag::get_bodyFdrag()
{
    return this->drag_parameters.bodyFdrag;
}

float Drag::get_finFdrag()
{
    return this->drag_parameters.finFdrag;
}

float Drag::get_smallDragCalcs()
{
    return this->drag_parameters.smallDragCalcs;
}

float Drag::get_proturbenceFdrag()
{
    return this->drag_parameters.proturbenceFdrag;
}

float Drag::get_Cd()
{
    return this->drag_parameters.Cd;
}

float Drag::get_M()
{
    return this->drag_parameters.M;
}

float Drag::get_Servo_angle()
{
    return this->drag_parameters.Servo_angle;
}