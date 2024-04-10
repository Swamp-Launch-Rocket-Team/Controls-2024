#include <wiringPi.h>
#include <iostream>
#include <chrono>

using namespace std;

const int Pwm_pin = 23;

int main()
{
	wiringPiSetup();        		//Access the wiringPi library
	pinMode(Pwm_pin, PWM_OUTPUT);       //Set the PWM pin as a PWM OUTPUT
	pwmSetMode(PWM_MODE_MS);

	int PWM_prescaler = 48;     	//Base freq is 19.2 MHz, THIS IS THE VALUE U CHANGE 
	int pwm_range = 1000 * 48 / PWM_prescaler;
	int min_Pwm = pwm_range/5;
    
	pwmSetClock(PWM_prescaler);
	pwmSetRange(pwm_range);

	cout << "Airbrake pic beginning" << endl;

	const float Pwm_home_value = 345;
	const float Pwm_max_value = 625;

	pwmWrite(Pwm_pin, Pwm_home_value);
	delay(2000);

	pwmWrite(Pwm_pin, Pwm_max_value);
	auto t_start = chrono::high_resolution_clock::now();

	while (chrono::duration<double>(chrono::high_resolution_clock::now() - t_start).count() < 180.0)
	{
		cout << "Pic time" << endl;
	}
	
	delay(500);
	pwmWrite(Pwm_pin, Pwm_home_value);
	delay(500);
	cout << "End of pics" << endl;
	
	return 0;
}

