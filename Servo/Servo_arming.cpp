#include "Servo_arming.h"
#include "wiringPi.h"
#include <iostream>

using namespace std;

//Defualt constructor
Servo::Servo()
{

}

//Constructor that creates the home and max pwm signals
void Servo::Servo_cal(int Pwm_pin, float Pwm_init_val, vector<int> homing_limit_switches, vector<int> max_limit_switches)
{
    pwmWrite(Pwm_pin, Pwm_init_val);        //Set the servo position to the initial PWM value, should be between home and max switches
    printf("Servo set to initial PWM value of %g\n", Pwm_init_val);     //Print a command for the intial PWM value the servo is set to
    delay(1000);        //Delay of 1 second so that the servo has time to get to this initial PWM command
    this->arming.Pwm_max_value = Pwm_init_val;       //Set the PWM max value as the initial value before entering the incremental while step

    //This while loop checks if any of the max limit switches are actuated, if they are, stop and store the PWM max value
    while(digitalRead(max_limit_switches[0]) == LOW && digitalRead(max_limit_switches[1]) == LOW && digitalRead(max_limit_switches[2]) == LOW && digitalRead(max_limit_switches[3]) == LOW)
    {
        this->arming.Pwm_max_value += 0.09;       //increase the PWM value by (180/1024)/2;
        if(arming.Pwm_max_value > 1024)        //Checking to make sure the PWM command is not above the 1024 threshold, if it is, break
        {
            printf("Maximum detection failed, tried to return PWM value greater than 1024 \nPWM_max_value = %g\n", arming.Pwm_max_value);
            return 0;       //break out of main
        }
        pwmWrite(Pwm_pin, arming.Pwm_max_value);       //Set the servo to the new PWM value
        delay(100);     //Delay for 0.1s
    }
    //End of the maximum detection

    pwmWrite(Pwm_pin, Pwm_init_val);        //Set the servo back to the initial PWM value, between max and home
    delay(1000);        //Delay of 1 s
    this->arming.Pwm_home_value = Pwm_init_val;      //set the PWM home value to the intitial PWM value

    //This while loop checks if any of the homing switches are actuated, if they are, stop and store the PWM home value
    while(digitalRead(homing_limit_switches[0]) == LOW && digitalRead(homing_limit_switches[0]) == LOW)
    {
        this->arming.Pwm_home_value -= 0.09;     //decrement the PWM home value by (180/1024)/2;
        if(arming.Pwm_home_value < 0)      //Checks to see if the PWM command is less than the minimum threshold of 0
        {
            printf("Homing failed, tried to return PWM value less than 0 \nPwm_home_value = %g\n", arming.Pwm_home_value);
            return 0;       //Break out of main because of min PWM threshold violation
        }
        pwmWrite(Pwm_pin, arming.Pwm_home_value);      //Set the servo motor to the new PWM command
        delay(100);     //Delay of 0.1s
    }

    printf("Arming is complete: \nMaximum PWM Value: %g \nHome PWM Value: %g\n", arming.Pwm_max_value, arming.Pwm_home_value);      //Print that arming is complete
    //Show the max and home PWM values, these will be returned as the OUTPUTS in the airbrake software
}

float Servo::get_Pwm_init_value()
{
    return this->arming.Pwm_init_val;
}

double Servo::get_Pwm_home_value()
{
    return this->arming.Pwm_home_value;
}

double Servo::get_Pwm_max_value()
{
    return this->arming.Pwm_max_value;
}