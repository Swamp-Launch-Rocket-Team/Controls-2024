#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

using namespace std;

const int Pwm_pin = 1;      //GPIO hardware PWM pin
const vector<int> homing_limit_switches{1,2};       //vector of GPIO pins for the homing limit switches
const vector<int> max_limit_switches{3,4,5,6};      //vector of GPIO pins for the max limit switches
const float Pwm_init_val = 500;        //initial PWM value for the servo, should correspond to around halfway between home and max
double Pwm_home_value;      //final home PWM value, this will be an output of the Arming state
double Pwm_max_value;       //final max PWM value, this will be an output of the Arming state

//This is a test script for the on-the-pad arming of the servo, setting its max and home PWM values
int main()
{
    wiringPiSetup();        //access the wiringPi library
    pinMode(Pwm_pin, PWM_OUTPUT);       //setting the Pwm pin as a PWM output pin
    printf("Servo Arming Testing \nPWM Pin: %g\n", Pwm_pin);        //Beginning of test

    for(int i = 0; i < homing_limit_switches.size(); i++)       //Defining the pins for the home limit switches
    {
       pinMode(homing_limit_switches[i], INPUT);        //Set eatch home limit switch as an input
       printf("Homing limit switch %g has been initialized\n", homing_limit_switches[i]);       //Print a command for each home switch initialized
    }

    for(int i = 0; i < max_limit_switches.size(); i++)      //Defininig the pins for the max limit switches
    {
        pinMode(max_limit_switches[i], INPUT);      //Sets each max limit switch as an input
        printf("Max limit switch %g has been initialized\n", max_limit_switches[i]);        //Print a command for each max switch initialized
    }

    pwmWrite(Pwm_pin, Pwm_init_val);        //Set the servo position to the initial PWM value, should be between home and max switches
    printf("Servo set to initial PWM value of %g\n", Pwm_init_val);     //Print a command for the intial PWM value the servo is set to
    delay(1000);        //Delay of 1 second so that the servo has time to get to this initial PWM command
    Pwm_max_value = Pwm_init_val;       //Set the PWM max value as the initial value before entering the incremental while step

    //This while loop checks if any of the max limit switches are actuated, if they are, stop and store the PWM max value
    while(digitalRead(max_limit_switches[0]) == LOW && digitalRead(max_limit_switches[1]) == LOW && digitalRead(max_limit_switches[2]) == LOW && digitalRead(max_limit_switches[3]) == LOW)
    {
        Pwm_max_value = Pwm_max_value + 0.09;       //increase the PWM value by (180/1024)/2;
        if(Pwm_max_value > 1024)        //Checking to make sure the PWM command is not above the 1024 threshold, if it is, break
        {
            printf("Maximum detection failed, tried to return PWM value greater than 1024 \nPWM_max_value = %g\n", Pwm_max_value);
            return 0;       //break out of main
        }
        pwmWrite(Pwm_pin, Pwm_max_value);       //Set the servo to the new PWM value
        delay(100);     //Delay for 0.1s
    }
    //End of the maximum detection

    pwmWrite(Pwm_pin, Pwm_init_val);        //Set the servo back to the initial PWM value, between max and home
    delay(1000);        //Delay of 1 s
    Pwm_home_value = Pwm_init_val;      //set the PWM home value to the intitial PWM value

    //This while loop checks if any of the homing switches are actuated, if they are, stop and store the PWM home value
    while(digitalRead(homing_limit_switches[0]) == LOW && digitalRead(homing_limit_switches[0]) == LOW)
    {
        Pwm_home_value = Pwm_home_value - 0.09;     //decrement the PWM home value by (180/1024)/2;
        if(Pwm_home_value < 0)      //Checks to see if the PWM command is less than the minimum threshold of 0
        {
            printf("Homing failed, tried to return PWM value less than 0 \nPwm_home_value = %g\n", Pwm_home_value);
            return 0;       //Break out of main because of min PWM threshold violation
        }
        pwmWrite(Pwm_pin, Pwm_home_value);      //Set the servo motor to the new PWM command
        delay(100);     //Delay of 0.1s
    }

    printf("Arming is complete: \nMaximum PWM Value: %g \nHome PWM Value: %g\n", Pwm_max_value, Pwm_home_value);      //Print that arming is complete
    //Show the max and home PWM values, these will be returned as the OUTPUTS in the airbrake software

    return 0;
}