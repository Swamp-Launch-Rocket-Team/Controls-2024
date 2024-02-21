#include <wiringPi.h>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cmath>

using namespace std;

const int Pwm_pin = 23;      //GPIO PWM pin
const int Pwm_init_val = 0;     //Initial PWM value to start the servo at

int main()
{
    wiringPiSetup();        //Access the wiringPi library
    pinMode(Pwm_pin, PWM_OUTPUT);       //Set the PWM pin as a PWM OUTPUT
    pwmSetMode(PWM_MODE_MS);

    int PWM_prescaler = 48;     //Base freq is 19.2 MHz, THIS IS THE VALUE U CHANGE 
    int pwm_range = 1000 * 48 / PWM_prescaler;
    int min_Pwm = pwm_range/5;
    
    pwmSetClock(PWM_prescaler);
    pwmSetRange(pwm_range);

    printf("Servo PWM Testing \nPWM Pin: %d\n", Pwm_pin);       //Print that testing is beginning and the PWM pin

    // pwmWrite(Pwm_pin, 285);
    // delay(1000);

    // for(int i = min_Pwm; i < pwm_range; i = i + 20)       //For loop for incrementally increasing the PWM signal to the servo
    // {
    //     pwmWrite(Pwm_pin, i);       //writes the new PWM signal to the servo
    //     delay(500);     //0.5s delay
    //     printf("PWM Value: %d\n", i);       //Prints the current PWM value
        
    // }

    //Finding the pwm signals that correspond to zero and 180
    //Remember the servo rotates counter clockwise for our application
    //For 0:
    //pwmWrite(Pwm_pin, 277);
    
    //For 180:
    //pwmWrite(Pwm_pin, 277 + 480);

    //Test to hit 105 degree using the 0 and 180 values
    float m = (480.0/180.0);        //THIS IS THE ACTUAL SLOPE WE USE
    float b = 345;                  //THIS IS THE B WE USE, THIS IS ALSO THE PWM_HOME VALUE AKA 0 DEGREE EXTENSION
    
    //int deg_105 = round(m*105 + b);
    //pwmWrite(Pwm_pin, deg_105);

    //Aibrake flap extension test from 0 degrees to a specified value in the int ext definition line.
    pwmWrite(Pwm_pin, b);
    // delay(1000);
    // int ext = round(m*90 + b);

    // for(int i = 0; i < 5; i++)
    // {
    //     pwmWrite(Pwm_pin, b);
    //     delay(1000);
    //     pwmWrite(Pwm_pin, ext);
    //     delay(1000);
    // }
    
    //pwmWrite(Pwm_pin, b);

    //Aibrake flap extension test from 0 degrees to 105 degrees
    //Use m and b to create a mapping from 0 degrees to 105 degrees.
    // int pwm_command;
    // for(int i = 0; i < 106; i = i + 5)
    // {
    //     pwm_command = round(m*i + b);
    //     delay(250);
    //     pwmWrite(Pwm_pin, pwm_command);
    //     cout << pwm_command << endl;        
    // }

    cout << "Testing done" << endl;     //Prints to the command that testing has concluded

    return 0;       //Ends the main loop
}