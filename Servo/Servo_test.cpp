#include <wiringPi.h>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

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

    pwmWrite(Pwm_pin, 285);
    delay(1000);

    // for(int i = min_Pwm; i < pwm_range; i = i + 20)       //For loop for incrementally increasing the PWM signal to the servo
    // {
    //     pwmWrite(Pwm_pin, i);       //writes the new PWM signal to the servo
    //     delay(500);     //0.5s delay
    //     printf("PWM Value: %d\n", i);       //Prints the current PWM value
        
    // }

    cout << "Testing done" << endl;     //Prints to the command that testing has concluded

    return 0;       //Ends the main loop
}