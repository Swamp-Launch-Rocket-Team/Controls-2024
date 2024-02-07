#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

using namespace std;

const int Pwm_pin = 1;      //GPIO PWM pin
const int Pwm_init_val = 0;     //Initial PWM value to start the servo at

int main()
{
    wiringPiSetup();        //Access the wiringPi library
    pinMode(Pwm_pin, PWM_OUTPUT);       //Set the PWM pin as a PWM OUTPUT
    printf("Servo PWM Testing \nPWM Pin: %d\n", Pwm_pin);       //Print that testing is beginning and the PWM pin

    for(int i = 0; i < 1024; i++)       //For loop for incrementally increasing the PWM signal to the servo
    {
        pwmWrite(Pwm_pin, i);       //writes the new PWM signal to the servo
        delay(500);     //0.5s delay
        printf("PWM Value: %d\n", i);       //Prints the current PWM value
    }

    cout << "Testing done" << endl;     //Prints to the command that testing has concluded

    return 0;       //Ends the main loop
}