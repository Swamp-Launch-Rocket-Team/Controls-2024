#include <wiringPi.h>
#include "busynano.h"

int main() {
    int pin = 8;
    wiringPiSetup();
	pinMode(8, OUTPUT);
    while(true)
    {
        digitalWrite(pin, 1);
	    busy10ns(100);

    	digitalWrite(pin, 0);
	    busy10ns(100);
    }
}
