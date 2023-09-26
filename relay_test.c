/**********************************************************************
* 
**********************************************************************/
#include <wiringPi.h>
#include <stdio.h>

#define  relPin    0	//define the relay pin number

void main(void)
{	
	printf("Program is starting ... \n");
	
	wiringPiSetup();	//Initialize wiringPi.
	
	pinMode(relPin, OUTPUT);//Set the pin mode
	printf("Using pin%d\n",relPin);	//Output information on terminal
	while(1){
		digitalWrite(relPin, HIGH);  //Make GPIO output HIGH level
		printf("led turned on >>>\n");		//Output information on terminal
		delay(1000);						//Wait for 1 second
		digitalWrite(relPin, LOW);  //Make GPIO output LOW level
		printf("led turned off <<<\n");		//Output information on terminal
		delay(1000);						//Wait for 1 second
	}
}

