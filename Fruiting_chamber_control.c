/**********************************************************************
* Use the following command to compile: gcc -Wall scd4x_i2c.c sensirion_i2c.c sensirion_i2c_hal.c sensirion_common.c Fruiting_chamber_control.c -o Fruiting_chamber_control  -lwiringPi -lADCDevice  -lm -lpthread -lwiringPiDev
run with: sudo ./Fruiting_chamber_control
**********************************************************************/
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <pcf8574.h>
#include <lcd.h>
#include <time.h>

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#define  heatPin    0	//define the heater relay pin number
#define  humPin    1	//define the humidifier relay pin number
#define  fanPin    2	//define the fan relay pin number
#define  lightPin    3	//define the light relay pin number
#define  LCDPowerPin    4	//define the LCD switch NPN pin number


//define setpoints for light on/off times
#define START_HOUR 8
#define START_MINUTE 3
#define END_HOUR 23
#define END_MINUTE 39

//define setpoints for humidity
#define  humLowSetpoint    35.0
#define  humHighSetpoint    40.0

//define setpoints for temperature
#define  tempLowSetpoint    23.0
#define  tempHighSetpoint    25.0

//define setpoints for c02
#define  co2LowSetpoint    800
#define  co2HighSetpoint    1000

int pcf8574_address = 0x27;        // PCF8574T:0x27, PCF8574AT:0x3F
#define BASE 64         // BASE any number above 64
//Define the output pins of the PCF8574, which are directly connected to the LCD1602 pin.
#define RS      BASE+0
#define RW      BASE+1
#define EN      BASE+2
#define LED     BASE+3
#define D4      BASE+4
#define D5      BASE+5
#define D6      BASE+6
#define D7      BASE+7

int lcdhd;// used to handle LCD

void initializeSCD41(void){
    int16_t error = 0;

    sensirion_i2c_hal_init();

    // Clean up potential SCD41 states
    scd4x_wake_up();
    scd4x_stop_periodic_measurement();
    scd4x_reinit();

    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
    if (error) {
        printf("Error executing scd4x_get_serial_number(): %i\n", error);
    } else {
        printf("serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
    }

    // Start Measurement

    error = scd4x_start_periodic_measurement();
    if (error) {
        printf("Error executing scd4x_start_periodic_measurement(): %i\n",
               error);
    }

    printf("Waiting for first measurement... (5 sec)\n");	
}

int detectI2C(int addr){
    int _fd = wiringPiI2CSetup (addr);   
    if (_fd < 0){		
        printf("Error address : 0x%x \n",addr);
        return 0 ;
    } 
    else{	
        if(wiringPiI2CWrite(_fd,0) < 0){
            printf("Not found device in address 0x%x \n",addr);
            return 0;
        }
        else{
            printf("Found device in address 0x%x \n",addr);
            return 1 ;
        }
    }
}

void initializeLCD(){
    int i;

    if(detectI2C(0x27)){
        pcf8574_address = 0x27;
    }else if(detectI2C(0x3F)){
        pcf8574_address = 0x3F;
    }else{
        printf("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
    }
    pcf8574Setup(BASE,pcf8574_address);//initialize PCF8574
    for(i=0;i<8;i++){
        pinMode(BASE+i,OUTPUT);     //set PCF8574 port to output mode
    } 
    digitalWrite(LED,HIGH);     //turn on LCD backlight
    digitalWrite(RW,LOW);       //allow writing to LCD
	lcdhd = lcdInit(2,16,4,RS,EN,D4,D5,D6,D7,0,0,0,0);// initialize LCD and return “handle” used to handle LCD
    if(lcdhd == -1){
        printf("lcdInit failed !");
    }

}

void readMeasurement(uint16_t* co2, float* temperature, float* humidity){
    // Read Measurement
    sensirion_i2c_hal_sleep_usec(5000000);
    int16_t error = 0;

    error = scd4x_read_measurement(co2, temperature, humidity);
    if (error) {
        printf("Error executing scd4x_read_measurement(): %i\n", error);
    } else if (co2 == 0) {
        printf("Invalid sample detected, skipping.\n");
    } else {
        // printf("CO2: %u ppm\n", co2);
        // printf("Temperature: %.2f °C\n", temperature);
        // printf("Humidity: %.2f RH\n", humidity);
    }
}

void lightTimer(void){
    time_t current_time;
    struct tm *time_info;
    int hour, minute;

    // Get the current time
    time(&current_time);
    time_info = localtime(&current_time);
    hour = time_info->tm_hour;
    minute = time_info->tm_min;

    // Check if it's time to set the pin mode
    if ((hour == START_HOUR && minute >= START_MINUTE) || 
        (hour == END_HOUR && minute <= END_MINUTE) || 
        (hour > START_HOUR && hour < END_HOUR)) {
        digitalWrite(lightPin, HIGH);
    } else {
        digitalWrite(lightPin, LOW);
    }

}

void humidifier(float humidity) {
    if (humidity < humLowSetpoint) {
        digitalWrite(humPin, HIGH);  // Set humPin to HIGH
    } else if (humidity > humHighSetpoint) {
        digitalWrite(humPin, LOW);   // Set humPin to LOW
    }
}

void heater(float temperature) {
    if (temperature < tempLowSetpoint) {
        digitalWrite(heatPin, HIGH);  // Set humPin to HIGH
    } else if (temperature > tempHighSetpoint) {
        digitalWrite(heatPin, LOW);   // Set humPin to LOW
    }
}

void fan(uint16_t co2) {
    if (co2 < co2LowSetpoint) {
        digitalWrite(fanPin, LOW);  // Set humPin to HIGH
    } else if (co2 > co2HighSetpoint) {
        digitalWrite(fanPin, HIGH);   // Set humPin to LOW
    }
}

void printToLCD(uint16_t co2, float temperature, float humidity){// sub function used to print CPU temperature

    lcdPosition(lcdhd,0,0);     // set the LCD cursor position to (0,0) 
    lcdPrintf(lcdhd,"T:%.1fC,RH:%.1f",temperature,humidity);// Display temperature and humidity
    lcdPosition(lcdhd,0,1);// set the LCD cursor position to (0,1) 
    lcdPrintf(lcdhd,"CO2:%u ppm",co2); //Display co2 level on LCD
}

void write_to_csv(uint16_t co2, float temperature, float humidity) {

    FILE *fp;

    fp = fopen("output.csv", "a");
    if (fp == NULL) {
        printf("Error: could not open file for writing.\n");
        return;
    }

    // Get current time
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    char timestamp[20];
    strftime(timestamp, 20, "%Y-%m-%d %H:%M:%S", tm_info);

    // Write values to file
    fprintf(fp, "%s,%.u,%.2f,%f\n", timestamp, co2, temperature, humidity);
    fclose(fp);
}

void sigintHandler(int sig_num)
{
    signal(SIGINT, sigintHandler);
    lcdDisplay(lcdhd,0);
	pinMode(heatPin, INPUT);//Set the pin mode
    pinMode(humPin, INPUT);//Set the pin mode
    pinMode(fanPin, INPUT);//Set the pin mode
    pinMode(lightPin, INPUT);//Set the pin mode
    pinMode(LCDPowerPin, INPUT);//Set the pin mode
    printf("\n Ctrl+C pressed. Exiting.\n");
    exit(0);
}



int main(void)
{	
	printf("Program is starting ... \n");

    signal(SIGINT, sigintHandler); //initialise Ctrl+C handler
	
	wiringPiSetup();	//Initialize wiringPi.
	
    
    

    uint16_t co2;
    float temperature;
    float humidity;

	pinMode(heatPin, OUTPUT);//Set the pin mode
    pinMode(humPin, OUTPUT);//Set the pin mode
    pinMode(fanPin, OUTPUT);//Set the pin mode
    pinMode(lightPin, OUTPUT);//Set the pin mode
    pinMode(LCDPowerPin, OUTPUT);//Set the pin mode

    digitalWrite(LCDPowerPin, HIGH);
    delay(1000);
    initializeLCD();
    delay(1000);
    initializeSCD41();	//Initialize sensor
	
	
	while(1){
        readMeasurement(&co2, &temperature, &humidity);

        lightTimer();
        humidifier(humidity);
        heater(temperature);
        fan(co2);

        printf("CO2: %u ppm\n", co2);
        printf("Temperature: %.2f °C\n", temperature);
        printf("Humidity: %.2f RH\n", humidity);

        printToLCD(co2, temperature, humidity);
        write_to_csv(co2, temperature, humidity);

		delay(1000);						//Wait for 1 second
	}


    return 0;
}
