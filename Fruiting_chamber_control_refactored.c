#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pcf8574.h>
#include <lcd.h>

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

// Pin definitions
const int HEAT_PIN = 0;
const int HUM_PIN = 1;
const int FAN_PIN = 2;
const int LIGHT_PIN = 3;
const int LCD_POWER_PIN = 4;

// Setpoints
const float HUM_LOW_SETPOINT = 35.0;
const float HUM_HIGH_SETPOINT = 40.0;
const float TEMP_LOW_SETPOINT = 23.0;
const float TEMP_HIGH_SETPOINT = 25.0;
const int CO2_LOW_SETPOINT = 800;
const int CO2_HIGH_SETPOINT = 1000;

// Light timings
const int START_HOUR = 8;
const int START_MINUTE = 3;
const int END_HOUR = 23;
const int END_MINUTE = 39;

// LCD settings
int pcf8574_address = 0x27; // Default address
const int BASE = 64;
const int RS = BASE + 0;
const int RW = BASE + 1;
const int EN = BASE + 2;
const int LED = BASE + 3;
const int D4 = BASE + 4;
const int D5 = BASE + 5;
const int D6 = BASE + 6;
const int D7 = BASE + 7;
int lcdhd;

void initializeSCD41(void) {
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
        printf("Error executing scd4x_start_periodic_measurement(): %i\n", error);
    }

    printf("Waiting for first measurement... (5 sec)\n");	
}

int detectI2C(int addr) {
    int _fd = wiringPiI2CSetup(addr);   
    if (_fd < 0) {		
        printf("Error address : 0x%x \n", addr);
        return 0;
    } else {	
        if (wiringPiI2CWrite(_fd, 0) < 0) {
            printf("Not found device in address 0x%x \n", addr);
            return 0;
        } else {
            printf("Found device in address 0x%x \n", addr);
            return 1;
        }
    }
}

void initializeLCD() {
    int i;

    if (detectI2C(0x27)) {
        pcf8574_address = 0x27;
    } else if (detectI2C(0x3F)) {
        pcf8574_address = 0x3F;
    } else {
        printf("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
        exit(1);
    }

    pcf8574Setup(BASE, pcf8574_address);
    for (i = 0; i < 8; i++) {
        pinMode(BASE + i, OUTPUT);
    } 

    digitalWrite(LED, HIGH);
    digitalWrite(RW, LOW);
    lcdhd = lcdInit(2, 16, 4, RS, EN, D4, D5, D6, D7, 0, 0, 0, 0);
    if (lcdhd == -1) {
        printf("lcdInit failed!");
        exit(1);
    }
}

void readMeasurement(uint16_t* co2, float* temperature, float* humidity) {
    sensirion_i2c_hal_sleep_usec(5000000);
    int16_t error = 0;

    error = scd4x_read_measurement(co2, temperature, humidity);
    if (error) {
        printf("Error executing scd4x_read_measurement(): %i\n", error);
    } else if (*co2 == 0) {
        printf("Invalid sample detected, skipping.\n");
    }
}

void lightTimer(void) {
    time_t current_time;
    struct tm *time_info;
    int hour, minute;

    time(&current_time);
    time_info = localtime(&current_time);
    hour = time_info->tm_hour;
    minute = time_info->tm_min;

    if ((hour == START_HOUR && minute >= START_MINUTE) || 
        (hour == END_HOUR && minute <= END_MINUTE) || 
        (hour > START_HOUR && hour < END_HOUR)) {
        digitalWrite(LIGHT_PIN, HIGH);
    } else {
        digitalWrite(LIGHT_PIN, LOW);
    }
}

void controlHumidifier(float humidity) {
    if (humidity < HUM_LOW_SETPOINT) {
        digitalWrite(HUM_PIN, HIGH);
    } else if (humidity > HUM_HIGH_SETPOINT) {
        digitalWrite(HUM_PIN, LOW);
    }
}

void controlHeater(float temperature) {
    if (temperature < TEMP_LOW_SETPOINT) {
        digitalWrite(HEAT_PIN, HIGH);
    } else if (temperature > TEMP_HIGH_SETPOINT) {
        digitalWrite(HEAT_PIN, LOW);
    }
}

void controlFan(uint16_t co2) {
    if (co2 < CO2_LOW_SETPOINT) {
        digitalWrite(FAN_PIN, LOW);
    } else if (co2 > CO2_HIGH_SETPOINT) {
        digitalWrite(FAN_PIN, HIGH);
    }
}

void printToLCD(uint16_t co2, float temperature, float humidity) {
    lcdPosition(lcdhd, 0, 0);
    lcdPrintf(lcdhd, "T:%.1fC,RH:%.1f", temperature, humidity);
    lcdPosition(lcdhd, 0, 1);
    lcdPrintf(lcdhd, "CO2:%u ppm", co2);
}

void write_to_csv(uint16_t co2, float temperature, float humidity) {
    FILE *fp = fopen("output.csv", "a");
    if (!fp) {
        printf("Error: could not open file for writing.\n");
        return;
    }

    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);

    fprintf(fp, "%s,%u,%.2f,%.2f\n", timestamp, co2, temperature, humidity);
    fclose(fp);
}

void sigintHandler(int sig_num) {
    lcdDisplay(lcdhd, 0);
    pinMode(HEAT_PIN, INPUT);
    pinMode(HUM_PIN, INPUT);
    pinMode(FAN_PIN, INPUT);
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LCD_POWER_PIN, INPUT);
    printf("\n Ctrl+C pressed. Exiting.\n");
    exit(0);
}

void setup() {
    printf("Program is starting ... \n");
    signal(SIGINT, sigintHandler);

    wiringPiSetup();
    pinMode(HEAT_PIN, OUTPUT);
    pinMode(HUM_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    pinMode(LCD_POWER_PIN, OUTPUT);

    digitalWrite(LCD_POWER_PIN, HIGH);
    delay(1000);
    initializeLCD();
    delay(1000);
    initializeSCD41();
}

void loop() {
    uint16_t co2;
    float temperature;
    float humidity;

    readMeasurement(&co2, &temperature, &humidity);

    lightTimer();
    controlHumidifier(humidity);
    controlHeater(temperature);
    controlFan(co2);

    printToLCD(co2, temperature, humidity);
    write_to_csv(co2, temperature, humidity);

    delay(1000);
}

int main(void) {
    setup();
    while (1) {
        loop();
    }
    return 0;
}
