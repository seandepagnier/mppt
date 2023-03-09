/* Copyright (C) 2023 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */


/* TODO:

real time clock for all logging
app the display and visualize data
update schematic with sync amp on high side for boosting, and isolated power for 100% duty

add hydro mppt algorithms
*/

//#define USE_BT
//#define USE_SD

#ifdef USE_BT

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
#endif

#ifdef USE_SD
#include "FS.h"
#include "SD.h"
#endif

float vmppt, duty;


#include <math.h>

const int tempInPin = 32;
const int iddInPin = 25;
const int iccInPin = 26;
const int vddInPin = 34;
const int vccInPin = 35;

const int pwrdwnPin = 2; // turn off 12v

const int buck_lPin = 14;
const int buck_hPin = 15;
const int boost_lPin = 12;
const int boost_hPin = 13;

const int ledPin = 27;

int minVoltage = 7;
int maxVoltage = 32;
int maxInputCurrent = 7;
float startVoltage = 13.4f;
float endVoltage = 13.8f;

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"



void setupSD() {
#ifdef USE_SD
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
  
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Size: %lluMB\n", cardSize);
        Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    }
#endif
}

void log(const char *message)
{
    Serial.println(message);
    Serial.flush();
#ifdef USE_BT
    for(const char *c = message; *c; c++)
        SerialBT.write(*c);
#endif
}

void writedata(const char *message) {
#ifdef USE_SD
    static char buffer[8192];
    strncat(buffer, message, 256);
    if(strlen(buffer) < sizeof buffer - 256)
        return;

    fs::FS &fs = SD;
    const char * path = "log.txt";
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file) {
        Serial.println("Failed to open file for appending");
        buffer[0] = 0;
        return;
    }
    if(!file.print(buffer))
        Serial.println("Append failed");

    file.close();
    buffer[0] = 0;
#endif
}


enum {IDLE, STANDBY, MPPT} state = IDLE;

float readVoltage(int pin) {
    int sensorValue = analogRead(pin);
    return 3.3*sensorValue/4096*350/10*1.45;
}

float readCurrent(int pin) {
    int sensorValue = analogRead(pin);
    return 3.3 * sensorValue * 10 / 4096;
}

float readTemperature() {
    int x = analogRead(tempInPin)+140;
    float r = 4096.0/x-1;
    float T = 1.0 / (1.0 / 298.0 + 1/3905.0 * log(r)) - 273.0;
    return T;
}


float vcc, vdd, icc, idd, temperature;
long lastt, ttotal;
unsigned long statechange = millis();

/*
Current ripple or max current in discontinuous buck mode
    current_max = duty / frequency / inductance * (vdd - vcc)

at continous/discontinuous point
    duty = vcc / vdd

in discontinuous mode,  this is the max current in the inductor

to solve the time to discharge inductor
    time_dis = current * inductance / vcc

at discontinuous point
    time = (1-duty)/frequency

for off time (diode conducting)
    time_dis = duty / frequency * (vdd/vcc - 1)


for boost dicontinuous mode
   current_max = duty/frequency/inductance*vdd

at discontinuous point
   duty = 1-vdd/vcc

   time_dis = current * inductance / (vcc - vdd)
   time_dis = duty/frequency  / (vcc/vdd-1)

*/
float minduty()
{
    if(vcc > vdd)
        return 1 - vdd / vcc;
    else
        return vcc / vdd - 1;

//    duty += .1; // add some margin to ensure forward transfer
}

float maxduty()
{
    // dont boost more than 4x input or up to max voltage
    return min(1 - vdd/maxVoltage, .75f);
}

void control_log(const char *message)
{
    Serial.print("status: ");
    Serial.println(message);
}

void idle(const char *reason)
{
    if(state == IDLE)
        return;

    pinMode(pwrdwnPin, OUTPUT);
    digitalWrite(pwrdwnPin, HIGH);

    delay(1);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);

    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

    state = IDLE;
    Serial.print("STATE: idle: ");
    Serial.println(reason);
}

void controlloop()
{
    int laststate = state;
    
    if(vcc > maxVoltage)
        idle("Output over voltage");
    else if(vdd > maxVoltage)
        idle("Input over voltage");
    else if(idd > maxInputCurrent)
       idle("Output over current");
    else if(icc > maxInputCurrent)
        idle("Input over current");
    else if(vcc > endVoltage)
        idle("Reached end voltage");
    else if(vcc < minVoltage || vdd < minVoltage)
        idle("min voltage");

    float min_duty = minduty(), max_duty = maxduty();
    if(max_duty - min_duty < .2)
      idle("insufficient duty range");
    
    switch(state) {
    case IDLE:
        pinMode(buck_hPin, OUTPUT);
        digitalWrite(buck_hPin, LOW);
        pinMode(buck_lPin, OUTPUT);
        digitalWrite(buck_lPin, LOW);

        digitalWrite(ledPin, HIGH); // turn LED off

        if(millis() - statechange > 2000) {
            if(vdd < minVoltage || vcc > startVoltage) {
                const unsigned long long uS_TO_S_FACTOR = 1000000ULL;
                esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
                if(vdd < minVoltage)
                    control_log("insufficient input voltage");
                else {
                    control_log("output voltage exceeds start voltage");
                    Serial.print(vcc);
                    Serial.print(" > ");
                    Serial.println(startVoltage);
                }

                control_log("going to sleep now");
                esp_deep_sleep_start();
            } else if(vcc < startVoltage) {
                state = STANDBY; // otherwise turn on
                control_log("STATE: standby");
            }
        }
        
        delay(20);
        break;

    case STANDBY:
        digitalWrite(ledPin, LOW); // turn LED ON

        pinMode(pwrdwnPin, OUTPUT);
        digitalWrite(pwrdwnPin, LOW); // turn on 12v power

        // charge bootstrap
        delay(1);
        digitalWrite(buck_lPin, HIGH);
        delay(1);

        vmppt = (vdd+vcc)/2;
        if(millis() - statechange > 3000) {
            control_log("STATE: mppt");
            duty = minduty();
            state = MPPT;

            digitalWrite(buck_lPin, LOW);

            // setup up pwm output
            mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, buck_hPin);
            //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, buck_lPin);

            mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, boost_hPin);
            mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, boost_lPin);
            

//            mcpwm_group_set_resolution(MCPWM_UNIT_0, 80000000);
//            mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 10000000);

            // use 40khz frequency
            mcpwm_config_t pwm_config;
            pwm_config.frequency = 20000;
            pwm_config.cmpr_a = 0;
            pwm_config.cmpr_b = 0;
            pwm_config.counter_mode = MCPWM_UP_COUNTER;
            pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

            // setup deadtime to ensure both transistors are off
            // 3 is 3x100ns    The transistors appear to switch in 100ns
            mcpwm_deadtime_enable(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3);
        
            mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above


            pwm_config.frequency = 2000;
            pwm_config.cmpr_a = 0;
            pwm_config.cmpr_b = 0;
            pwm_config.counter_mode = MCPWM_UP_COUNTER;
            pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
            mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);   //Configure PWM0A & PWM0B with above            

            mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
            mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_1);
            
            mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,0);
            //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,99);
            
            mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,0);
            mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_B,0);
        }
        break;

    case MPPT:
    {
        float err = (vdd - vmppt);
        duty += .0005 * err; // adjust duty to follow mppt voltage

        if(idd > maxInputCurrent)
            duty -= .1 * (idd - maxInputCurrent);

        duty = min(max(duty, min_duty), max_duty);


        if(duty < -.01) {
            mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,100*(1+duty));
            //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,0);

            mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,99.5);
            mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_B,99.5);
        } else if(duty > .01) {
            // for now not boosting
            if(duty > 0)
                duty = 0;

            //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,99);
            //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,0);

            //mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,0);
            //mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_B,100*duty);
        } else {
            // duty is nearly neutral,  just keep both sides on
            mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,99);
            //mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,0);

            //mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,0);
            //mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_B,100*duty);
        }


        static float trackingdir = .1, avgpout, lastpout;
        static int trackingcount = 0;
        if(icc > 0.2)
            if(fabs(vdd-vmppt) < .15 || duty == max_duty || duty == min_duty) {
                float pout = vcc*icc;
                avgpout += pout;
                trackingcount++;
            }

        float pin = vdd*idd, pout = vcc*icc;
        if(trackingcount == 10) {
            avgpout /= trackingcount;
            trackingcount = 0;
            avgpout = 0;

            if(lastpout > avgpout) // change dir
                trackingdir = -trackingdir;
            lastpout = avgpout;

            if(duty == max_duty) {
                trackingdir = fabs(trackingdir);
                Serial.println("max duty mppt");
            } else if(duty == min_duty) {
                trackingdir = -fabs(trackingdir);
                Serial.println("min duty mppt");
            }

            vmppt += trackingdir; // update voltage set point


            if(duty >= 0)
                idle("Buck mode only!");
        }
    }  break;
    }
    
    if(temperature > 45)
        idle("OverTemperature");

    if(state != laststate)
        statechange = millis();
}

int global_samplecount;
void control(void *param)
{
    unsigned long t0 = millis();
    const int loop_period = 100;
    for(;;) {
        int samplecount = 0;
        float vcc_in = 0, vdd_in = 0, icc_in = 0, idd_in = 0;
        float temperature_in = 0;
        while(samplecount < 10 || millis()-t0 < loop_period - 10) {
            vcc_in+=readVoltage(vccInPin);
            vdd_in+=readVoltage(vddInPin);
            icc_in+=readCurrent(iccInPin);
            idd_in+=readCurrent(iddInPin);

            samplecount++;
        }

        temperature_in = readTemperature();

        temperature = temperature*.95+temperature_in*.05;
        
        global_samplecount = samplecount;

        vcc = vcc_in / samplecount;
        vdd = vdd_in / samplecount;
        icc = icc_in / samplecount;
        idd = idd_in / samplecount;
        
        controlloop();
        unsigned long t1 = millis();
        ttotal = t1 - t0;

        if(ttotal < loop_period) {
            unsigned long dt = ttotal > loop_period ? loop_period : ttotal;
            const TickType_t xDelay = (loop_period - ttotal) / portTICK_PERIOD_MS;
            vTaskDelay( xDelay );
            t0 += loop_period;
        } else
            t0 = t1;
    }
}

TaskHandle_t Control;                 //SYSTEM PARAMETER  - Used for the ESP32 dual core operation
void setup() {

#ifdef USE_BT
    SerialBT.begin("mppt-node"); //Bluetooth device name
#endif
    Serial.begin(115200);

    setupSD();

    pinMode(pwrdwnPin, OUTPUT);
    digitalWrite(pwrdwnPin, HIGH);
    
    // initialize serial communications
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);


    int sketchsize = ESP.getSketchSize();
    delay(100);
    uint64_t fusemac = ESP.getEfuseMac();
    uint8_t* chipid = (uint8_t*) & fusemac;
    char mac[20];
    snprintf(mac, sizeof mac, "%02x:%02x:%02x:%02x:%02x:%02x", chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5]);
    Serial.print("mac ");
    Serial.println(mac);

    xTaskCreatePinnedToCore(control,"control",10000,NULL,9/*priority*/,&Control,0);

    delay(200);
}


void loop()
{
    char buf[256];
    float pin = vdd*idd, pout = vcc*icc;
    float efficiency;
    if(pin)
        efficiency = pout / pin * 100.0f;
    else
        efficiency = 0;

    if(vcc || vdd) {
        sprintf(buf, "mppt (%.2fV %.2fA %.1fW) (%.2fV %.2fA %.1fW) %.4f%% %.2fV %.1fC %.1f%% %ldms %d", vdd, idd, pin, vcc, icc, pout, 100.0f*duty, vmppt, temperature, efficiency, ttotal, global_samplecount);
        log(buf);
    }
//writ_edata(buf);
    //vTaskGetRunTimeStats(buf);

    //Serial.println(buf);

    // Wait for the next cycle.
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    vTaskDelay( xDelay );
}
