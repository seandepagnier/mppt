/* Copyright (C) 2020 Sean D'Epagnier <seandepagnier@gmail.com>                                                                                                                              *                                                                                                                                                                                           * This Program is free software; you can redistribute it and/or                                                                                                                             * modify it under the terms of the GNU General Public                                                                                                                                       * License as published by the Free Software Foundation; either                                                                                                                              * version 3 of the License, or (at your option) any later version.                                                                                                                         */
                                                                         
///   mppt code

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Hardware SPI nokia5110 display
Adafruit_PCD8544 display = Adafruit_PCD8544(PB10, PA4, PB11);

#include <EEPROM.h>

#define BACKLIGHT PB12
#define LED PC13
#define SENSE PC14
#define DRIVER_POWER PC15  // enable power to drive mosfet gates

bool running = false;

float mppt_voltage = 17;
float max_temp = 55;
float duty;

int keys[] = {PB3, PB4, PB5};
int keys_n = (sizeof keys)/(sizeof *keys);
uint32_t keyt[3];
uint32_t off_time, on_time;
static float lastvin, lastpout;
float vin, vout, iin, iout, pin, pout, temp=25, debug;
int page=1, pages = 6;

struct {
    float off_voltage;
    float on_voltage;
    int backlight;
    int frequency;
    uint32_t sig;
} eemem;


TIM_HandleTypeDef pwm_handle;
const int deadtime = 14; // cycles, .625uS
static uint32_t pwm_max, pwm_min_comp;
static float min_duty, max_duty;


bool need_write = false;
void read_keys() {
    int trig = -1;
    uint32_t t = HAL_GetTick();
    for(int i=0; i<keys_n; i++)
        if(digitalRead(keys[i])) {
          if(keyt[i] && t-keyt[i] > 50) {
               trig = i;
               keyt[i] = 0;
          }
        } else {
          if(!keyt[i])
              keyt[i] = t;
        }

    int mod = 0;
    switch(trig) {
       case 0:
          if(++page == pages)
              page = 0;
          if(need_write)
              EEPROM.put(0, eemem);
          break;
       case 1:
          mod = 1;
          break;
       case 2:
          mod = -1;
          break;
    }

    if(mod)
        if(page < 2)     
            page = !page;
        else {
          float off_voltage = eemem.off_voltage, on_voltage = eemem.on_voltage;
          if(page == 2)
               off_voltage += .1*mod; 
          else if(page == 3)
               on_voltage += .1*mod;
          else if(page == 4) {
              eemem.backlight = eemem.backlight == LOW ? HIGH : LOW;
              digitalWrite(BACKLIGHT, eemem.backlight);
          } else if(page == 5) {
               eemem.frequency += mod;
          }
          
          if(on_voltage < off_voltage - .1) {
              eemem.off_voltage = off_voltage;
              eemem.on_voltage = on_voltage;
          }
          need_write = true;
        }
}

void display_number(float v)
{
    display.setTextSize(2);
    int n, x, y = display.getCursorY();
    if(v<100) {
        n = v*100;
        x = 23;
    }    else {
        n = v*10;
        x = 34;
    }
    for(int i=0; i<4; i++) {
        display.setCursor((3-i)*12,y);
        display.print(n%10);
        n/=10;
    }

    // draw decimal without wasting horizontal space    
    display.fillCircle(x, y+14, 1, BLACK);
    display.println("");
} 

void display_default()
{
    display.setTextColor(BLACK);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("voltage");
    display_number(vout);
    display.setTextSize(1);
    display.println("watts");
    display_number(pout);

    if(running) {
        display.setTextSize(1);
        display.println("\nmppt");
        display.println(mppt_voltage);
    } else
        display.println("\nOFF");
}

void display_info()
{
    static uint32_t lt;
    uint32_t t = HAL_GetTick();
    uint32_t dt = t - lt;
    lt = t;
  
    // rotation example
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.print("Vi ");

    display.println(int(vin*100)/100.0);
    display.print("Ii ");
    display.println(iin);
    display.print("Pi ");
    display.println(pin);
    display.print("Vo ");
    display.println(int(vout*100)/100.0);
    display.print("Io ");
    display.println(iout);
    display.print("Po ");
    display.println(pout);

    if(running) {
        display.print("Eff ");
        display.print(int(pout / pin * 100.0) );
        display.println("%");
        display.print("du ");
        display.println(int(10000*duty)/100.0);    
    } else
        display.println("OFF");

    //display.print ("hz ");
    //display.println(dt);
    //display.print("t ");
    //display.println(temp);
  
    display.println(mppt_voltage);
    display.println(debug);  
}

void display_set_off_voltage()
{
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.println("PV Off");
    display.print(eemem.off_voltage);
    display.println("v");
}

void display_set_on_voltage()
{
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.println("PV On");
    display.print(eemem.on_voltage);
    display.println("v");
}

void display_set_backlight()
{
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.println("back\nlight");
    display.println(eemem.backlight == LOW ? "on" : "off");
    display.println("uses\n36mW");
}

int get_frequency()
{
    switch(eemem.frequency) {
        case 1:
           return 20000;
        case 2:
           return 40000;
        case 3:
           return 80000;
        default:
           eemem.frequency = 0;
           return 10000;
    }
}

void display_set_frequency()
{
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.println("frequency");

    display.println(get_frequency());
}


void setup()   {  
    // enable led output
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); // off
  
    // enable sense and reference
    digitalWrite(SENSE, HIGH);
    pinMode(SENSE, OUTPUT);

    // set input keys internal pullup resistor
    for(int i=0; i<keys_n; i++)
        pinMode(keys[i], INPUT_PULLUP);

    display.begin();
    display.setContrast(50);

    display.display(); // show splashscreen

    analogReadResolution(12); // set adc to use full 12 bits

    // read eeprom settings for panel on and off voltage
    EEPROM.get(0, eemem);
    uint32_t sig = 0xF2BB328F;
    if(eemem.sig != sig) {//
        eemem.off_voltage = 13.6; // turn off above this voltage
        eemem.on_voltage = 13.2;
        eemem.backlight = LOW;
        eemem.frequency = 0;
        eemem.sig = sig;
    }

    // backlight
    pinMode(BACKLIGHT, OUTPUT);
    digitalWrite(BACKLIGHT, eemem.backlight);

    setup_pwm();
}


void setup_pwm()
{
    pwm_handle.Instance = TIM1;
    pwm_handle.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    pwm_handle.hdma[0] = NULL;
    pwm_handle.hdma[1] = NULL;
    pwm_handle.hdma[2] = NULL;
    pwm_handle.hdma[3] = NULL;
    pwm_handle.hdma[4] = NULL;
    pwm_handle.hdma[5] = NULL;
    pwm_handle.hdma[6] = NULL;
    pwm_handle.Lock = HAL_UNLOCKED;
    pwm_handle.State = HAL_TIM_STATE_RESET;

    /* Configure timer with some default values */
    pwm_handle.Init.Prescaler = 0;
    pwm_handle.Init.Period = 400;
    pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    TIM_HandleTypeDef *h = &pwm_handle;
    enableTimerClock(h);
    HAL_TIM_Base_Init(h);

    /* Configure some default values. Maybe overwritten later */
    TIM_OC_InitTypeDef channelOC;
    channelOC.OCMode = TIMER_NOT_USED;
    channelOC.Pulse = __HAL_TIM_GET_COMPARE(h, TIM_CHANNEL_1);
    channelOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    channelOC.OCFastMode = TIM_OCFAST_DISABLE;
    channelOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    channelOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    channelOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    channelOC.OCMode = TIM_OCMODE_PWM1;

    HAL_TIM_PWM_ConfigChannel(h, &channelOC, TIM_CHANNEL_1);

    /* Set the Break feature & Dead time */
    TIM_BreakDeadTimeConfigTypeDef sBreakConfig;
    sBreakConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakConfig.DeadTime         = deadtime;
    sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakConfig.LockLevel        = 0;//TIM_LOCKLEVEL_1;  
    sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;  // does matter?
    sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
  
    HAL_TIMEx_ConfigBreakDeadTime(h, &sBreakConfig);
    LL_TIM_SetPrescaler(TIM1, 0);
}

void pwm_on()
{
    // enable 12v power
    pinMode(DRIVER_POWER, OUTPUT);
    digitalWrite(DRIVER_POWER, HIGH);
    delay(10);
  
    // ensure bootstrap capacitor is charged by manually setting low side mosfet on
    digitalWrite(PB13, LOW);
    digitalWrite(PA8, LOW);
    pinMode(PB13, OUTPUT);
    pinMode(PA8, OUTPUT);
    digitalWrite(PB13, HIGH);
    digitalWrite(PB13, LOW);

    delay(1);
    delay(1);
    
    // begin pwm
    pwm_max = 8000000 / get_frequency();

    min_duty = .5;
    pwm_min_comp = deadtime*5/2;
    max_duty = 1 - (float)pwm_min_comp/pwm_max;

    __HAL_TIM_SET_AUTORELOAD(&pwm_handle, pwm_max);
    __HAL_TIM_SET_COMPARE(&pwm_handle, TIM_CHANNEL_1, 35);

    pinmap_pinout(digitalPinToPinName(PA8), PinMap_PWM); 
    HAL_TIM_PWM_Start(&pwm_handle, TIM_CHANNEL_1);
    for(int i=10; i<90; i++) {
        __HAL_TIM_SET_COMPARE(&pwm_handle, TIM_CHANNEL_1, i*4);
        delay(1);
    }     

  duty = 1;
  pwm_set();
    // enable pwm (with dead time) to drive low-side mosfet now
    pinmap_pinout(digitalPinToPinName(PB13), PinMap_PWM);
    HAL_TIMEx_PWMN_Start(&pwm_handle, TIM_CHANNEL_1);
}

void pwm_set()
{
    int compare = pwm_max*(1.0-duty);

    if(compare < pwm_min_comp)
        compare = pwm_min_comp;   // minimum allowed

    if(compare > pwm_max/2)
        compare = pwm_max;  // maximum allowed

    __HAL_TIM_SET_COMPARE(&pwm_handle, TIM_CHANNEL_1, pwm_max-compare);
    //__HAL_TIM_SET_COMPARE(&pwm_handle, TIM_CHANNEL_1, pwm_max-65);
}

void pwm_off()
{

    digitalWrite(PB13, LOW);
    digitalWrite(PA8, LOW);
    pinMode(PB13, OUTPUT);
    pinMode(PA8, OUTPUT);
    HAL_TIM_PWM_Stop(&pwm_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&pwm_handle, TIM_CHANNEL_1);

    return;

    delay(10);
    // disable 12v power
    pinMode(DRIVER_POWER, OUTPUT);
    digitalWrite(DRIVER_POWER, LOW);
}

void update_duty() {
    // update duty cycle with pid filter on input voltage
    float err = (vin - mppt_voltage);
    float vindt = vin - lastvin;

    // D should be less than P
    float P = .01, D = 0;
    duty += P*err + D*vindt;
    if(duty > max_duty)
        duty = max_duty;

    // for now, avoid large voltage differences
    if(duty < min_duty)
        duty = min_duty;

    pwm_set();
}

void loop()
{
    int samples = 256;
    int channels[] = {PA0, PA1, PA2, PA3, PB0};
    int adc_n = (sizeof channels)/(sizeof *channels);
    vin = vout = iin = iout = 0;
    static float offin, offout;
    float offin_s = 0, offout_s = 0;

    for(int count = 0; count < samples; count++) {
        uint32_t reading[6];
        for(int i=0; i<adc_n; i++)
            reading[i]=analogRead(channels[i]);
      
        vin += reading[0]/4096.0 * 3.3 * 13;
        vout +=reading[1]/4096.0 * 3.3 * 13;
        int csref = reading[4];
        iin += ((float)csref-reading[2]-offin)*3.3/(4096*.001*200)   *1.2;
        iout += ((float)csref-reading[3]-offout)*3.3/(4096*.001*200) *1.2;

        if(!running) {
            offin_s+=(float)csref-reading[2];
            offout_s+=(float)csref-reading[3];
        }

        if((count & 0x1f) == 0)
            read_keys(); // read keys a bit more often
    }

    vin /= samples;
    vout /= samples;
    iin /= samples;
    iout /= samples;

    if(!running) {
        offin = offin_s/samples;
        offout = offout_s/samples;
    }

    pin = vin*iin;
    pout = vout*iout;

    // read temp less frequently, internal channels take longer also
    uint16_t treading = analogRead(ATEMP);
    // convert to degrees C
    float rtemp = ((1.43-treading/4096.0*3.3)/0.0043) + 25;
    temp = .9*temp + .1*rtemp; //lowpass

    if(running) {
        uint32_t t = HAL_GetTick();
        if(t-on_time > 500) { // don't stop or adjust duty unless running at least 500 ticks
            // overtemp, current has gone negative, or output voltage too high: turn off
            if(temp > max_temp || iout < -.05 || vout > eemem.off_voltage || pwm_max != 8000000 / get_frequency()) {
                pwm_off();
                off_time = t;
                running = false;
                digitalWrite(LED, HIGH); // off
            } else {
                update_duty();
                static float trackingdir = .15, avgpout;
                static int trackingcount = 0;
                if(fabs(vin-mppt_voltage) < .05 || duty == max_duty || duty == min_duty) {
                    trackingcount++;
                    avgpout += pout;
                }
                if(trackingcount == 5) {
                    avgpout /= trackingcount;
                    debug = max_duty;
                    trackingcount = 0;
                    if(lastpout > avgpout) // change dir
                        trackingdir = -trackingdir;
                    if(duty == max_duty)
                        trackingdir = fabs(trackingdir);
                    else if(duty == min_duty)
                        trackingdir = -fabs(trackingdir);
                    lastpout = avgpout;
                    mppt_voltage += trackingdir;
                    avgpout = 0;
                }
            }
        }
    } else { // not running
        if(temp+1 < max_temp && vin > vout+1 && vout < eemem.on_voltage) {
            uint32_t t = HAL_GetTick();
            if(t-off_time > 3000) { // 10 seconds
                on_time = t;
                pwm_on();
                running = true;
                digitalWrite(LED, LOW); // on
            }
        }
    }

    lastvin = vin;
 
    display.clearDisplay();
    display.setRotation(1);
    switch(page) {
    case 0: display_default(); break;
    case 1: display_info(); break;
    case 2: display_set_off_voltage(); break;
    case 3: display_set_on_voltage(); break;
    case 4: display_set_backlight(); break;
    case 5: display_set_frequency(); break;
    }

    if(temp > max_temp) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0,0);
        display.println("WARNING");
        display.println("OVERTEMP");
    }
  
    display.display();
}
