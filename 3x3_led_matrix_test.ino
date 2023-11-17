/*
* Program name: 3x3 RGB LED Matrix Test
* Author: Jasper Favis
* Date: 2023-11-16
* Purpose: Create LED Pattern logic

* Hardware:
  * SN74LS128 3-to-8 Decoder
  * PCA9685 16-channel 12-bit PWM LED Driver
  * Arduino Uno Rev3 (ATmega328 Microcontroller)

* Notes:
  * Timer interrupt used to multiplex rows.
  * PCA9685 library used to set PWM outputs on the PCA9685 chip.
*/

#include <Arduino.h>
#include "PCA9685.h"

#define CHANNEL_COUNT  16
#define LED_BRIGHTNESS_RESOLUTION 2048 
#define MULTIPLEX_FREQ 1000 // hz
#define PRESCALER      256  // (clk / 156)
#define SHIFT8  4
#define SHIFT9  3
#define SHIFT10 2
#define SHIFT11 1

uint16_t pwms[CHANNEL_COUNT];
uint16_t rgb[] = {2047,0,0}; // red: fully on, green: off, blue: off
uint16_t count = 0;
uint8_t dec = 0; // dec and inc point to the current color pair in rgb[]
uint8_t inc = 1;
uint8_t row = 0; // address input for SN74LS138
bool interruptTriggered = false;

PCA9685 pwmController;

void setup() {
    /* I2C initialization */
    Wire.begin();

    /* PCA9685 initialization */
    pwmController.resetDevices();
    pwmController.init(
      PCA9685_OutputDriverMode_TotemPole, 
      PCA9685_OutputEnabledMode_Normal, 
      PCA9685_OutputDisabledMode_Low, 
      PCA9685_ChannelUpdateMode_AfterAck, 
      PCA9685_PhaseBalancer_None);
    pwmController.setPWMFrequency(1000);

    /* Port initialization */
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4); // 3 address select pins on SN74LS138

    /* Timer initialization */
    cli();  // disable interrupts
    timer0Setup(MULTIPLEX_FREQ, PRESCALER); // (important!) You also need to hardcode prescaler value inside timerXSetup.
    sei();  // enable interrupts
}

void loop() {
    if(interruptTriggered) {
        interruptTriggered = false;

        /* This is where we multiplex the rows. */
        turnOnRow(row);
        row = (row + 1) % 3;

        /* Crossfade LED pair */
        rgb[dec] -= 1;
        rgb[inc] += 1;
        count++;

        /* Convert 11-bit to 12-bit color values (0-2047 --> 0-4095).
           Then set channels to rgb values. */
        uint16_t r = rgb[0] << SHIFT11;
        uint16_t g = rgb[1] << SHIFT11;
        uint16_t b = rgb[2] << SHIFT11;
        setChannelsRGB(r,g,b,pwms);

        /* Move to the next LED pair when crossfade is complete. */
        if(count >= LED_BRIGHTNESS_RESOLUTION) {
            inc = (inc + 1) % 3;
            dec = (dec + 1) % 3;
            count = 0;
        }
    }
}

/*  Prescaler select bits for Timer0
    CS02 | CSO1 | CS00 | Description
    0     0     0     No clock source (Timer/Counter Stopped)
    0     0     1     No prescaling
    0     1     0     clk / 8
    0     1     1     clk / 64
    1     0     0     clk / 256
    1     0     1     clk / 1024
    1     1     0     Ext clk pin T0 (Falling Edge)
    1     1     1     Ext clk pin T0 (Rising Edge)
*/
void timer0Setup(uint32_t interruptFreq, uint32_t prescaler) {
    TCCR0A  = 0;                                         // set entire TCCR0A register to 0
    TCCR0B  = 0;                                         // same for TCCR0B
    TCNT0   = 0;                                         // initialize counter value to 0
    OCR0A   = (16*10^6) / (interruptFreq*prescaler) - 1; // set compare match register. (important!: must be <256 for 8-bt Timer0)
    TCCR0A |= (1 << WGM01);                              // turn on CTC mode
    TCCR0B |= (1 << CS02);                               // 256 prescaler
    TIMSK0 |= (1 << OCIE0A);                             // enable timer compare interrupt
}

/*  Prescaler select bits for Timer1
    CS12 | CS11 | CS10 | Description
    0     0     0     No clock source (Timer/Counter Stopped)
    0     0     1     No prescaling
    0     1     0     clk / 8
    0     1     1     clk / 64
    1     0     0     clk / 256
    1     0     1     clk / 1024
    1     1     0     Ext clk pin T1 (Falling Edge)
    1     1     1     Ext clk pin T1 (Rising Edge)
*/
void timer1Setup(uint32_t interruptFreq, uint32_t prescaler) {
    TCCR1A  = 0;                                         // set entire TCCR1A register to 0
    TCCR1B  = 0;                                         // same for TCCR1B
    TCNT1   = 0;                                         // initialize counter value to 0
    OCR1A   = (16*10^6) / (interruptFreq*prescaler) - 1; // set compare match register. (important!: must be <65536 for 16-bit Timer1)
    TCCR1B |= (1 << WGM12);                              // turn on CTC mode
    TCCR1B |= (1 << CS12);                               // 256 prescaler
    TIMSK1 |= (1 << OCIE1A);                             // enable timer compare interrupt
}

/*  Prescaler select bits for Timer2 
    CS22 | CS21 | CS20 | Description
    0     0     0     No clock source (Timer/Counter Stopped)
    0     0     1     No prescaling
    0     1     0     clk / 8
    0     1     1     clk / 32
    1     0     0     clk / 64
*/
void timer2Setup(uint32_t interruptFreq, uint32_t prescaler) {
    TCCR2A  = 0;                                         // set entire TCCR2A register to 0
    TCCR2B  = 0;                                         // same for TCCR2B
    TCNT2   = 0;                                         // initialize counter value to 0
    OCR2A   = (16*10^6) / (interruptFreq*prescaler) - 1; // set compare match register. (important!: must be <256 for 8-bit Timer2)
    TCCR2A |= (1 << WGM21);                              // turn on CTC mode
    TCCR2B |= (1 << CS22);                               // 64 prescaler
    TIMSK2 |= (1 << OCIE2A);                             // enable timer compare interrupt
}

/* Interrupt service routine sets flag which is handled in the main loop. */
ISR(TIMER0_COMPA_vect) {
    interruptTriggered = true;
}

/* Pins 2, 3, and 4 set the 3-bit address. */
void turnOnRow(uint8_t rowAddr) {
    PORTD = rowAddr << PD2;
}

/*  The PWM values are set according to the following arrangement:
    Channel: 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15
    Cathode: r  g  b  r  g  b  r  g  b  r  g  b  r  g  b  r
 */
void setChannelsRGB(uint16_t r, uint16_t g, uint16_t b, uint16_t pwms[]) {
    for(int i = 0; i < CHANNEL_COUNT; i++){
        if(i % 3 == 0) {
            pwms[i] = r;
        } else if ((i - 1) % 3 == 0) {
            pwms[i] = g;
        } else {
            pwms[i] = b;
        }
    }
    pwmController.setChannelsPWM(0, CHANNEL_COUNT, pwms);
}