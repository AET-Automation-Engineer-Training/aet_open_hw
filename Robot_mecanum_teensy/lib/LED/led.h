/**
 ******************************************************************************
 * @file    led.h
 * @author  parkhoon1609
 * @version V1.0.0
 * @date    09-01-2024
 * @brief
 ******************************************************************************
 */

#ifndef _LED_H_
#define _LED_H_

#include <Adafruit_NeoPixel.h>
#include "TeensyThreads.h"

// #define LIGHT_PIN 33
#define LED_RGB_PIN 19

#define NUMPIXELS 20
#define BRIGHTNESS 50

#define RGB_RED pixels.Color(255, 0, 0)
#define RGB_GREEN pixels.Color(0, 255, 0)
#define RGB_BLUE pixels.Color(0, 0, 255)
#define RGB_WHITE pixels.Color(127, 127, 127)
#define RGB_YELLOW pixels.Color(255, 255, 0)

void main_led(void);
void Init_led(void);
void Warning_state();
void Full_state();
void Normal_state();
uint32_t Wheel(byte WheelPos);

#endif