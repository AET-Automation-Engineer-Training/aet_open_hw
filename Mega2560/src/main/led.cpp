/**
 ******************************************************************************
 * @file    led.c
 * @author  parkhoon1609
 * @version V1.0.0
 * @date    09-01-2024
 * @brief
 ******************************************************************************
 */

#include "led.h"
#include <Arduino.h>

uint8_t j_in_state = 0;


Adafruit_NeoPixel pixels(NUMPIXELS, LED_RGB_PIN, NEO_GRB + NEO_KHZ800);

void main_led(void)
{
    Init_led();
    // while(1)
    // {
        // Normal_state();
    // }
}

void Init_led(void)
{
    pixels.begin();
    pixels.setBrightness(BRIGHTNESS);
}

void Warning_state() // Nhay do
{
    for (int i = 0; i < NUMPIXELS; i++)
    {
        pixels.setPixelColor(i, RGB_RED);
    }
    delay(200);
    pixels.show();

    pixels.clear();
    delay(200);
    pixels.show();
}

void Full_state() // Nhay do
{
    for (int i = 0; i < NUMPIXELS; i++)
    {
        pixels.setPixelColor(i, RGB_GREEN);
    }
    delay(200);
    pixels.show();

    pixels.clear();
    delay(200);
    pixels.show();
}

void Normal_state()
{
    uint16_t i;
    for (i = 0; i < pixels.numPixels(); i++)
    {
        pixels.setPixelColor(i, Wheel(((i * 256 / NUMPIXELS) + j_in_state) & 255));
    }
    delay(200);
    pixels.show();

    pixels.clear();
    delay(200);
    pixels.show();
    j_in_state = (j_in_state + 1) % (256 * 5);
}

uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
