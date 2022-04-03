# DCC Accessory Decoder for single or double light Italian railway signals based on Arduino and Adafruit TLC5947
![License](https://img.shields.io/badge/License-MIT-green)

*Versione in Italiano [qui](README.md)*

## Table of contents
- [Introduction](#introduction)
- [What is needed](#what-is-needed)
- [Connections](#connections)
- [Configuration](#configuration)
- [Table of CVs (configuration variables) used](#table-of-cvs-configuration-variables-used)
- [Some specifics on how the sketch works](#some-specifics-on-how-the-sketch-works)
- [Version and Contributors](#version-and-contributors)

## Introduction

This sketch for [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) implements a complete accessory-type DCC decoder to drive LEDs (normal or RGB) to be used in Italian railway signals with 1 or 2 lights

![Overview](/images/DCC_ItalianSignals_Overview.jpg)

## What is needed

To create a complete Italian railway signals DCC accessory-decoder, you need:
* a board [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) or [compatible](https://google.com/search?q=geekcreit+arduino+uno)
* an interface [Adafruit TLC5947](https://www.adafruit.com/product/1429) (demo available [here](https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout)) capable of driving up to 24 LEDs; TLC5947 interface allows you to work with multiple cascaded interfaces following the instructions at this [link](https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout/connecting-to-the-arduino#chaining-boards-2151225-14)
* a shield or interface that is able to "read" the DCC signal through Arduino, I highly recommend [shield or interface](https://github.com/lucadentella/arduino-dccshield) of my friend Luca Dentella

## Connections

* Arduino Uno or compatible interface
    - use the USB cable (useful if you want to monitor the traffic on the serial monitor window) or the appropriate 5V power plug
* DCC interface / shield
    - use the appropriate DCC terminal block of the interface / shield to which you will connect the signal coming from your control unit, the DCC signal works in AC mode and therefore you can connect both one way and the other
    - VCC must be connected to one of the dedicated 5V Arduino pins (voltage)
    - GND must be connected to one of the dedicated GND Arduino pins (ground)
    - DCC must be connected to the pin dedicated to the data input with interrupt, pin declared for this sketch is 2
    - ACK must be connected to the pin dedicated to the ACK signal, the pin declared for this sketch is 3
* TLC5947
    - V+  must be connected to one of the dedicated 5V Arduino pins (voltage)
    - GND must be connected to one of the dedicated GND Arduino pins (ground)
    - DIN must be connected to pin D4
    - CLK must be connected to pin D5
    - LAT must be connected to pin D6
> **If you decide to drive a fairly large number of LEDs, you can use an external power supply by disconnecting V+ pin from Arduino and connecting it to an external power supply by sharing the ground connections on GND pins.**
> **All instructions for using the external power supply are available at this [link](https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout/power-and-leds#connecting-an-external-supply-940507-7)!**

## Configuration

Sketch includes this configuration:
- definition of the number of LEDs to drive, the default value is 24 the maximum allowed by the TLC5947 interface
- 2 CVs specifications to memorize configuration of 8 possible lights that can be used with 24 LEDs (each light will therefore have 3 outputs used respectively for the red signal, the yellow signal and the green signal)

## Table of CVs (configuration variables) used

| CV | Description | Notes | Default value |
|: - |: - |: - | -: |
| 33 | first 4 pairs of 2 bits for the configuration of the lights | | 9 |
| 34 | second 4 pairs of 2 bits for the configuration of the lights | | 0 |

> bits of each CV are structured as follows: **b7-b6 b5-b4 b3-b2 b1-b0** and each pair of bits has the following meaning:
> **00** = **nothing**
> **01** = **single light**
> **10** = **double light**
> **11** = **not used** (whoever wants to can dedicate this configuration for 3-light signals by modifying the sketch appropriately)

> Default value set with the reset command sent to the decoder (any value on CV8) sets the **CV33** to **9** and the **CV34** to **0**
> the meaning is therefore the following:
> **9** => **00 00 10 01** (starting from the right: first configuration 1 light, second configuration 2 lights, third and fourth not managed)
> **0** => **00 00 00 00** (no lights configured)
>
> If, for example, we wanted to configure the decoder to drive 4 single lights, it is necessary to set the value **85** on **CV33** (corresponding to the sequence of bits equal to: **01 01 01 01**)
>
> **TIP**: I recommend this [link](https://www.binaryhexconverter.com/binary-to-decimal-converter) to quickly calculate  desired values

## Some specifics on how the sketch works

***
Enumeration containing the possible status of the single LED:
- **OFF** led off
- **ON** led on
- **NORMAL_BLINKING** indicates one flash every second
- **REVERSED_BLINKING** indicates one flash every second with on / off opposite to normal flashing (currently used only in double sail signals with yellow and green flashing alternately)
```c
enum LedState {
    OFF,
    ON,
    NORMAL_BLINKING,
    REVERSED_BLINKING
};

LedState led[NUM_LEDS]; 
```
***
Definition of the operating structure of a single light, to drive the 4 possible states of the light, a total of 2 DCC addresses are needed, whose on / off combinations determine the display mode (fixed red, fixed green, fixed yellow, flashing yellow)
```c
typedef struct SingleLight {
    bool dccAddress1State;
    bool dccAddress2State;
    LedState redLight;
    LedState yellowLight;
    LedState greenLight;
};

#define SINGLE_LIGHT_NUM_STATES 4

SingleLight singleLight[SINGLE_LIGHT_NUM_STATES] = {
    {false, false, ON,  OFF, OFF},             // Fixed red
    {false, true,  OFF, OFF, ON},              // Fixed green
    {true,  false, OFF, ON,  OFF},             // Fixed yellow
    {true,  true,  OFF, NORMAL_BLINKING, OFF}, // Blinking yellow
};
```
***
Definition of the operating structure of a double light, to drive the possible 11 states of the 2 lights, a total of 4 DCC addresses are needed, whose on / off combinations determine the display mode specified in the comments below
```c
typedef struct DoubleLight {
    bool dccAddress1State;
    bool dccAddress2State;
    bool dccAddress3State;
    bool dccAddress4State;
    LedState redLightOne;
    LedState yellowLightOne;
    LedState greenLightOne;
    LedState redLightTwo;
    LedState yellowLightTwo;
    LedState greenLightTwo;
};

#define DOUBLE_LIGHT_NUM_STATES 11

DoubleLight doubleLight[DOUBLE_LIGHT_NUM_STATES] = {
    {false, false, false, false, ON,  OFF, OFF,             OFF, OFF, OFF},               // Fixed red
    {false, true,  false, false, OFF, OFF, ON,              OFF, OFF, OFF},               // Fixed green
    {true,  false, false, false, OFF, ON,  OFF,             OFF, OFF, OFF},               // Fixed yellow
    {true,  true,  false, false, OFF, NORMAL_BLINKING,      OFF, OFF, OFF, OFF},          // Blinking yellow
    {false, false, true,  false, ON,  OFF, OFF,             OFF, ON,  OFF},               // Fixed red, fixed yellow
    {false, true,  true,  false, ON,  OFF, OFF,             OFF, NORMAL_BLINKING, OFF},   // Fixed red, blinking yellow
    {true,  false, true,  false, ON,  OFF, OFF,             OFF, OFF, ON},                // Fixed red, fixed green
    {true,  true,  true,  false, OFF, ON,  OFF,             OFF, ON,  OFF},               // Fixed yellow, fixed yellow
    {false, false, false, true,  OFF, ON,  OFF,             OFF, OFF, ON},                // Fixed yellow, fixed green
    {false, true,  false, true,  OFF, NORMAL_BLINKING, OFF, OFF, OFF, NORMAL_BLINKING},   // Blinking yellow and green at same time
    {true,  false, false, true,  OFF, NORMAL_BLINKING, OFF, OFF, OFF, REVERSED_BLINKING}, // Blinking yellow and green alternatively
};
```
***
Structure definition containing current status of each single light (minimum 4 if all double lights, maximum 8 if all single light) and the corresponding on / off value of the corresponding DCC addresses
```c
enum LightType {
    NONE,
    SINGLE,
    DOUBLE,
    NOT_USED
};

typedef struct Light {
    LightType type;
    int dccAddressOffset;
    bool dccAddress1State;
    bool dccAddress2State;
    bool dccAddress3State;
    bool dccAddress4State;
};

Light light[NUM_LEDS / 3] = {
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
    {NONE, 0, false, false, false, false},
};
```
***
In the initial setup, when the Arduino is turned on, a loop is performed to turn off all the LEDs and then initialize each light to its first set state (usually fixed red)
```c
void setup() {
    ...
    for (int idx = 0; idx < NUM_LEDS; idx++) {
        tlc.setPWM(idx, 0);
    }
    tlc.write();
    delay(50);
    ...
    loadLightsConfiguration();
    setLedTable();
    ...
}
```
***
Each loop cycle of the sketch, number of milliseconds from power on is counted to obtain a virtual counter from 1 to 1000 to indicate the flashing time of 1 second
```c
void loop() {
    unsigned long blinkingCounter;
    ...
    // Set blinking counter (range 0 to 1000 milliseconds)
    blinkingCounter = millis();
    blinkingCounter = blinkingCounter % 1000;
    ...
}
```
***
Execution of the loop on each LED to control its behavior (off, on, normal or reverse flashing)
```c
void loop() {
    ...
    // Loop on every led to set PWM state (0 => off, 1000 => on)
    for (int idx = 0; idx < NUM_LEDS; idx++) {
        switch (led[idx]) {
            case OFF:
                tlc.setPWM(idx, 0);
                break;
            case ON:
                tlc.setPWM(idx, 1000);
                break;
            case NORMAL_BLINKING:
                tlc.setPWM(idx, (blinkingCounter > 500 ? 1000 : 0));
                break;
            case REVERSED_BLINKING:
                tlc.setPWM(idx, (blinkingCounter < 500 ? 1000 : 0));
                break;
            default:
                break;
        };
    };

    // Send write to TLC and wait some milliseconds
    tlc.write();
    delay(50);
    ...
}
```
***
Function that loads configuration of the lights, used both when the Arduino is turned on and at each change of CVs 33 and 34.
The following operations are performed in sequence:
* initialized the **numDCCUsedAddresses** variable to calculate number of DCC addresses used
* read the configuration 2 bits at a time through the instruction **cvValue & 3**
* initialized structure of the LEDs depending on whether it is a single light or a double light (**SINGLE_LIGHT_CONFIGURATION** or **DOUBLE_LIGHT_CONFIGURATION**)
* shifting of 2 bits to the right for each cycle with the instruction **cvValue = cvValue >> 2**
```c
void loadLightsConfiguration() {
    int min, max, lightCounter, cvValue;
    ...
    // Reset number of DCC used addresses
    numDCCUsedAddresses = 0;
    ...
        int bitValue = cvValue & 3;
    ...
            case SINGLE_LIGHT_CONFIGURATION: {
                if (lightCounter < (NUM_LEDS / 3)) {
                    light[idx].type = SINGLE;
                    light[idx].dccAddressOffset = numDCCUsedAddresses;
                    light[idx].dccAddress1State = false;
                    light[idx].dccAddress2State = false;
                    light[idx].dccAddress3State = false;
                    light[idx].dccAddress4State = false;
                    lightCounter++;
                    numDCCUsedAddresses += 2;
                }
                break;
            }
    ...
            case DOUBLE_LIGHT_CONFIGURATION: {
                if (lightCounter < ((NUM_LEDS / 3) - 1)) {
                    light[idx].type = DOUBLE;
                    light[idx].dccAddressOffset = numDCCUsedAddresses;
                    light[idx].dccAddress1State = false;
                    light[idx].dccAddress2State = false;
                    light[idx].dccAddress3State = false;
                    light[idx].dccAddress4State = false;
                    lightCounter += 2;
                    numDCCUsedAddresses += 4;
                }
                break;
            }
    ...
        cvValue = cvValue >> 2;
    ...
}
```
***
Function that performs setup of each single LED according to the configuration of the lights, invoked at each configuration change or at each new DCC command received addressed to the managed lights. Loop is performed on the entire group of lights, checking whether it is a single light or a double light and setting the value of each individual LED according to the corresponding table. If there is an unexpected combination among those surveyed, all the corresponding LEDs are turned off
```c
void setLedTable() {
    ...
        if (light[idxLight].type == SINGLE) {
            lightProcessed = false;
            for (int idx = 0; idx < SINGLE_LIGHT_NUM_STATES; idx++) {
                if (light[idxLight].dccAddress1State == singleLight[idx].dccAddress1State &&
                    light[idxLight].dccAddress2State == singleLight[idx].dccAddress2State) {
    ...
        if (light[idxLight].type == DOUBLE) {
            lightProcessed = false;
            for (int idx = 0; idx < DOUBLE_LIGHT_NUM_STATES; idx++) {
                if (light[idxLight].dccAddress1State == doubleLight[idx].dccAddress1State &&
                    light[idxLight].dccAddress2State == doubleLight[idx].dccAddress2State &&
                    light[idxLight].dccAddress3State == doubleLight[idx].dccAddress3State &&
                    light[idxLight].dccAddress4State == doubleLight[idx].dccAddress4State) {
    ...
            if (!lightProcessed) {
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
    ...
}
```
***
Callback function of change position of an exchange, it's verified if the command is directed to one of the lights managed by checking the main address range assigned to decoder and number of addresses resulting from the configuration of number of lights. It's also checked whether a change of state has occurred which determines the change in the display of the corresponding lights
```c
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {
    // Get decoder DCC address
    long dccAddress = Dcc.getAddr();

    // Check if DCC packets needs to be processed on one of the lights
    if (Addr >= dccAddress && Addr < dccAddress + numDCCUsedAddresses) {
    ...
                case SINGLE: {
                    if (light[idx].dccAddressOffset == offsetAddress) {
                        if (light[idx].dccAddress1State != value) {
                            light[idx].dccAddress1State = value;
                        }
                        processed = true;
                    } else if (light[idx].dccAddressOffset == (offsetAddress - 1)) {
    ...
                case DOUBLE: {
                    if (light[idx].dccAddressOffset == offsetAddress) {
                        if (light[idx].dccAddress1State != value) {
                            light[idx].dccAddress1State = value;
                        }
                        processed = true;
                    } else if (light[idx].dccAddressOffset == (offsetAddress - 1)) {
    ...

}
```
***

## Version and Contributors
Version: 1.0.0
Published date: April 2022
Author: [Giovanni Colasurdo](mailto:gio.colasurdo@gmail.com)