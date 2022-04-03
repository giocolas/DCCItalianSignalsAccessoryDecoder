/*
 * DCC Italian signals accessory decoder based on:
 *      - Arduino Uno or other Arduino compatible board
 *      - DCC Shield by Luca Dentella (see https://github.com/lucadentella/arduino-dccshield)
 *      - Adafruit TLC5947 or other compatible interfaces (see https://www.adafruit.com/product/1429)
 * 
 * Release: 1.0.0
 * 
 * Author: Giovanni Colasurdo
 * Date: April 2022
 * 
 */

/*
 * Please comment this line if you don't want to debug this sketch on the serial console
 */
#define DEBUG 1

/*
 * Include referenced libraries
 */
#include <NmraDcc.h>
#include "Adafruit_TLC5947.h"

/*
 * NMRA DCC library
 */
NmraDcc Dcc;

/*
 * Board pin used to get DCC packets by DCC shield
 * Board pin used to send DCC ACK signal
 */
#define DCC_PIN 2
#define ACK_PIN 3

/*
 * Define board parameters
 */
#define NUM_TLC5974 1
#define PIN_DATA    4
#define PIN_CLOCK   5
#define PIN_LATCH   6
#define PIN_OE      -1  // set to -1 to not use the enable pin (its optional)
#define NUM_LEDS    24

/*
 * Adafruit TLC5947 Led driver
 */
Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5974, PIN_CLOCK, PIN_DATA, PIN_LATCH);

/*
 * Define enum led state
 */
enum LedState {
    OFF,
    ON,
    NORMAL_BLINKING,
    REVERSED_BLINKING
};

LedState led[NUM_LEDS]; 

/*
 * Single light table definition (total of 4 aspects, 2 DCC addresses used):
 * 
 *    board address pins     => red led
 *    board address pins + 1 => yellow led
 *    board address pins + 2 => green led
 *    
 */
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

/*
 * Double light table definition (total of 11 aspects, 4 DCC addresses used):
 * 
 *    board address pins     => red led one
 *    board address pins + 1 => yellow led one
 *    board address pins + 2 => green led one
 *    board address pins + 3 => red led two
 *    board address pins + 4 => yellow led two
 *    board address pins + 5 => green led two
 */
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

/*
 * Connected lights
 */
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

/*
 * Number of DCC used addresses (calculated on single & double lights configuration)
 */
int numDCCUsedAddresses = 0;

/*
 * Lights CVs table:
 *      - 33 & 34 => lights configuration: block of 2 * 8 bits
 * 
 *        CV 33      bit 7 / bit 6    bit 5 / bit 4    bit 3 / bit 2    bit 1 / bit 0
 *                   ================================================================
 *                    Light n. 4       Light n. 3       Light n. 2       Light n. 1
 * 
 *        CV 34      bit 7 / bit 6    bit 5 / bit 4    bit 3 / bit 2    bit 1 / bit 0
 *                   ================================================================
 *                    Light n. 8       Light n. 7       Light n. 6       Light n. 5
 * 
 *         bits specification detail:       bit 1      bit 0
 *                                         ==================
 *                                            0          0     => none
 *                                            0          1     => single light
 *                                            1          0     => double light
 *                                            1          1     => not used
 * 
 *        Default values: CV 33 => 00 00 10 01 (light n. 1 single & light n. 2 double) => 9
 *                        CV 34 => 00 00 00 00 (none lights selected) => 0
 * 
 */
#define CONFIG_LIGHTS_MIN_CV_ADDR  33
#define CONFIG_LIGHTS_MAX_CV_ADDR  34

#define DEFAULT_MIN_CV_VALUE       9
#define DEFAULT_MAX_CV_VALUE       0

#define NONE_CONFIGURATION         0  // bit values: 0 0
#define SINGLE_LIGHT_CONFIGURATION 1  // bit values: 0 1
#define DOUBLE_LIGHT_CONFIGURATION 2  // bit values: 1 0
#define NOT_USED_CONFIGURATION     3  // bit values: 1 1

/*
 * Special sketch states (used to optimize factory reset servo movements)
 */
bool resettingState = false;

/*
 * Initial setup of board
 */
void setup() {

    // Set serial baud rate
#ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Starting ...");
#endif

    // Init NmraDcc library & set ACK pin to output
    Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, false);
    Dcc.init(MAN_ID_DIY, 1, FLAGS_DCC_ACCESSORY_DECODER | FLAGS_OUTPUT_ADDRESS_MODE, 0);
    pinMode(ACK_PIN, OUTPUT);
#ifdef DEBUG
    Serial.print("\tNmraDcc library initialized, DCC Pin: "); Serial.print(DCC_PIN); Serial.print(", ACK Pin: "); Serial.print(ACK_PIN); Serial.println();
#endif

    // Init Adafruit TLC5947 interface, 0E pin actually not used!
    tlc.begin();
    if (PIN_OE >= 0) {
        pinMode(PIN_OE, OUTPUT);
        digitalWrite(PIN_OE, LOW);
    }

    // Init all led values to off mode
    for (int idx = 0; idx < NUM_LEDS; idx++) {
        tlc.setPWM(idx, 0);
    }
    tlc.write();
    delay(50);

    // Load lights configuration & set Led state
    loadLightsConfiguration();
    setLedTable();

}

/*
 * Board main loop
 */
void loop() {
    unsigned long blinkingCounter;

    // Process DCC incoming packets
    Dcc.process();

    // Set blinking counter (range 0 to 1000 milliseconds)
    blinkingCounter = millis();
    blinkingCounter = blinkingCounter % 1000;

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
}

/*
 * Load lights configuration by reading CV values
 */
void loadLightsConfiguration() {
    int min, max, lightCounter, cvValue;

    // Get min & max values
    min = Dcc.getCV(CONFIG_LIGHTS_MIN_CV_ADDR);
    max = Dcc.getCV(CONFIG_LIGHTS_MAX_CV_ADDR);
#ifdef DEBUG
    Serial.print("Load Light Configuration, min CV value: "); Serial.print(min, DEC); Serial.print(" , max CV value: "); Serial.println(max, DEC);
#endif

    // Reset number of DCC used addresses
    numDCCUsedAddresses = 0;

    // Loop on min & max values to determining lights configuration
    lightCounter = 0;
    cvValue = min;
    for (int idx = 0; idx < 4; idx++) {
        int bitValue = cvValue & 3;
        switch (bitValue) {
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
            default:
                break;
        }
        cvValue = cvValue >> 2;
    }
    cvValue = max;
    for (int idx = 0; idx < 4; idx++) {
        int bitValue = cvValue & 3;
        switch (bitValue) {
            case SINGLE_LIGHT_CONFIGURATION: {
                if (lightCounter < (NUM_LEDS / 3)) {
                    light[idx + 4].type = SINGLE;
                    light[idx + 4].dccAddressOffset = numDCCUsedAddresses;
                    light[idx + 4].dccAddress1State = false;
                    light[idx + 4].dccAddress2State = false;
                    light[idx + 4].dccAddress3State = false;
                    light[idx + 4].dccAddress4State = false;
                    numDCCUsedAddresses += 2;
                    lightCounter++;
                }
                break;
            }
            case DOUBLE_LIGHT_CONFIGURATION: {
                if (lightCounter < ((NUM_LEDS / 3) - 1)) {
                    light[idx + 4].type = DOUBLE;
                    light[idx + 4].dccAddressOffset = numDCCUsedAddresses;
                    light[idx + 4].dccAddress1State = false;
                    light[idx + 4].dccAddress2State = false;
                    light[idx + 4].dccAddress3State = false;
                    light[idx + 4].dccAddress4State = false;
                    numDCCUsedAddresses += 4;
                    lightCounter += 2;
                }
                break;
            }
            default:
                break;
        }
        cvValue = cvValue >> 2;
    }
}

/*
 * Calculate led state table based on lights configuration
 */
void setLedTable() {

    int idxLight = 0, idxLed = 0;
    bool lightProcessed;

    // Loop on lights grouping by 3 leds every time (total of 8 signals)
    while (idxLight < (NUM_LEDS / 3) && idxLed < NUM_LEDS) {

        // Single light process routine
        if (light[idxLight].type == SINGLE) {
            lightProcessed = false;
            for (int idx = 0; idx < SINGLE_LIGHT_NUM_STATES; idx++) {
#ifdef DEBUG
                Serial.print("Process single light n.: " ); Serial.print(idxLight); Serial.print(", state n.: " ); Serial.println(idx);
#endif
                if (light[idxLight].dccAddress1State == singleLight[idx].dccAddress1State &&
                    light[idxLight].dccAddress2State == singleLight[idx].dccAddress2State) {

                        led[idxLed] = singleLight[idx].redLight;
                        idxLed++;

                        led[idxLed] = singleLight[idx].yellowLight;
                        idxLed++;

                        led[idxLed] = singleLight[idx].greenLight;
                        idxLed++;

                        lightProcessed = true;

                        break;
                }
            };

            if (!lightProcessed) {
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
            }
        }

        // Double light process routine
        if (light[idxLight].type == DOUBLE) {
            lightProcessed = false;
            for (int idx = 0; idx < DOUBLE_LIGHT_NUM_STATES; idx++) {
#ifdef DEBUG
                Serial.print("Process double light n.: " ); Serial.print(idxLight); Serial.print(", state n.: " ); Serial.println(idx);
#endif
                if (light[idxLight].dccAddress1State == doubleLight[idx].dccAddress1State &&
                    light[idxLight].dccAddress2State == doubleLight[idx].dccAddress2State &&
                    light[idxLight].dccAddress3State == doubleLight[idx].dccAddress3State &&
                    light[idxLight].dccAddress4State == doubleLight[idx].dccAddress4State) {

                        led[idxLed] = doubleLight[idx].redLightOne;
                        idxLed++;

                        led[idxLed] = doubleLight[idx].yellowLightOne;
                        idxLed++;

                        led[idxLed] = doubleLight[idx].greenLightOne;
                        idxLed++;

                        if (idxLed < NUM_LEDS) {
                            led[idxLed] = doubleLight[idx].redLightTwo;
                            idxLed++;
  
                            led[idxLed] = doubleLight[idx].yellowLightTwo;
                            idxLed++;
  
                            led[idxLed] = doubleLight[idx].greenLightTwo;
                            idxLed++;
                        }
                        lightProcessed = true;

                        break;
                }
            };

            // All leds off if found an not valid combination 
            if (!lightProcessed) {
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
                led[idxLed] = OFF;
                idxLed++;
            }
        }

        idxLight++;
    };
}

/*
 * Callback function used to get turnout commands
 */
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower) {

    // Get decoder DCC address
    long dccAddress = Dcc.getAddr();

#ifdef DEBUG
    Serial.print("DCC command received, address: " ); Serial.print(Addr, DEC); Serial.print(", direction: "); Serial.println(Direction, DEC);
#endif

    // Check if DCC packets needs to be processed on one of the lights
    if (Addr >= dccAddress && Addr < dccAddress + numDCCUsedAddresses) {
        int offsetAddress = Addr - dccAddress;
        bool value = (Direction == 1 ? true : false);
        bool processed = false;

        // Loop on every light to determine if there is a changing state
        for (int idx = 0; idx < (NUM_LEDS / 3); idx++) {
            switch (light[idx].type) {
                case SINGLE: {
                    if (light[idx].dccAddressOffset == offsetAddress) {
                        if (light[idx].dccAddress1State != value) {
                            light[idx].dccAddress1State = value;
                        }
                        processed = true;
                    } else if (light[idx].dccAddressOffset == (offsetAddress - 1)) {
                        if (light[idx].dccAddress2State != value) {
                            light[idx].dccAddress2State = value;
                        }
                        processed = true;
                    }
                    break;
                }
                case DOUBLE: {
                    if (light[idx].dccAddressOffset == offsetAddress) {
                        if (light[idx].dccAddress1State != value) {
                            light[idx].dccAddress1State = value;
                        }
                        processed = true;
                    } else if (light[idx].dccAddressOffset == (offsetAddress - 1)) {
                        if (light[idx].dccAddress2State != value) {
                            light[idx].dccAddress2State = value;
                        }
                        processed = true;
                    } else if (light[idx].dccAddressOffset == (offsetAddress - 2)) {
                        if (light[idx].dccAddress3State != value) {
                            light[idx].dccAddress3State = value;
                        }
                        processed = true;
                    } else if (light[idx].dccAddressOffset == (offsetAddress - 3)) {
                        if (light[idx].dccAddress4State != value) {
                            light[idx].dccAddress4State = value;
                        }
                        processed = true;
                    }
                    break;
                }
                default:
                    break;
            }
            if (processed) {
                setLedTable();
                break;
            }
        }
    }
}

/*
 * Callback function to send ACK
 */
void notifyCVAck( void ) {

    // Set ACK pin for about 8ms
    digitalWrite(ACK_PIN, HIGH);
    delay(8);
    digitalWrite(ACK_PIN, LOW);

}

/*
 * Callback function used to log CV changes
 */
void notifyCVChange(uint16_t CV, uint8_t Value) {

#ifdef DEBUG
    Serial.print("CV "); Serial.print(CV, DEC); Serial.print(" changed, value (hex) : "); Serial.print(Value, HEX); Serial.println();
#endif

    // Check if is in resetting state to prevent double execution settings
    if (resettingState) {
        return;
    }

    // Change CVs lights configuration
    if (CV == CONFIG_LIGHTS_MIN_CV_ADDR || CV == CONFIG_LIGHTS_MAX_CV_ADDR) {
#ifdef DEBUG
        Serial.println("Setting new lights configuration");
#endif
        loadLightsConfiguration();
    }
}

/*
 * Callback function to reset CVs & parameters to default values
 */
void notifyCVResetFactoryDefault() {

#ifdef DEBUG
    Serial.println("Factory reset!");
#endif

    // Activate reset state
    resettingState = true;

    // Reset DCC accessory address to 1
    Dcc.setCV(1, 1);
    Dcc.setCV(9, 0);

    // Reset CVs values
    Dcc.setCV(CONFIG_LIGHTS_MIN_CV_ADDR, DEFAULT_MIN_CV_VALUE);
    Dcc.setCV(CONFIG_LIGHTS_MAX_CV_ADDR, DEFAULT_MAX_CV_VALUE);

    // Deactivate reset state
    resettingState = false;

    // Load new configuration
    loadLightsConfiguration();
    setLedTable();
}
