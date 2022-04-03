# Decoder DCC accessori per segnali italiani con vele singole o doppie mediante Arduino e Adafruit TLC5947
![License](https://img.shields.io/badge/License-MIT-green)

*English version [here](README.EN.md)*

## Indice
- [Introduzione](#introduzione)
- [Cosa serve](#cosa-serve)
- [Connessioni](#connessioni)
- [Configurazione](#configurazione)
- [Tabella delle CVs (configuration variables) utilizzate](#tabella-delle-cvs-configuration-variables-utilizzate)
- [Alcune specifiche sul funzionamento dello sketch](#alcune-specifiche-sul-funzionamento-dello-sketch)
- [Versione e contributors](#versione-e-contributors)

## Introduzione
Lo sketch per [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) di questo progetto implementa un completo decoder DCC di tipo accessori per pilotare led, normali o di tipo RGB, da utilizzare nei segnali ferroviari italiani a 1 o 2 vele

![Overview](/images/DCC_ItalianSignals_Overview.jpg)

## Cosa Serve
Per realizzare un decoder DCC con led per segnali ferroviari italiani occorrono:
* una scheda [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3) o [compatibile](https://google.com/search?q=geekcreit+arduino+uno)
* un'interfaccia [Adafruit TLC5947](https://www.adafruit.com/product/1429) (demo disponibile [qui](https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout)) capace di pilotare fino a 24 led; l'interfaccia TLC5947 consente di lavorare con più interfacce a cascata seguendo le istruzioni a questo [link](https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout/connecting-to-the-arduino#chaining-boards-2151225-14)
* uno shield o un'interfaccia che sia in grado di "leggere" il segnale DCC attraverso Arduino, io consiglio vivamente [lo shield o l'interfaccia](https://github.com/lucadentella/arduino-dccshield) dell'amico Luca Dentella

## Connessioni
* Arduino Uno o interfaccia compatibile
    - utilizzare il cavo USB (utile se si vuole monitorare il traffico sulla finestra del monitor seriale) o l'apposito spinotto di alimentazione a 5V
* Interfaccia / shield DCC
    - utilizzare l'apposita morsettiera DCC dell'interfaccia / shield alla quale collegherete il segnale proveniente dalla vostra centralina, il segnale DCC viaggia in corrente alternata e quindi potete collegare sia un senso che nell'altro
    - VCC va collegato ad uno dei pins di Arduino indicati con 5V (tensione)
    - GND va collegato ad uno dei pins di Arduino indicati con GND (massa)
    - DCC va collegato al pin dedicato all'ingresso dati con interrupt, nello sketch il pin dichiarato è il 2
    - ACK va collegato al pin dedicato al segnale di ACK, nello sketch il pin dichiarato è il 3
* TLC5947
    - V+  va collegato ad uno dei pins di Arduino indicati con 5V (tensione)
    - GND va collegato ad uno dei pins di Arduino indicati con GND (massa)
    - DIN va collegato al pin D4
    - CLK va collegato al pin D5
    - LAT va collegato al pin D6
> **Se si decidesse di pilotare un numero abbastanza corposo di led è possibile utilizzare un'alimentazione esterna scollegando il pin V+ da Arduino e collegarlo ad un'alimentazione esterna mettendo in comune le connessioni di massa sui pin GND.**
> **Tutte le istruzioni per utilizzare l'alimentazione esterna sono disponibili a questo [link](https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout/power-and-leds#connecting-an-external-supply-940507-7)!**


![Overview](/images/DCC_ItalianSignals_TLC5947Connections.jpg)

## Configurazione

Lo sketch prevede questa configurazione:
- definizione del numero dei led da pilotare, il valore di default è 24 il massimo consentito dall'interfaccia TLC5947
- 2 specifiche CV per memorizzare la configurazione delle 8 possibili vele utilizzabili con 24 led (ogni vela avrà quindi 3 uscite utilizzate rispettivamente per il segnale rosso, il segnale giallo ed il segnale verde)

## Tabella delle CVs (configuration variables) utilizzate

| CV | Descrizione | Note | Valore di default |
|:--|:--|:--|--:|
| 33 | prime 4 coppie di 2 bit per la configurazione delle vele | | 9 |
| 34 | seconde 4 coppie di 2 bit per la configurazione delle vele | | 0 |

> i bits di ogni CV sono strutturati in questo modo: **b7-b6 b5-b4 b3-b2 b1-b0** ed ogni coppia di bit ha il seguente significato:
> **00** = **nessuna vela**
> **01** = **singola vela**
> **10** = **doppia vela**
> **11** = **non utilizzato** (chi vuole può dedicare questa configurazione per segnali a 3 vele modificando opportunamente lo sketch)

> Il valore di default impostato con il comando di reset inviato al decoder (qualsiasi valore su CV8) imposta la **CV33** a **9** e la **CV34** a **0**
> il significato quindi è il seguente:
> **9** => **00 00 10 01** (partendo da destra: prima configurazione 1 vela, seconda configurazione 2 vele, terza e quarta non gestite)
> **0** => **00 00 00 00** (nessuna vela configurata)
> 
> Se ad esempio volessimo configurare il decoder per pilotare 4 vele singole occorre impostare il valore **85** su **CV33** (corrispondente alla sequenza di bits pari a: **01 01 01 01**)
>
> **TIP**: consiglio questo [link](https://www.binaryhexconverter.com/binary-to-decimal-converter) per calcolare velocemente i valori desiderati

## Alcune specifiche sul funzionamento dello sketch

***
Enumerazione contenente il possibile stato del singolo led:
- **OFF** led spento
- **ON** led acceso
- **NORMAL_BLINKING** indica un lampeggio ogni secondo
- **REVERSED_BLINKING** indica un lampeggio ogni secondo con accensione / spegnimento contrario al lampeggio normale (utilizzato al momento solo nei segnali a doppia vela con giallo e verde che lampeggiano alternativamente)
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
Definizione della struttura di funzionamento di una singola vela, per pilotare i possibili 4 stati della vela occorrono in totale 2 indirizzi DCC le cui combinazioni acceso / spento determinano la modalità di visualizzazione (rosso fisso, verde fisso, giallo fisso, giallo lampeggiante)
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
Definizione della struttura di funzionamento di una doppia vela, per pilotare i possibili 11 stati delle 2 vele occorrono in totale 4 indirizzi DCC le cui combinazioni acceso / spento determinano la modalità di visualizzazione specificate nei commenti sotto riportati
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
Definizione struttura contenente lo stato attuale di ogni singola vela (minimo 4 se tutte a doppia vela, massimo 8 se tutte a singola vela) e il corrispondente valore acceso / spento degli indirizzi DCC corrispondenti
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
Nel setup iniziale all'accensione di Arduino viene eseguito un ciclo per spegnere tutti i led per poi inizializzare ogni vela al suo primo stato impostato (di norma rosso fisso)
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
Ad ogni ciclo di loop dello sketch viene conteggiato il numero di millisecondi dall'accensione per ottenere un contatore virtuale da 1 a 1000 ad indicare il tempo del lampeggio di 1 secondo
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
Esecuzione del ciclo su ogni led per pilotarne il comportamento (spento, acceso, lampeggiante normale o inverso)
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
Funzione che esegue il caricamento della configurazione delle vele, utilizzata sia all'accensione di Arduino che ad ogni cambiamento delle CV 33 e 34.
Vengono in sequenza eseguite le seguenti operazioni:
* inizializzata la variabile **numDCCUsedAddresses** per calcolare il numero di indirizzi DCC utilizzati
* letta la configurazione di 2 bit alla volta tramite l'istruzione **cvValue & 3**
* inizializzata la struttura dei led a seconda che si tratti di vela singola o vela doppia (**SINGLE_LIGHT_CONFIGURATION** o **DOUBLE_LIGHT_CONFIGURATION**)
* shifting di 2 bit a destra per ogni ciclo con l'istruzione **cvValue = cvValue >> 2** 
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
Funzione che esegue il setup di ogni singolo led in funzione della configurazione delle vele, invocata ad ogni cambio di configurazione oppure ad ogni nuovo comando DCC ricevuto indirizzato alle vele gestite. Viene eseguito un ciclo sul'intero gruppo di luci controllando se trattasi di singola vela o doppia vela ed impostando il valore di ogni singolo led in funzione della corrispondente tabella. Se risulta una combinazione non prevista tra quelle censite si ha lo spegnimento di tutti i led corrispondenti
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
Funzione di callback del cambio posizione di uno scambio, viene verificato se il comando è diretto ad una delle vele gestite controllando il range di indirizzo principale assegnato al decoder e numero di indirizzi scaturiti dalla configurazione del numero di vele. Viene inoltre controllato se si è verificato un cambio stato che determina la modifica di visualizzazione delle vele corrispondenti
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

## Versione e Contributors
Versione: 1.0.0
Data di pubblicazione: Aprile 2022
Autore: [Giovanni Colasurdo](mailto:gio.colasurdo@gmail.com)