/*
  AIRWidget

  Copyright (C) 2022 Tommy Tyler

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

   Note:

   Developed by Arduino forum member 6v6gt ( https://forum.arduino.cc/u/6v6gt )
   and currently maintained by Graham Dixon.

   Issued under MIT Licence as described above.

   AIRWidget mode tested with  IRscope 3.05 (option Arduino Widget) and an Arduino
   Nano CH340 clone (other combinations may work).

   Uses photo sensor QSE159 connected to 5V and GND power pins with output to Nano
   pin D3 and an LED to show signal reception connected between 5V and D3 through a
   1K resistor. D3 must be pulled to 5V with a 4.7k resistor.  An optional Session
   Ready LED may be connected between D6 and GND also through a 1K resistor.

   Two modes of operation are selectable through the boolean parameter compatibilityMode:
   (1) AIRwidget mode. This works with an unmodified Nano and host applications such as
         IRScope v3.05 and IrScrutinizer v2.4.0.  It is selected by setting
         compatibilityMode=false.
   (2) IRwidget compatibility mode. This requires
        (a) a connection from the Nano's USB/UART chip RTS pin to Nano pin D8.
        (b) a 10uF capacitor jumpered between the Nano reset pin and ground to suppress
            the DTR signal which the host application (such as IRscope v2.01a and
            IRscrutinizer v2.3.1) may otherwise send, forcing the Nano into a reset
            operation.  Remove this jumper temporarily when uploading code to the Nano.
        (c) setting compatibilityMode=true.
   Reset the Nano after changing the mode.

   Version history:
   2022-11-02 V1_00  first official release

*/

#define BAUD 115200
#define SENSOR_VCC 3
//#define SENSOR_GND

static const uint8_t irSense = 2; // must be an external Interrupt pin
static const uint8_t captureLedPin = LED_BUILTIN;  // optional - indicating the device is ready to receive IR data

volatile uint8_t irPulseCount = 0 ;  // cumulative pulses found
enum state_t {
    START,
    WAIT_FIRST_PULSE,
    IN_SEND_TO_HOST
};
state_t state;
state_t stateOld;

void extISR() {
    // ISR called by externalInterrupt
    irPulseCount++; // rolls over at 0xFF
}

void changeState(state_t stateNew) {
    stateOld = state;
    state = stateNew;
}

void setup() {
#ifdef SENSOR_VCC
    pinMode(SENSOR_VCC, OUTPUT);
    digitalWrite(SENSOR_VCC, HIGH);
#endif
#ifdef SENSOR_GND
    pinMode(SENSOR_GND, OUTPUT);
    digitalWrite(SENSOR_GND, LOW);
#endif

    pinMode(irSense, INPUT_PULLUP);
    pinMode(captureLedPin, OUTPUT);

    Serial.begin(BAUD); // 115200 baud, SERIAL_8N1: 8bits, no parity, 1 stop bit (default)
    attachInterrupt(digitalPinToInterrupt(irSense), extISR, FALLING);

    state = state_t::IN_SEND_TO_HOST; // different to START
    changeState(state_t::START);

    // reconfigure timer0 (millis(), micros() etc. disabled)
    TCCR0A = bit(WGM00) | bit(WGM01); // fast PWM
    TCCR0B = bit(WGM02) | bit(CS01); // fast PWM, prescaler /8
    OCR0A = 199; // 10KHz with PS = /8 with 16MHz clock
    TIMSK0 = 0; // no timer0 interrupts
    TCNT0 = 0;
}

uint32_t lastIrPulseAtTicks = 0; // 100uS ticks
uint8_t lastIrPulseCount = 0;
uint32_t ticks100us = 0; // 100uS counter

void loop() {
    switch (state) {
        case state_t::START:
        {
            if (state != stateOld) {
                digitalWrite(captureLedPin, LOW);
                //Serial.flush(); // ??
                stateOld = state;
            }

            digitalWrite(captureLedPin, HIGH);
            changeState(state_t::WAIT_FIRST_PULSE);
            lastIrPulseAtTicks = ticks100us;
            break;
        }

        case state_t::WAIT_FIRST_PULSE:
        {
            if (irPulseCount != lastIrPulseCount) {
                // initialise timer0
                cli();
                TCNT0 = 0;
                TIFR0 |= bit(TOV0);
                sei();
                changeState(state_t::IN_SEND_TO_HOST);
                digitalWrite(captureLedPin, LOW);
                Serial.write(lastIrPulseCount); // binary
            }
            break;
        }

        case state_t::IN_SEND_TO_HOST:
        {
            if (TIFR0 & bit(TOV0)) { // every 100us
                TIFR0 |= bit(TOV0); // reset flag
                ticks100us++;
                Serial.write(irPulseCount); // binary
                if (irPulseCount != lastIrPulseCount) {
                    // we are still getting pulses
                    lastIrPulseAtTicks = ticks100us;
                    lastIrPulseCount = irPulseCount;
                } else if (ticks100us - lastIrPulseAtTicks > 5000UL) {
                    // start again 500us after last pulse received
                    changeState(state_t::START);
                }
            }
            break;
        }
    } // switch
}
