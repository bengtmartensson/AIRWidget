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

   Developed by Arduino forum member 6v6gt ( https://forum.arduino.cc/u/6v6gt ) up to V0_14 and afterwards by Graham Dixon.

   Issued under MIT Licence as described above.

   Tested with  IRscope 3.00 (option Arduino Widget) and an Arduino Nano CH340 clone (other combinations may work)

   Uses photo sensor QSE159 connected to 5V and GND power pins with output to Nano pin D3 and an LED to show signal reception connected between 5V and
   D3 through a 1K resistor. D3 must be pulled to 5V with a 4.7k resistor.  An optional Session Ready LED may be connected between D6 and GND
   also through a 1K resistor.

   2 modes of operation are selectable (1) AIRwidget mode [default] (2) IRwidget compatibility mode
   Reset the Nano after changing the mode.

   (1) AIRwidget mode. This works with an unmodified Nano.
   (2) IRwidget compatibility mode. This requires 
        (a) a connection from the Nano's USB/UART chip RTS pin to Nano pin D8 
        (b) a 10uF capacitor jumpered between the Nano reset pin and ground
            to suppress the DTR signal which the host application (such as IRscope 2.01a, IRscrutinizer etc.)
            may otherwise send,  forcing the Nano into a reset operation. Remove this jumper temporarily
            when uploading code to the Nano.
        (c) a jumper from pin D4 to ground    



   2022-09-30 V0_04  ready for user tests
   2022-10-01 V0_05  restructure state machine to run at loop speed instead of 100uS intervals
                     direct read of rtsPin
   2022-10-01 V0_06  handle timeout from host correctly. Switch off captureLedPin
   2022-10-02 V0_07  restructure to ignore RTS.
                     LED pin 6 now indicates Widget ready to accept IR.
                     Eliminate loop overhead
                     Times out of an IR session 2 seconds after last pulse detected
   2022-10-04 V0_08  Cleanup. Activate pull up resistor on irSense pin.
   2022-10-08 V0_09  Deliver a 0 on first pulse
   2022-10-09 V0_10  rework timer0, disable capture ready led
   2022-10-15 V0_11  various pin monitor experiments
   2022-10-15 V0_12  clean up, reinstate ready led
   2022-10-20 V0_13  tests with IRscrutinizer / IRwidget compatibility+
   2022-10-27 V0_14  IRwidget Compatibility Mode set by jumper on D4.
                       D4 jumpered to ground = IRwidget compatibility mode
                       otherwise AIRwidget mode
   2022-10-31 V0_15  Remove reset of irPulseCount in START state, change
                       value written to Serial in WAIT_FIRST_PULSE state
                       from 0 to lastIrPulseCount and correspondingly in
                       WAIT_FIRST_PULSE the conditional test on irPulseCount
                       being from !=0 to !=lastIrPulseCount
*/

/*
    To do:
    At end of accepatance, increment to version 1.00, deleted revision history
    and delete this notice

*/



#define RTSHIGH ((PINB & 1<<0))  // pin 8 = PB0. change this if rtsPin is changed
#define RTSLOW  (!(PINB & 1<<0))  // pin 8 = PB0. change this if rtsPin is changed



const uint8_t irSense = 3 ; // must be an external Interrupt pin
const uint8_t modeSelectPin = 4 ; // compatibility with IR widget
const uint8_t captureLedPin = 6 ;  // optional - indicating the device is ready to receive IR data
const uint8_t rtsPin = 8 ;  // compatibilityMode (warning: direct port access also)

volatile uint8_t irPulseCount = 0 ;  // cumulative pulses found
enum state_t { START, WAIT_FIRST_PULSE, IN_SEND_TO_HOST } ;
state_t state ;
state_t stateOld ;

uint8_t compatibilityMode ;  // false for AIRwidget mode; true for IRwidget compatibility mode


void extISR( ) {
  // ISR called by externalInterrupt
  irPulseCount++ ; // rolls over at 0xFF
}

void changeState( state_t stateNew ) {
  stateOld = state ;
  state = stateNew ;
}



void setup() {
  pinMode( irSense, INPUT_PULLUP ) ;
  pinMode( modeSelectPin, INPUT_PULLUP ) ;
  pinMode( captureLedPin, OUTPUT ) ;
  pinMode( rtsPin, INPUT_PULLUP ) ; // compatibilityMode

  Serial.begin( 115200 ) ;  // 115200 baud, SERIAL_8N1: 8bits, no parity, 1 stop bit (default)
  attachInterrupt( digitalPinToInterrupt( irSense ) , extISR, FALLING ) ;

  state = state_t::IN_SEND_TO_HOST ;  // different to START
  changeState( state_t::START ) ;

  compatibilityMode = digitalRead( modeSelectPin ) == LOW ? true : false ;

  // reconfigure timer0 (millis(), micros() etc. disabled)
  TCCR0A = bit(WGM00) | bit(WGM01) ;  // fast PWM
  TCCR0B = bit(WGM02) | bit(CS01) ;   // fast PWM, prescaler /8
  OCR0A = 199 ; // 10KHz with PS = /8 with 16MHz clock
  TIMSK0 = 0 ;  // no timer0 interrupts
  TCNT0 = 0 ;

  uint32_t lastIrPulseAtTicks = 0 ;  // 100uS ticks
  uint8_t lastIrPulseCount = 0 ;
  uint32_t ticks100us = 0 ;  // 100uS counter


  for (;;) {

    switch ( state ) {

      case state_t::START : {

          if ( state != stateOld ) {
            digitalWrite( captureLedPin, LOW ) ;
            Serial.flush() ; // ??
            stateOld = state ;
          }

          if ( ( ! compatibilityMode ) || ( compatibilityMode && RTSLOW ) ) {
            digitalWrite( captureLedPin, HIGH ) ;
            changeState( state_t::WAIT_FIRST_PULSE ) ;
            lastIrPulseAtTicks = ticks100us ;
          }
          break ;
        }


      case state_t::WAIT_FIRST_PULSE : {

          if (  compatibilityMode && RTSHIGH ) {
            changeState( state_t::START ) ;
          }

          if ( irPulseCount  != lastIrPulseCount ) {
            // initialise timer0
            cli() ;
            TCNT0 = 0 ;
            TIFR0 |= bit( TOV0)  ;
            sei() ;
            changeState( state_t::IN_SEND_TO_HOST ) ;
            digitalWrite( captureLedPin, LOW ) ;
            Serial.write( lastIrPulseCount ) ;  // binary
          }
          break ;
        }


      case state_t::IN_SEND_TO_HOST : {

          if ( TIFR0 & bit( TOV0 )  ) {  // every 100us
            TIFR0 |= bit( TOV0)  ; // reset flag
            ticks100us++ ;
            Serial.write( irPulseCount ) ;  // binary
            if ( irPulseCount != lastIrPulseCount ) {
              // we are still getting pulses
              lastIrPulseAtTicks = ticks100us ;
              lastIrPulseCount = irPulseCount ;
            }
            else if ( !compatibilityMode && (ticks100us - lastIrPulseAtTicks > 5000UL) ) {  // 0.5 secconds without pulse
              // start again Xus after last pulse received
              changeState( state_t::START ) ;
            }
            if ( compatibilityMode && RTSHIGH ) {
              // rts host says stop
              changeState( state_t::START ) ;
            }
          }
          break ;
        }

    }  // switch

  }  // for
}

void loop() {}
