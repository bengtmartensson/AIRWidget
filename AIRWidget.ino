/*
   AIRWidget

   Note:
   
   Tested with the current version of IRscope 201a
   and an Arduino Nano CH340 clone (other combinations may work)

  
   Put a ~10uF capacitor (best with a jumper) between the Nano Reset pin and GND to suppress the 
     reset caused be (unwanted) DTR signal sent from IRscope when the user clicks on "capture".
	   Disconnect this capacitor (via the jumper) during uploading of sketches to the Nano.

   QSE159 output to Nano pin 3
   
   Wigit ready for IR session led (optional) between Nano pin 6 and ground with ~1k resistor

   2022-09-30 V0_04 ready for user tests
   2022-10-01 V0_05 restructure state machine to run at loop speed instead of 100uS intervals
                     direct read of rtsPin
   2022-10-01 V0_06 handle timeout from host correctly. Switch off captureLedPin
   2022-10-02 V0_07 restructure to ignore RTS.
                    LED pin 6 now indicates Widget ready to accept IR.
                    Eliminate loop overhead
                    Times out of an IR session 2 seconds after last pulse detected
   2022-10-04 V0_08 Cleanup. Activate pull up resistor on irSense pin.  
   2022-10-04 V0_09 Deliver a 0 on first pulse             

*/


const uint8_t irSense = 3 ; // must be an external Interrupt pin
const uint8_t captureLedPin = 6 ;  // optional - indicated device ready to receive IR data

const uint16_t sendIntervalUs = 100 ; // microseconds

volatile uint8_t irPulseCount = 0 ;  // cumulative pulses found

enum state_t { START, WAIT_FIRST_PULSE, IN_SEND_TO_HOST } ;
state_t state ;



void extISR( ) {
  // ISR called by externalInterrupt
  irPulseCount++ ; // rolls over at 0xFF
}


void setup() {
  pinMode( irSense, INPUT_PULLUP ) ;
  pinMode( captureLedPin, OUTPUT ) ;
  Serial.begin( 115200 ) ;  // 115200 baud, SERIAL_8N1: 8bits, no parity, 1 stop bit (default)
  attachInterrupt( digitalPinToInterrupt( irSense ) , extISR, FALLING ) ;

  digitalWrite( captureLedPin, LOW ) ;
  state = state_t::START ;


  uint32_t lastSendAtUs = 0 ;
  uint32_t lastIrPulseAtUs = 0 ;
  uint8_t lastIrPulseCount = 0 ;
  uint32_t us ;


  for (;;) {

    us = micros() ;
    
    switch ( state ) {

      case state_t::START : {
          digitalWrite( captureLedPin, HIGH ) ;
          irPulseCount = 0 ; //reset
          state = state_t::WAIT_FIRST_PULSE ;
          lastIrPulseAtUs = us ;
          break ;
        }

      case state_t::WAIT_FIRST_PULSE : {
          if ( 0 != irPulseCount ) {
            lastSendAtUs = us ;  //  V0_09
            state = state_t::IN_SEND_TO_HOST ;
            digitalWrite( captureLedPin, LOW ) ; //  V0_09AIRW
            Serial.write( 0 ) ;  // binary
          }
          break ;
        }

      case state_t::IN_SEND_TO_HOST : {
          if ( us - lastSendAtUs >= sendIntervalUs ) {  // every 100us
      
            Serial.write( irPulseCount ) ;  // binary
            if ( irPulseCount != lastIrPulseCount ) {
              // we are still getting pulses
              lastIrPulseAtUs = us ;
              lastIrPulseCount = irPulseCount ;
            }
            else if ( us - lastIrPulseAtUs > 2000000UL ) {  // 2 seconds
              // start again Xus after last pulse received
              state = state_t::START ;
            }
            lastSendAtUs += sendIntervalUs ;
          }
          break ;
        }

    }  // switch

  }  // for
}

void loop() {}
