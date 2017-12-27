// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
// Syringe Injector Model 2 (NEMA23 + PCB 20171101 + TB6600)
// Firmware Version 2017-12-18 -REV2
// -----------------------------------------------------------
//
// Usage Notes :
//
//  * The stepper driver is always active (hardwired). 
//    It may overheat if left on for too long (depends on the trimpot settings)
//
//  * After turning the power on, it is required to home the injector. 
//    It's designed to prevent injection if no homing has been done.
//
// -----------------------------------------------------------
// 
// References :
//
// interrupts - https://www.arduino.cc/en/Reference/attachInterrupt
// Micro, Leonardo, other 32u4-based 0, 1, 2, 3, 7
//
// http://www.glennsweeney.com/tutorials/interrupt-driven-analog-conversion-with-an-atmega328p
// https://bennthomsen.wordpress.com/arduino/peripherals/continuous-adc-capture/
//  http://diyprojects.eu/wiring-and-running-tb6600-stepper-driver-with-arduino/
//
// -----------------------------------------------------------
// 
// tbd in next versions
//    measure amounts, measure pressure (conflicts with interrupt, or will make it slow)
//
// -----------------------------------------------------------
//
//  TB6600 Configuration
//    M1 - High |
//    M2 - High | - > 1/16 microstepping
//    M3 - Low  |
//    M4 - Low   -> LOW = Driver us locked until power off then on (security) ; HIGH = autoreset
// 
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 


////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION

// Serial communication settings
#define SERIAL_SPEED 115200   // bauds per second

// Accurate measuring & Logs
// NOTE // These values have not been tuned. 
// Speed has been chosen experimentally.
#define SYRINGE_AREA (3.1415*18.0/2.0*18.0/2.0) // one cylinder, mm2; 1mm3 = 1microliter
#define MICROSTEPPING 16.0 // Microstepping mode
#define SCREW_PITCH 1.0 // in mm
#define STEPS_PER_MM (200.0*MICROSTEPPING/SCREW_PITCH)
#define MICROLITER_PER_STEP (1.0*SYRINGE_AREA/STEPS_PER_MM)

// PCB Pinout > DRIVER TB6600
// NOTE // The Enabled Pin has been bypassed. The driver is always on. Carefull with overheat (trimpot)
#define PIN_STP 10  // Step 
#define PIN_DIR 11  // Direction
#define PIN_ENA 12  // Enabled  
#define MOTOR_ON HIGH // matches the driver board config
#define MOTOR_OFF LOW // matches the driver board config


// PCB Pinout > Sensors
#define PIN_ENDSTOP 2  // Endstop (homing)
#define PIN_FSR 999    // FSR / Syringe plunger detection    // NOT IMPLEMENTED !! 

// PCB Pinout > Keypad Function Keys
#define BTN_F1 9  // Forward Slow > Inject
#define BTN_F2 8  // Stop 
#define BTN_F3 7  // Backwards Slow > Remove Pressure 
#define BTN_F4 5  // Dump Status Informations over serial / usb (requires a connexion to a computer)
#define BTN_F5 4  // Forward Fast     // NOTE: FSR NOT IMPLEMENTED, The user has to Stop it manually.
#define BTN_F6 3  // Backward Fast    // Stops when the endstop is triggered

// PCB Pinout > Stepper Speed Slider 
#define PIN_SLIDER A1   // NOT IMPLEMENTED !!


// Stepper control choices
#define DIR_FWD LOW
#define DIR_RWD HIGH
#define DIR_STOP 999


// Speed control
#define FREQ_LOW 500  // Steps per second
#define FREQ_HIGH 100 // Steps per second

// interval is the half period (toggle up down)
#define INTVL_LOW_MS  300 // 500 // (unsigned long)((1/FREQ_LOW)*1000000/2)  // Bug in CAST
#define INTVL_VERYLOW_MS  600 // 20171222 - implementing a very slow speed for priming
#define INTVL_HIGH_MS  30 // (unsigned long)((1/FREQ_HIGH)*1000000/2)
#define INTVL_HOMING_MS 30 
#define INTVL_CHGRATE_MS 1 

#define INTVL_BTN_MS 200*1000 // 200 ms

#define MAX_STEPS 426265ul  //556293ul // before it hits the wall
#define SYRBEG_STEPS 59630ul // 80266ul // just before the beginning of syringe

#define MINUTES_BEFORE_MOTORSHUTOFF 5UL
#define MINUTES_TO_MICROS 60000000UL
//unused - #define MICROS_BEFORE_MOTORSHUTOFF (MINUTES_BEFORE_MOTORSHUTOFF * MINUTES_TO_MICROS)

////////////////////////////////////////////////////////////////////////////////////////////////
// BOOT

void setup() {

  // Attach interrupts that detect when the endstop & the FSR are triggered
  attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP), stepper_stop_endstop, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FSR), stepper_stop_FSR, RISING);  // NOT USED
  
  // Set pinmodes for the stepper driver controls  
  pinMode(PIN_STP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); 

  // The PCB relies on the internal PULLUP of the arduino board
  pinMode(BTN_F1, INPUT_PULLUP);
  pinMode(BTN_F2, INPUT_PULLUP);
  pinMode(BTN_F3, INPUT_PULLUP);
  pinMode(BTN_F4, INPUT_PULLUP);
  pinMode(BTN_F5, INPUT_PULLUP);
  pinMode(BTN_F6, INPUT_PULLUP);


  pinMode(PIN_ENDSTOP, INPUT);
  pinMode(PIN_FSR, INPUT);
  pinMode(PIN_SLIDER, INPUT); // NOT USED NOT TESTED
  
  // Not used
  // pinMode(LED_BUILTIN, OUTPUT);


  // Initialize Step & Dir Pins
  digitalWrite( PIN_DIR, DIR_RWD);
  digitalWrite( PIN_STP, LOW);
  digitalWrite(PIN_ENA, MOTOR_OFF); // Start with the steppers disabled
  
  // Open serial 
  Serial.begin(SERIAL_SPEED);
  
  // Disabled (would wait forever if no computer is attached to the usb)
  /* while (!Serial) {
   ; // wait for serial port to connect. Needed for native USB port only
  } */

  Serial.print("INTVL_LOW_MS "); Serial.println(INTVL_LOW_MS);
  Serial.print("INTVL_HIGH_MS "); Serial.println(INTVL_HIGH_MS);
  
}

////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES, WITH INITIAL STATE INITIALISATION

int dir = DIR_STOP;                               // Direction of the stepper motor
int stepVal = HIGH;                               // Step initial state, goes with the logic of the functions

unsigned long ts = micros();                      // Last known time reading from the microcontroller
                                                  // 2^32/1000000/60 = 71 minutes - https://www.arduino.cc/reference/en/language/functions/time/micros/
unsigned long minutes_since_stop = 0UL;           // we keep track of minutes elapsed since stop (disable driver)
unsigned long ts_last_step = ts;                  // stepper heartbeat : Previous time reading (to calc the diff)
unsigned long ts_last_keypad = ts;                // keypad heartbeat : Previous time reading (to calc the diff)
unsigned long ts_last_minutesTicker = ts;         // Count minutes : Previous time reading (to calc the diff)


unsigned long ts_interval = INTVL_LOW_MS;         // Default "current speed" for the stepper (usefull for speed rampup calcs)
unsigned long ts_interval_target = INTVL_LOW_MS;  // Default "target speed" for the stepper (usefull for speed rampup calcs)


boolean FSRseekMode = true;                       // Seek for a syringe ? // NOT IMPLEMENTED, could be buggy.
boolean postEndstop = false;                      // Endstop triggers the interrupt twice. We need to distinguish these to update the stepper direction

unsigned long stepCount = 0;                      // We count steps. Up to 4 billion steps.
unsigned long stepCountAtFSR = 0;                 // Stores the nb steps last time we hit the syringe // NOT IMPLEMENTED - could be buggy


////////////////////////////////////////////////////////////////////////////////////////////////
// THE LOOP
//
/* Code Logic :

  * Consistency of timings is implemented in the loop via "heartbeat" conditions
  * There are two heartbeats:
   - stepper hearbeat : triggers a step every [ts_interval] (global)
   - keyboard heartbeat: reads the value every [INTVL_BTN_MS] 
  * The speed of the stepper is ramped up, or down, progressively to [ts_interval_target]. 
    The current interval is stored in [ts_interval] (global)
  * The number of steps are counted, in [stepCount]
    [stepCount] is reset during homing
  * Some safeguards are implemented
    - stepper : [ts_interval] is limited to { min, max } = { INTVL_LOW_MS , INTVL_HOMING_MS }
    - avoid hitting the front wall : [stepCount] < [MAX_STEPS]
    - the fast forward is not disabled while injecting ( [stepCount] > [SYRBEG_STEPS] )
      because it is a needed option. In future versions, it may be opportune to 
      implement limited fast forward during injection, by a max duration or max number of steps
  * Heartbeats rely on the internal clock of the microcontroller. It's accurate enough for our use.
    The clock information is stored in [ts] (global).
    This is done in the stepper heartbeat condition (as well as [ts_last_step])


  * The machine behavior is dicated by the state in which it is.
  * The machine state is updated by 
      - either via the keypad buttons
      - either internal events (override)

  * Keypad Buttons - To simplify -a lot- the algo (and keep conditions evals minimal => speedy), 
    buttons read continously. Meaning, pressing a button does not log an action that is completed before 
    another action can be done. Instead, a button simply updates and reupdates the state of machine.

  * Machine States (synthetic -> read the code if you're looking for details & edge cases):
    
    - Injection / Slow Forward :
              low stepper speed [ts_interval_target] = [INTVL_LOW_MS] 
              && direction forward [dir] = [DIR_FWD]
    
    - Vent out bubbles / Eject Mold / Fast Forward :
              maximum stepper speed [ts_interval_target] = [INTVL_HOMING_MS] 
              && direction forward [dir] = [DIR_FWD]

    - Home the plungers / Fast Backwards : 
              maximum stepper speed [ts_interval_target] = [INTVL_HOMING_MS] 
              && direction backwards [dir] = [DIR_RWD]

    - Slow Backward / Ease the placement of the syringe / Fine tuning :
              low stepper speed [ts_interval_target] = [INTVL_LOW_MS] 
              && direction forward [dir] = [DIR_FWD]
    
    - Stop / Emergency stop :
              disables the evaluation of the stepper heartbeat (the stepper wont move)
                via [dir] = [DIR_STOP]
                it's been coded this way so that this button also plays the role of an emergency stop
                when the driver enable / disable will be implemented, the driver state should be set to OFF
                so that the user can easily rotate the motor with his hands (may damage the stepper driver though)
              && 
                sets the speed to low so that we can restart without skipping steps
                   [ts_interval_target] = [INTVL_LOW_MS] 

    - Dump data :
              derogates from the state machine principle. 
              Instead, it immediately dumps on the serial / to the computer the current steps & other infos
              keeping the button pressed will slow down fast speeds
              Can be enhanced (nice to have)
                      
              
*/



void loop() {

  ///////////////////
  // SAFEGUARDS
  // 
  // Enforce minimum and maximum stepper speeds
  //    Normally, these should never be triggered, however, improperly fed coders do make logic mistakes. 
  //    Keep those conditions to avoid weird noises when they make mistakes.
  if ( ts_interval < INTVL_HOMING_MS ) { ts_interval = INTVL_HOMING_MS; }
  if ( ts_interval > INTVL_VERYLOW_MS ) { ts_interval = INTVL_VERYLOW_MS; }
  // 
  // Prevent hitting walls
  //    If the operator gets distracted ...
  if ( stepCount > MAX_STEPS && dir == DIR_FWD ) { dir = DIR_STOP; }
  ///////////////////
  


  ///////////////////
  // FOR ALL HEARTBEATS
  // Update time readings
  ts = micros();
  unsigned long ts_diff_step = ts - ts_last_step;
  unsigned long ts_diff_keypad = ts - ts_last_keypad;
  unsigned long ts_diff_minutesTicker = ts - ts_last_minutesTicker;
  // handle overflow
  if ( ts > (4294967296UL - 102UL) ) { // multiple of 8
    delayMicroseconds(102+8);
    ts = micros();
    ts_last_step = ts_diff_step + 8 ;
    ts_last_keypad = ts_diff_keypad + 8;
    ts_last_minutesTicker = ts_diff_minutesTicker + 8;
  }
  
  ///////////////////
  // MOTOR ON/OFF HEARTBEAT
  //
  // Update the minutes timer (shutoff stepper if inactive)
  // BUG: The driver board doesn't seem to have this function implemented.  Don't overheat the motor (trimpot)
  // 
  if ( ts_diff_minutesTicker > MINUTES_TO_MICROS ) { // check is done every minute
    ts_last_minutesTicker = ts;
    // Increment the minute counter if the motor is idle, otherwise, reset the counter
    // the reenable part is also put in the keypad loop, so that it is more responsive (faster heartbeat)
    if ( dir == DIR_STOP ) { 
      minutes_since_stop += 1;
      
      // power off ...
      if ( minutes_since_stop > MINUTES_BEFORE_MOTORSHUTOFF ) {
        digitalWrite( PIN_ENA, MOTOR_OFF );
        Serial.println("Disabling Motors");
      } else {
        // power on // bulletproofing against bugs, shouldn't be needed.
        digitalWrite( PIN_ENA, MOTOR_ON );
      }

    } else {
      // power on ...
      minutes_since_stop = 0;
      digitalWrite( PIN_ENA, MOTOR_ON );
    }
   }      
  
  

  ///////////////////
  // KEYPAD HEARTBEAT  => SETS THE STATE OF THE MACHINE
  // 
  if ( ts_diff_keypad > INTVL_BTN_MS ) {
    
    // Updates the last time the keypad heartbeat was evaluated - keep this first
    ts_last_keypad = ts;


    // Ramps up -or down- the current speed until it matches the target speed
    // This call has been put here because it is evaluated less often than directly in the loop
    //   it artifically slows down the rate at which the rampup happens
    //    and avoids using a dedicated heartbeat just for that ... it's a dirty fix, but it reliably and efficiently works.
    if ( ts_interval < ts_interval_target ) { ts_interval += INTVL_CHGRATE_MS; }
    if ( ts_interval > ts_interval_target ) { ts_interval -= INTVL_CHGRATE_MS; }


    int btnVal = 0; // Identifies which button is being pressed at the moment of the reading
                    // placed here to make sure that the code is amnesic / the previous reading is forgotten

    // Poll of the buttons that are active
    // In the even of Multiple buttons being pressed at the same time, the last one in this list wins
    // change the order if you feel another one should be winning (obviously, "emergency stop")
    if ( digitalRead(BTN_F3) == LOW ) { btnVal = 3; Serial.println("F3"); }
    if ( digitalRead(BTN_F4) == LOW ) { btnVal = 4; Serial.println("F4");  }
    if ( digitalRead(BTN_F5) == LOW ) { btnVal = 5; Serial.println("F5");  }
    if ( digitalRead(BTN_F6) == LOW ) { btnVal = 6; Serial.println("F6");  }
    if ( digitalRead(BTN_F1) == LOW ) { btnVal = 1; Serial.println("F1"); } // should be "homing" if the pinouts are correct
    if ( digitalRead(BTN_F2) == LOW ) { btnVal = 2; Serial.println("F2");  } // should be "stop" if the pinouts are correct

    // Implementation of the state machine logics (see notes hereabove)
    // The following code assumes that the buttons are set like this:
            // F1 : EXTRUDE NORMAL 
            // F2 : EXTRUDE SLOW
            // F3 : EXTRUDE STOP
            // F4 : EXTRUDE RWD
            // REMOVED - F4 : PRINT CONSUMPTION & RESET
            // F5 : FAST FORWARD / SEEK FSR
            // F6 : FAST BACKWARDS / SEEK ENDSTOP
    if ( btnVal == 1 ) { dir = DIR_FWD; ts_interval_target = INTVL_LOW_MS; digitalWrite( PIN_DIR, DIR_FWD); }  
    if ( btnVal == 2 ) { dir = DIR_FWD; ts_interval_target = INTVL_VERYLOW_MS; digitalWrite( PIN_DIR, DIR_FWD); }  
    if ( btnVal == 3 ) { dir = DIR_STOP; ts_interval_target = INTVL_LOW_MS; }
    if ( btnVal == 4 ) { dir = DIR_RWD; ts_interval_target = INTVL_LOW_MS; digitalWrite( PIN_DIR, DIR_RWD); }
    // REPLACED ---- if ( btnVal == 4 ) { Serial.print(ts); Serial.print(" - Steps: "); Serial.print(stepCount); Serial.println(""); /* float microliters = MICROLITER_PER_STEP * stepCount; Serial.print(" muL: "); Serial.println(microliters); */ }
    if ( btnVal == 5 ) { dir = DIR_FWD; ts_interval = INTVL_LOW_MS; ts_interval_target = INTVL_HIGH_MS; digitalWrite( PIN_DIR, DIR_FWD);  }
    if ( btnVal == 6 ) { dir = DIR_RWD; ts_interval = INTVL_LOW_MS; ts_interval_target = INTVL_HOMING_MS; digitalWrite( PIN_DIR, DIR_RWD); }
    

    // Revive the motors if needed
    // Done here (duplicate from above) so that we have a faster response time.
    // The duplicate has been kept to help understanding the code
    if ( dir == DIR_FWD || dir == DIR_RWD ) {
      minutes_since_stop = 0;
      digitalWrite( PIN_ENA, MOTOR_ON );
    }

  }

  ///////////////////
  // HANDLE ENDSTOP TRIGGERS
  // 
  // Prevents smashing the endstop for too long
  // Post Endstops handling occurs before the stepper heartbeat, so that the heatbeat starts with a clean initial state
  // It also is a blocking call, so that it prevents reading the buttons while this op is done ...
  // 
  if ( postEndstop ) {  
    stepper_post_endstop();
  }


  ///////////////////
  // STEPPER HEARTBEAT  => MAKES THE MOTOR MOVE.
  //
  if ( ts_diff_step > ts_interval && dir != DIR_STOP ) {
    // keep track of stepCount (resets on endstop hit and FSR hit)
    if ( dir == DIR_FWD ) { stepCount = stepCount + 1UL; } 
    if ( dir == DIR_RWD ) { stepCount = stepCount - 1UL; } 
    // Choose how to make a step
    if ( stepVal == HIGH ) { stepVal = LOW; } else { stepVal = HIGH; }
    // Updated after the calculations, so that we keep an accurate timing
    ts_last_step = ts;
    // make the step
    digitalWrite( PIN_STP, stepVal );
  }
  
}



////////////////////////////////////////////////////////////////////////////////////////////////
// SUPPORT FUNCTIONS
//
/* Notes :
  ...
*/


// The blocking sequence to apply after the endstop has been hit
// Buttons will be disables while this is being done. Keep it short.
void stepper_post_endstop() {
  Serial.println("## POST ENDSTOP SEQU");
  Serial.println( digitalRead(PIN_ENDSTOP) );
  digitalWrite( PIN_DIR, DIR_FWD);
  // for (int k=0;k<300; k++) {
  int k = 0;
  while ( digitalRead(PIN_ENDSTOP) == LOW ){
    k++;
    stepVal = LOW;
    digitalWrite( PIN_STP, stepVal );
    delayMicroseconds(INTVL_LOW_MS);
    stepVal = HIGH;
    digitalWrite( PIN_STP, stepVal );
    delayMicroseconds(INTVL_LOW_MS);
  }
  Serial.print("## DONE, Steps: ");
  Serial.println(k);
  dir = DIR_STOP;
  postEndstop = false;
  stepCount = 0; // reset
}


// Handles the endstop trigger. 
// This function is called by the interrupt only.
// Keep it super short + special care with global vars updated
// The logic is that it overrides the stepper heartbeat.
// The function does nothing if the endstop has already been hit
//   && the stepper_post_endstop() has started but not finished (relies on global variables ...)
// 
void stepper_stop_endstop() {
  if ( ! postEndstop ) {
    Serial.println("ENDSTOP");
    if ( dir == DIR_RWD ) {
      Serial.println("ENDSTOP !!");
      postEndstop = true;
      dir = DIR_STOP;
      stepCount = 0; // reset
    }    
  } 
}

// For the FSR. NOT IMPLEMENTED, NOT TESTED. Can be buggy.
void stepper_stop_FSR() {
  if ( FSRseekMode ) {
    dir = DIR_STOP;
    stepCountAtFSR = stepCount;
    ts_interval = INTVL_LOW_MS;
    ts_interval_target = INTVL_LOW_MS;
  }
}

/* 
// UNUSED - 
void reset_stepCount() {
  stepCount = 0;
}

// UNUSED - 
void print_consumption() {
  float consumption = (stepCount * MICROLITER_PER_STEP);
  Serial.print("Consumed ");
  Serial.print(consumption);
  Serial.println(" ml");
}
*/