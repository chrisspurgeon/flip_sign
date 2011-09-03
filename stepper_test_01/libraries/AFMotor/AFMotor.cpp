#include <avr/io.h>
#include "WProgram.h"
#include "AFMotor.h"

static uint8_t latch_state;

#define MICROSTEPS 16  // 8, 16 & 32 are popular

//#define MOTORDEBUG 1

AFMotorController::AFMotorController(void) {
}

void AFMotorController::enable(void) {
  // setup the latch
  LATCH_DDR |= _BV(LATCH);
  ENABLE_DDR |= _BV(ENABLE);
  CLK_DDR |= _BV(CLK);
  SER_DDR |= _BV(SER);
  latch_state = 0;

  latch_tx();  // "reset"

  ENABLE_PORT &= ~_BV(ENABLE); // enable the chip outputs!
}


void AFMotorController::latch_tx(void) {
  uint8_t i;

  LATCH_PORT &= ~_BV(LATCH);
  SER_PORT &= ~_BV(SER);
  for (i=0; i<8; i++) {
    CLK_PORT &= ~_BV(CLK);
    if (latch_state & _BV(7-i))
      SER_PORT |= _BV(SER);
    else
      SER_PORT &= ~_BV(SER);
    CLK_PORT |= _BV(CLK);
  }
  LATCH_PORT |= _BV(LATCH);
}

static AFMotorController MC;


/******************************************
               MOTORS
******************************************/


AF_DCMotor::AF_DCMotor(uint8_t num, uint8_t freq) {
  motornum = num;
  pwmfreq = freq;

  MC.enable();

  switch (num) {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    
    // use PWM from timer2A
    TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc0
    TCCR2B = freq & 0x7;
    OCR2A = 0;
    DDRB |= _BV(3);
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    
    // use PWM from timer2B
    TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
    TCCR2B = freq & 0x7;
    DDRD |= _BV(3);
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    
    // use PWM from timer0A
    TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0b
    //TCCR0B = freq & 0x7;
    OCR0A = 0;
    DDRD |= _BV(6);
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    
    // use PWM from timer0B
    TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a
    //TCCR0B = freq & 0x7;
    OCR0B = 0;
    DDRD |= _BV(5);
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  switch (motornum) {
  case 1:
    OCR2A = speed; break;
  case 2:
    OCR2B = speed; break;
  case 3:
    OCR0A = speed; break;
  case 4:
    OCR0B = speed; break;
  }
}

/******************************************
               STEPPERS
******************************************/

AF_Stepper::AF_Stepper(uint16_t steps, uint8_t num) {
  MC.enable();

  revsteps = steps;
  steppernum = num;

  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
    
#ifdef MICROSTEPPING

    // use PWM for microstepping support
    // use PWM from timer2B
    TCCR2A = _BV(COM2B1) | _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc0a AND oc0b
    TCCR2B = _BV(CS20);
  

    OCR2B = 255;
    OCR2A = 255;
#endif

    // enable both H bridges
    DDRD |= _BV(3);
    PORTD |= _BV(3);
    DDRB |= _BV(3);
    PORTB |= _BV(3);
  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();

#ifdef MICROSTEPPING    
    // use PWM for microstepping support
    // use PWM from timer0B
    TCCR0A = _BV(COM0B1) | _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a AND oc0b
    // tccr0b is already used for timing so its 1khz
    OCR0B = 255;
    OCR0A =255;
#endif

    // enable both H bridges
    DDRD |= _BV(5) | _BV(6);
    PORTD |= _BV(5) | _BV(6);
  }
}

void AF_Stepper::setSpeed(uint16_t rpm) {
  usperstep = 60000000 / (revsteps * rpm);
  steppingcounter = 0;
}

void AF_Stepper::release(void) {
    if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();
  }
}

void AF_Stepper::step(uint16_t steps, uint8_t dir,  uint8_t style) {
  uint32_t uspers = usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE) {
    uspers /= 2;
  }
#ifdef MICROSTEPPING
 else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = "); Serial.println(steps, DEC);
#endif
  }
#endif

  while (steps--) {
    ret = onestep(dir, style);
    delay(uspers/1000); // in ms
    steppingcounter += (uspers % 1000);
    if (steppingcounter >= 1000) {
      delay(1);
      steppingcounter -= 1000;
    }
  }
#ifdef MICROSTEPPING
  if (style == MICROSTEP) {
    //Serial.print("last ret = "); Serial.println(ret, DEC);
    while ((ret != 0) && (ret != MICROSTEPS)) {
      ret = onestep(dir, style);
      delay(uspers/1000); // in ms
      steppingcounter += (uspers % 1000);
      if (steppingcounter >= 1000) {
	delay(1);
	steppingcounter -= 1000;
      } 
    }
  }
#endif

}

uint8_t AF_Stepper::onestep(uint8_t dir, uint8_t style) {
  uint8_t a, b, c, d;
  uint8_t step;
  uint8_t mstep = 0;
#ifdef MICROSTEPPING
  uint8_t ocrb, ocra;
#endif
  if (steppernum == 1) {
    a = _BV(MOTOR1_A);
    b = _BV(MOTOR2_A);
    c = _BV(MOTOR1_B);
    d = _BV(MOTOR2_B);

#ifdef MICROSTEPPING
    ocra = OCR2A;
    ocrb = OCR2B;

    if (style == MICROSTEP) {
      //TCCR2B = _BV(CS21);
    }
#endif
  } else if (steppernum == 2) {
    a = _BV(MOTOR3_A);
    b = _BV(MOTOR4_A);
    c = _BV(MOTOR3_B);
    d = _BV(MOTOR4_B);

#ifdef MICROSTEPPING
    ocra = OCR0A;
    ocrb = OCR0B;
 
    if (style == MICROSTEP) {
      //TCCR0B = _BV(CS00);
    }   
#endif

  } else {
    return 0;
  }

#ifdef MOTORDEBUG
  Serial.print("a = "); Serial.print(ocra, DEC);
  Serial.print(" b = "); Serial.print(ocrb, DEC);
  Serial.print("\t");
#endif

  // OK next determine what step we are at 
  if ((latch_state & (a | b)) == (a | b))
    step = 1 * MICROSTEPS; 
  else if ((latch_state & (b | c)) == (b | c))
    step = 3 * MICROSTEPS; 
  else if ((latch_state & (c | d)) == (c | d))
    step = 5 * MICROSTEPS;
  else if ((latch_state & (d | a)) == (d | a))
    step = 7 * MICROSTEPS;
  else if (latch_state & a)
    step = 0;
  else if (latch_state & b)
    step = 2 * MICROSTEPS;
  else if (latch_state & c)
    step = 4 * MICROSTEPS;
  else
    step = 6 * MICROSTEPS;

  //Serial.print("step "); Serial.print(step, DEC); Serial.print("\t");
  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE) {
    if ((step/MICROSTEPS) % 2) { // we're at an odd step, weird
      if (dir == FORWARD)
	step = (step + MICROSTEPS) % (8*MICROSTEPS);
      else
	step = (step + 7*MICROSTEPS) % (8*MICROSTEPS);
    } else {           // go to the next even step
      if (dir == FORWARD)
	step = (step + 2*MICROSTEPS) % (8*MICROSTEPS);
      else
	step = (step + 6*MICROSTEPS) % (8*MICROSTEPS);  

    }
#ifdef MICROSTEPPING
    ocra = 255;
    ocrb = 255;
#endif

  } else if (style == DOUBLE) {
    if (! (step/MICROSTEPS % 2)) { // we're at an even step, weird
      if (dir == FORWARD)
	step = (step + MICROSTEPS) % (8*MICROSTEPS);
      else
	step = (step + 7*MICROSTEPS) % (8*MICROSTEPS);
    } else {           // go to the next odd step
      if (dir == FORWARD)
	step = (step + 2*MICROSTEPS) % (8*MICROSTEPS);
      else
	step = (step + 6*MICROSTEPS) % (8*MICROSTEPS);
    }
#ifdef MICROSTEPPING
    ocra = 255;
    ocrb = 255;
#endif
  } else if (style == INTERLEAVE) {
     if (dir == FORWARD)
       step = (step + 1*MICROSTEPS) % (8*MICROSTEPS);
     else
       step = (step + 7*MICROSTEPS) % (8*MICROSTEPS);
#ifdef MICROSTEPPING
    ocra = 255;
    ocrb = 255;
#endif
  } 
#ifdef MICROSTEPPING
  else if (style == MICROSTEP) {

    //Serial.print("Step #"); Serial.print(step/MICROSTEPS, DEC); Serial.print("\t");
    if (dir == FORWARD) {

      // if at even step, make sure its 'odd'
      if (! (step/MICROSTEPS) % 2) {
	step = (step + MICROSTEPS) % (8*MICROSTEPS);
      }

      // fix hiccups from changing direction
      if (((ocra == 255) && ((step/MICROSTEPS)%4 == 3)) ||
	  ((ocrb == 255) && ((step/MICROSTEPS)%4 == 1))) {
	step += 2*MICROSTEPS;
	step %= 8*MICROSTEPS;
      }

      if ((step == MICROSTEPS) || (step == 5*MICROSTEPS)) {
	// get the current microstep
	if (ocrb == 255)
	  mstep = MICROSTEPS;
	else
	  mstep = ocrb / (256UL / MICROSTEPS);
#ifdef MOTORDEBUG
	Serial.print("uStep = "); Serial.print(mstep, DEC);
#endif
	// ok now go to next step
	mstep++;
	mstep %= (MICROSTEPS+1);
	if (mstep == MICROSTEPS)
	  ocrb = 255;
	else 
	  ocrb = mstep * (256UL / MICROSTEPS);
	ocra = 255 - ocrb;
#ifdef MOTORDEBUG
	Serial.print(" -> "); Serial.println(mstep, DEC);
#endif
	if (mstep == MICROSTEPS)
	  step = (step + 2*MICROSTEPS) % (8*MICROSTEPS);
      } else {
	// get the current microstep
	if (ocrb == 255)
	  mstep = MICROSTEPS;
	else
	  mstep = ocrb / (256UL / MICROSTEPS);
#ifdef MOTORDEBUG
	Serial.print("uStep = "); Serial.print(mstep, DEC);
#endif
	// ok now go to next step
	mstep += MICROSTEPS;
	mstep %= (MICROSTEPS+1);
	if (mstep == MICROSTEPS)
	  ocrb = 255;
	else 
	  ocrb = mstep * (256UL / MICROSTEPS);
	ocra = 255 - ocrb;
#ifdef MOTORDEBUG
	Serial.print(" +> "); Serial.println(mstep, DEC);
#endif
	if (mstep == 0)
	  step = (step + 2*MICROSTEPS) % (8*MICROSTEPS);
      }
    } else {

      // fix hiccups from changing direction
      if (((ocra == 255) && ((step/MICROSTEPS)%4 == 1)) ||
	  ((ocrb == 255) && ((step/MICROSTEPS)%4 == 3))) {
	step = (step + 6*MICROSTEPS);
	step %= (8*MICROSTEPS);
      }

      // if at even step, make sure its 'odd'
      if (! (step/MICROSTEPS % 2)) {
	step = (step + 7*MICROSTEPS) % (8*MICROSTEPS);
      }
      if ((step == MICROSTEPS) || (step == 5*MICROSTEPS)) {
	// get the current microstep
	if (ocrb == 255)
	  mstep = MICROSTEPS;
	else
	  mstep = ocrb / (256UL / MICROSTEPS);
#ifdef MOTORDEBUG
	Serial.print(" uStep = "); Serial.print(mstep, DEC);
#endif
	// ok now go to next step
	mstep += MICROSTEPS;
	mstep %= (MICROSTEPS+1);
	if (mstep == MICROSTEPS)
	  ocrb = 255;
	else
	  ocrb = mstep * (256UL / MICROSTEPS);
	ocra = 255 - ocrb;
#ifdef MOTORDEBUG
	Serial.print(" !> "); Serial.println(mstep, DEC);
#endif
	if (mstep == 0)
	  step = (step + 6*MICROSTEPS) % (8*MICROSTEPS);
      } else {
	// get the current microstep
	if (ocrb == 255)
	  mstep = MICROSTEPS;
	else
	  mstep = ocrb / (256UL / MICROSTEPS);
#ifdef MOTORDEBUG
	Serial.print("uStep = "); Serial.print(mstep, DEC);
#endif
	// ok now go to next step
	mstep++;
	mstep %= (MICROSTEPS + 1);
	if (mstep == MICROSTEPS)
	  ocrb = 255;
	else 
	  ocrb = mstep * (256UL / MICROSTEPS);
	ocra = 255 - ocrb;
#ifdef MOTORDEBUG
	Serial.print(" *> "); Serial.println(mstep, DEC);
#endif
	if (mstep == MICROSTEPS)
	  step = (step + 6*MICROSTEPS) % (8*MICROSTEPS);
      }
    }
  }


  //Serial.print(" -> step = "); Serial.print(step/MICROSTEPS, DEC); Serial.print("\t");

  if (steppernum == 1) {
    OCR2A = ocra;
    OCR2B = ocrb;
  } else if (steppernum == 2) {
    OCR0B = ocrb;
    OCR0A = ocra;
  }

#endif  // microstepping

  // release all
  latch_state &= ~a & ~b & ~c & ~d; // all motor pins to 0

  //Serial.println(step, DEC);

  switch (step/MICROSTEPS) {
  case 0:
    latch_state |= a; // energize coil 1 only
    break;
  case 1:
    latch_state |= a | b; // energize coil 1+2
    break;
  case 2:
    latch_state |= b; // energize coil 2 only
    break;
  case 3:
    latch_state |= b | c; // energize coil 2+3
    break;
  case 4:
    latch_state |= c; // energize coil 3 only
    break; 
  case 5:
    latch_state |= c | d; // energize coil 3+4
    break;
  case 6:
    latch_state |= d; // energize coil 4 only
    break;
  case 7:
    latch_state |= d | a; // energize coil 1+4
    break;
  }
  MC.latch_tx();
  return mstep;
}

