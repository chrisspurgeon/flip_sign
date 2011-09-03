#include <AFMotor.h>


#include "WProgram.h"
void setup();
void loop();
AF_Stepper motor(48, 2);


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  motor.setSpeed(10);  // 10 rpm   

  motor.release();
  delay(1000);
  motor.step(100, BACKWARD, INTERLEAVE); 
}

void loop() {

}


int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

