#include <AFMotor.h>


AF_Stepper motor(48, 1);


int position = 0;
int STEPS = 96;

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("Stepper test!");

    motor.setSpeed(10);  // 10 rpm   

    motor.release();
    delay(1000);
    motor.step(100, BACKWARD, INTERLEAVE); 
}

void loop() {
    if (Serial.available() > 0) {
        int inByte = Serial.read();

        switch (inByte) {
        case 'a':
            motorAdvance(0);
            break;
        case 'b':
            motorAdvance(4);
            break;
        case 'c':
            motorAdvance(8);
            break;
        case 'd':
            motorAdvance(12);
            break;
        case 'e':
            motorAdvance(16);
            break;
        case 'f':
            motorAdvance(20);
            break;
        case 'g':
            motorAdvance(24);
            break;
        case 'h':
            motorAdvance(28);
            break;
        case 'i':
            motorAdvance(32);
            break;
        case 'j':
            motorAdvance(36);
            break;
        case 'k':
            motorAdvance(40);
            break;
        case 'l':
            motorAdvance(44);
            break;
        case 'm':
            motorAdvance(48);
            break;
        case 'n':
            motorAdvance(52);
            break;
        case 'o':
            motorAdvance(56);
            break;
        case 'p':
            motorAdvance(60);
            break;
        case 'r':
            motorAdvance(64);
            break;
        case 's':
            motorAdvance(68);
            break;
        case 't':
            motorAdvance(72);
            break;
        case 'u':
            motorAdvance(76);
            break;
        case 'v':
            motorAdvance(80);
            break;
        case 'w':
            motorAdvance(84);
            break;
        case 'y':
            motorAdvance(88);
            break;
        case 'z':
            motorAdvance(92);
            break;
        case '0':
    motor.step(1, BACKWARD, INTERLEAVE);
            break;
        } 
    }
}

void motorAdvance(int N) {
    int change = 0;
    if (position <  N) {
        change = N - position;
    }
    if (position > N) {
        change = N - position + STEPS;
    }

    Serial.println("I think change is ");
    Serial.println(change, DEC);
    motor.step(change, BACKWARD, INTERLEAVE);
    position = N;

}




