#define ENABLE_PIN 8
#define PULSE_PIN 13


int position = 0;
int STEPS = 96;

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("Stepper test!");
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(PULSE_PIN, OUTPUT);
}

void loop() {
    digitalWrite(ENABLE_PIN, LOW);

    digitalWrite(PULSE_PIN, HIGH);
    Serial.println("Step");

    delay(2);                  // wait for a second
    digitalWrite(PULSE_PIN, LOW);    // set the LED off
    delay(5);
}


