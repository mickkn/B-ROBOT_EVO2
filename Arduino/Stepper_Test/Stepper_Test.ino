#define ENABLE_MOTORS 4

#define STEP_M1 11
#define DIR_M1  8

#define STEP_M2 12
#define DIR_M2  5

const int SLOW_DELAY_US = 8000;   // very slow
const int FAST_DELAY_US = 2000;
const int STEPS_PER_BLOCK = 800;

void setup() {
  pinMode(ENABLE_MOTORS, OUTPUT);

  pinMode(STEP_M1, OUTPUT);
  pinMode(DIR_M1, OUTPUT);
  pinMode(STEP_M2, OUTPUT);
  pinMode(DIR_M2, OUTPUT);

  digitalWrite(ENABLE_MOTORS, LOW); // try HIGH if no holding torque

  digitalWrite(DIR_M1, HIGH);
  digitalWrite(DIR_M2, HIGH);

  digitalWrite(STEP_M1, LOW);
  digitalWrite(STEP_M2, LOW);

  delay(1000);
}

void stepBoth(int delayUs) {
  digitalWrite(STEP_M1, HIGH);
  digitalWrite(STEP_M2, HIGH);
  delayMicroseconds(delayUs);

  digitalWrite(STEP_M1, LOW);
  digitalWrite(STEP_M2, LOW);
  delayMicroseconds(delayUs);
}

void loop() {
  // very slow forward
  for (int i = 0; i < STEPS_PER_BLOCK; i++) {
    stepBoth(SLOW_DELAY_US);
  }

  delay(1000);

  // reverse direction
  digitalWrite(DIR_M1, !digitalRead(DIR_M1));
  digitalWrite(DIR_M2, !digitalRead(DIR_M2));

  delay(1000);
}