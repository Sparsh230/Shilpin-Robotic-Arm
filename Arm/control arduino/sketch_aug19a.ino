#include <Stepper.h>

// 28BYJ-48
const int STEPS = 2048;

// Base: D2 D3 D4 D5
Stepper baseMotor(STEPS, 2, 4, 3, 5);

// Middle: D6 D7 D8 D9
Stepper middleMotor(STEPS, 6, 8, 7, 9);

// Upper: D10 D11 D12 D13
Stepper upperMotor(STEPS, 10, 12, 11, 13);

void setup() {
  Serial.begin(9600);

  baseMotor.setSpeed(5);
  middleMotor.setSpeed(5);
  upperMotor.setSpeed(5);

  Serial.println("================================");
  Serial.println("3 DOF ROBOT ARM TEST");
  Serial.println("================================");
  Serial.println("Commands:");
  Serial.println("B+  = Base forward");
  Serial.println("B-  = Base backward");
  Serial.println("M+  = Middle forward");
  Serial.println("M-  = Middle backward");
  Serial.println("U+  = Upper forward");
  Serial.println("U-  = Upper backward");
  Serial.println("STOP = stop command");
  Serial.println("================================");
}

void loop() {

  if (Serial.available() > 0) {

    String command = Serial.readStringUntil('\n');
    command.trim();

    // BASE
    if (command == "B+") {
      Serial.println("Base forward");
      baseMotor.step(100);
    }

    else if (command == "B-") {
      Serial.println("Base backward");
      baseMotor.step(-100);
    }

    // MIDDLE
    else if (command == "M+") {
      Serial.println("Middle forward");
      middleMotor.step(100);
    }

    else if (command == "M-") {
      Serial.println("Middle backward");
      middleMotor.step(-100);
    }

    // UPPER
    else if (command == "U+") {
      Serial.println("Upper forward");
      upperMotor.step(100);
    }

    else if (command == "U-") {
      Serial.println("Upper backward");
      upperMotor.step(-100);
    }

    else if (command == "STOP") {
      Serial.println("STOP");
    }

    else {
      Serial.println("Unknown command");
    }
  }
}