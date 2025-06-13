#include <ESP32Servo.h>

// Create Servo objects
Servo gripServo;      // D27
Servo waistServo;     // D28
Servo elbowServo;     // D26
Servo waistRotServo;  // D25
Servo shoulderServo1; // D33
Servo shoulderServo2; // D32

// Angle ranges
const int gripMin = 0, gripMax = 85;
const int waistMin = 30, waistMax = 150;
const int elbowMin = 0, elbowMax = 90;
const int waistRotMin = 0, waistRotMax = 180;
const int shoulderMin = 40, shoulderMax = 130;

void testServo(Servo &servo, const char* name, int minAngle, int maxAngle) {
  Serial.print("Testing ");
  Serial.println(name);

  for (int angle = minAngle; angle <= maxAngle; angle += 10) {
    servo.write(angle);
    delay(30);
  }
  delay(200);
  for (int angle = maxAngle; angle >= minAngle; angle -= 10) {
    servo.write(angle);
    delay(30);
  }
  delay(300);
}

void testShouldersTogether(int minAngle, int maxAngle) {
  Serial.println("Testing Shoulders Together");
  for (int angle = minAngle; angle <= maxAngle; angle += 10) {
    shoulderServo1.write(angle);
    shoulderServo2.write(angle);
    delay(30);
  }
  delay(200);
  for (int angle = maxAngle; angle >= minAngle; angle -= 10) {
    shoulderServo1.write(angle);
    shoulderServo2.write(angle);
    delay(30);
  }
  delay(300);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  gripServo.attach(27);
  gripServo.write(0);
  waistServo.attach(28);
  elbowServo.attach(26);
  waistRotServo.attach(25);
  shoulderServo1.attach(33);
  shoulderServo2.attach(32);

  Serial.println("Starting servo test...");
}

void loop() {
  // testServo(gripServo, "Grip", gripMin, gripMax);
  // testServo(waistServo, "Waist", waistMin, waistMax);
  // testServo(elbowServo, "Elbow", elbowMin, elbowMax);
  // testServo(waistRotServo, "Waist Rotate", waistRotMin, waistRotMax);
  // testShouldersTogether(shoulderMin, shoulderMax);
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the whole line
    int angle = input.toInt();                    // Convert to integer
    angle = constrain(angle, 0, 180);             // Clamp between 0–180°
    gripServo.write(angle);                         // Move the servo
    Serial.print("Servo moved to: ");
    Serial.println(angle);
  }

  delay(1000);
}
