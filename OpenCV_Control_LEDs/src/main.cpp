#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

const char *ssid = "LXJ 1558";
const char *password = "big L on you";

Servo myServo;
AsyncWebServer server(80);

const int thumbLedPin = 27;
const int indexLedPin = 26;
const int middleLedPin = 25;
const int ringLedPin = 33;
const int pinkyLedPin = 32;
const int servoPin = 13;

// Count how many "fingers" (LEDs) are ON
int countFingersOn() {
  int count = 0;
  if (digitalRead(thumbLedPin) == HIGH) count++;
  if (digitalRead(indexLedPin) == HIGH) count++;
  if (digitalRead(middleLedPin) == HIGH) count++;
  if (digitalRead(ringLedPin) == HIGH) count++;
  if (digitalRead(pinkyLedPin) == HIGH) count++;
  return count;
}

// Set servo angle based on number of fingers
void setServoByFingerCount() {
  int fingers = countFingersOn();
  int angle = fingers * 8;
  myServo.write(angle);
  Serial.print("Fingers detected: ");
  Serial.print(fingers);
  Serial.print(" -> Servo angle: ");
  Serial.println(angle);
}

// Registers on/off handlers for a given LED
void setupLedHandlers(const char* name, int pin) {
  String onPath = String("/led/") + name + "/on";
  String offPath = String("/led/") + name + "/off";

  server.on(onPath.c_str(), HTTP_GET, [pin, name](AsyncWebServerRequest *request) {
    digitalWrite(pin, HIGH);
    setServoByFingerCount();
    request->send(200, "text/plain", String(name) + " LED is ON");
  });

  server.on(offPath.c_str(), HTTP_GET, [pin, name](AsyncWebServerRequest *request) {
    digitalWrite(pin, LOW);
    setServoByFingerCount();
    request->send(200, "text/plain", String(name) + " LED is OFF");
  });
}

void setup() {
  Serial.begin(115200);

  pinMode(thumbLedPin, OUTPUT);
  pinMode(indexLedPin, OUTPUT);
  pinMode(middleLedPin, OUTPUT);
  pinMode(ringLedPin, OUTPUT);
  pinMode(pinkyLedPin, OUTPUT);
  digitalWrite(thumbLedPin, LOW);
  digitalWrite(indexLedPin, LOW);
  digitalWrite(middleLedPin, LOW);
  digitalWrite(ringLedPin, LOW);
  digitalWrite(pinkyLedPin, LOW);

  myServo.attach(servoPin);
  myServo.write(0);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Register endpoints using function
    // setupLedHandlers("thumb", thumbLedPin);
    // setupLedHandlers("index", indexLedPin);
    // setupLedHandlers("middle", middleLedPin);
    // setupLedHandlers("ring", ringLedPin);
    // setupLedHandlers("pinky", pinkyLedPin);

    server.begin();
    Serial.println("Server started");
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the whole line
    int angle = input.toInt();                    // Convert to integer
    angle = constrain(angle, 0, 180);             // Clamp between 0–180°
    myServo.write(angle);                         // Move the servo
    Serial.print("Servo moved to: ");
    Serial.println(angle);
  }
}

