#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

Servo motor;
byte motorPin = 8;
static char dir = 0;
static char line[16];
static size_t lineLen = 0;

void setup() {
  Serial.begin(115200);
  motor.attach(motorPin, 1000, 2000); // Adjusts PWM ranges to fit the BLDC motor controller

  // ESC arming sequence
  motor.writeMicroseconds(1500); // Reset motor to neutral
  delay(3000);
  Serial.println("ESC armed. Ready to recieve commands.");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == 'a' || c == 'b') {
      dir = c;
      lineLen = 0;
      continue;
    }

    if (dir) {
      line[lineLen++] = c;
        if (lineLen >= sizeof(line) - 1 || c == '\n') {
          line[lineLen] = '\0';
          int speed = atoi(line);
          if (dir == 'a') { //Sets direction
            speed = constrain(speed, 1000, 1500);
          } else if (dir == 'b') { // Sets direction
            speed = constrain(speed, 1500, 2000);
          }
          
          motor.writeMicroseconds(speed);

          Serial.print("Motor direction: ");
          Serial.println(dir);
          Serial.print("Motor speed: ");
          Serial.print(abs(speed - 1500)/5.0);
          Serial.println("%");

          dir = 0;
      }
    }
  }
}