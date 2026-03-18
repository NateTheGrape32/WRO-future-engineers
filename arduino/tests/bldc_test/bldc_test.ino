#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <ctype.h>

#define ENABLE 9
static int speed = 50;
static char word = 0;
static char line[16];
static size_t lineLen = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE,OUTPUT); // This pin will control motor speed
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == 'a' || c == 'b' || c == 's') {
      word = c;
      lineLen = 0;
      continue;
    }

    if (word) {
      line[lineLen++] = c;
        if (lineLen >= sizeof(line) - 1 || c == '\n') {
          line[lineLen] = '\0';
          lineLen = 0;
          if (word == 'a') { //Sets direction
            digitalWrite(4, HIGH);
          } else if (word == 'b') { // Sets direction
            digitalWrite(5, HIGH);
          } else if (word == 's') { // Sets speed via PWM
            speed = atoi(line);
            analogWrite(ENABLE, speed);
          }
}