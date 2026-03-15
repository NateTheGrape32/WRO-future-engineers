#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <ctype.h>

Servo servo;
static int pos = 0;
static int pulse = 500;
static char state = '';
static char line[4];
static size_t lineLen = 0;

void setup() {
    Serial.begin(112500);
    servo.attach(9);
}

void loop() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == 'a') {
            state = 'a';
        } else {
            state = 'b';
        }

        if (state) {
            line[lineLen++] = c;
            if (lineLen >= sizeof(line) - 1 || c == '\n') {
                line[lineLen] = '\0';
                lineLen = 0;
                if (state == 'a') {
                    pos = stoi(line);
                    if (pos > 180) pos = 180;
                    if (pos < 0) pos = 0;
                    servo.write(pos);
                    Serial.print("Servo position: ");
                    Serial.println(pos);
                } else {
                    pulse = stoi(line);
                    if (pulse > 2500) pulse = 500;
                    if (pulse < 500) pulse = 2500;
                    servo.writeMicroseconds(pulse);
                    Serial.print("Servo pulse: ");
                    Serial.println(pulse);
                }
            }
        }
    }
}