#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <ctype.h>

Servo servo;
byte servoPin = 9;
static int pos = 0;
static int pulse = 500;
static char state = 0;
static char line[16];
static size_t lineLen = 0;

void setup() {
    Serial.begin(115200);
    servo.attach(servoPin, 900, 2100); //Adjusts PWM ranges to fit the HS-5055MG servo
}

void loop() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r') continue;

        if (c == 'a' || c == 'b') {
            state = c;
            lineLen = 0;
            continue;
        }

        if (state) {
            line[lineLen++] = c;
            if (lineLen >= sizeof(line) - 1 || c == '\n') {
                line[lineLen] = '\0';
                if (state == 'a') {
                    pos = atoi(line);
                    pos = constrain(pos, 0, 180);
                    pulse = map(pos, 0, 180, 900, 2100);
                    servo.writeMicroseconds(pulse);
                    Serial.print("Servo position: ");
                    Serial.println(pos);
                } else if (state == 'b') {
                    //The commented code would work with the Nano 33 BLE Sense
                    //Apparently, writeMicroseconds is a little broken on the Nano 33 BLE Sense
                    pulse = atoi(line);
                    pulse = constrain(pulse, 900, 2100);
                    servo.writeMicroseconds(pulse);
                    Serial.print("Servo pulse: ");
                    Serial.println(pulse);
                    // Convert microseconds to angle and use write() instead of writeMicroseconds()
                    // Standard mapping: 544us = 0°, 2400us = 180°
                    /*int angle = map(pulse, 900, 2100, 0, 180);
                    servo.write(angle);
                    Serial.print("Servo pulse (as angle): ");
                    Serial.println(angle);*/
                }
            }
        }
    }
}