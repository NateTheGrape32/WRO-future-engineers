#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <Servo.h>
#include <Arduino_BMI270_BMM150.h>

#define SERVO_PIN 9
#define MOTOR_PIN 8

#define MAX_LINE 16
#define START_CHAR '$'

Servo servo;
Servo motor;

char command[MAX_LINE + 1];
int commandLength = 0;
bool readingCommand = false;

float gyroBiasZ = 0;
float heading = 0;
unsigned long lastTime = 0;

//gyro
void calibrateGyro() {
  Serial.println("Keep still — calibrating gyro...");

  float sumZ = 0;
  const int samples = 2000;

  for (int i = 0; i < samples; i++) {
    float x, y, z;

    while (!IMU.gyroscopeAvailable());

    IMU.readGyroscope(x, y, z);
    sumZ += z;

    delay(3);
  }

  gyroBiasZ = sumZ / samples;

  Serial.print("Gyro Z bias: ");
  Serial.println(gyroBiasZ, 4);
}


void startupCalibration() {
  calibrateGyro();

  heading = 0.0f;
  lastTime = micros();

  Serial.println("Ready!");
}


//commands
void processCommand(char *cmd) {
  if (cmd == nullptr || cmd[0] == '\0')
    return;

  // Remove carriage return if there is one
  char *cr = strchr(cmd, '\r');
  if (cr)
    *cr = '\0';

  // Skip spaces
  while (*cmd && isspace(*cmd))
    cmd++;

  if (*cmd == '\0')
    return;

  char type = *cmd++;

  char* endp = nullptr;
  long value = strtol(cmd, &endp, 10);

  switch (type) {

    case 'C':   // Calibrate
      startupCalibration();
      break;

    case 'S':   // Servo
      if (value >= 25 && value <= 130)
        servo.write(value);
      break;

    case 'M':   // Motor
      if (value >= 1000 && value <= 2000)
        motor.writeMicroseconds(value);
      break;

    case 'L':   // LED
      if (value >= 22 && value <= 24) {
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDB, LOW);

        digitalWrite(value, HIGH);
      }
      break;

    case 'I':   // Print heading
      Serial.println(heading);
      break;
  }
}


//setup

void setup() {
  // LEDs
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);

  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
  digitalWrite(24, LOW);

  // Servo and motor
  servo.attach(SERVO_PIN, 900, 2100);
  motor.attach(MOTOR_PIN, 1000, 2000);

  servo.write(80);
  motor.writeMicroseconds(1500);

  // Serial
  Serial.begin(115200);
  while (!Serial);

  // IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (true);
  }

  Serial.println("IMU initialized successfully.");
  startupCalibration();
}


//loop
void loop() {

  // Read serial commands
  if (Serial.available()) {
    char c = Serial.read();

    if (c == START_CHAR) {
      readingCommand = true;
      commandLength = 0;
    }
    else if (readingCommand) {

      if (c == '\n') {
        command[commandLength] = '\0';

        processCommand(command);

        readingCommand = false;
        commandLength = 0;
      }

      else if (commandLength < MAX_LINE) {
        command[commandLength++] = c;
      }

      else {
        // Command was too long
        readingCommand = false;
        commandLength = 0;
      }
    }
  }


  // Read gyro
  if (IMU.gyroscopeAvailable()) {

    float x, y, z;
    IMU.readGyroscope(x, y, z);

    z -= gyroBiasZ;

    // Ignore small gyro noise
    if (abs(z) < 0.1)
      z = 0;

    unsigned long now = micros();

    float dt = (now - lastTime) * 0.000001f;

    lastTime = now;

    // Integrate angular velocity into heading
    heading -= z * dt;
  }
}
