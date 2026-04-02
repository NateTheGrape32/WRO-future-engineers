#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>
#include <Arduino_BMI270_BMM150.h>

static const size_t MAX_LINE = 16;  // Max command line length, excluding EOL
static char lineBuf[MAX_LINE + 1];
static size_t lineLen = 0;
static const char SOC = '$';  // start of command character
static bool inCommand = false;
static float biasX = 0.0, biasY = 0.0, biasZ = 0.0;
Servo servo;
Servo motor;
#define SERVO_PIN 9
#define MOTOR_PIN 8

static void calibrateGyro(float& biasX, float& biasY, float& biasZ) {
  const int samples = 1000;
  int collected = 0;

  delay(200);  // wait for sensor to warm up

  while (collected < samples) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      biasX += gx;
      biasY += gy;
      biasZ += gz;
      collected++;

      delay(5);  // match gyro's ODR of ~100-200 Hz
    }
  }
  biasX /= samples;
  biasY /= samples;
  biasZ /= samples;
}

static void driveStraight(float correction, int speed=1800) {
  motor.writeMicroseconds(speed);
  int pwm = map(correction, -6000, 6000, 900, 2100);  // Limit correction to valid servo angles
  servo.writeMicroseconds(pwm);
}



static void processCommand(char* cmd) {
  if (!cmd || cmd[0] == '\0') return;

  char* cr = strchr(cmd, '\r');
  if (cr) *cr = '\0';

  while (*cmd && isspace((unsigned char)*cmd)) cmd++;  // Trim leading whitespace
  if (*cmd == '\0') return;

  char type = *cmd++;

  // Parse int argument
  char* endp = nullptr;
  long val = strtol(cmd, &endp, 10);

  switch (type) {
    case 'S':                            // Set servo position
      if (val < -6000 || val > 6000) return;  // Invalid angle
      driveStraight(val);
      break;
    case 'T':  // return IMU data and turn
      val < 0 ? servo.writeMicroseconds(900) : servo.writeMicroseconds(2100);
      motor.writeMicroseconds(1600);
      float gx, gy, gz;
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        Serial.println(gx - biasX);
      }
      delay(10);
      break;
    default:
      // Ignore unknown command type
      break;
  }
}

void setup() {
  servo.attach(SERVO_PIN, 900, 2100);
  motor.attach(MOTOR_PIN, 1000, 2000);

  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  Serial.println("IMU initialized successfully.");

  calibrateGyro(biasX, biasY, biasZ);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == SOC) {
      inCommand = true;
      lineLen = 0;
    }
    if (inCommand) {
      if (c == '\n') {
        lineBuf[lineLen] = '\0';  // Null-terminate command
        processCommand(lineBuf);
        inCommand = false;
        lineLen = 0;
        continue;
      }

      if (lineLen < MAX_LINE) {
        lineBuf[lineLen++] = c;  // Append char to command buffer
      } else {
        // Command too long
        inCommand = false;
        lineLen = 0;
      }
    }
  }
}