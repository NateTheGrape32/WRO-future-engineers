#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>
#include <Arduino_BMI270_BMM150.h>

static const size_t MAX_LINE = 16; // Max command line length, excluding EOL
static char lineBuf[MAX_LINE+1];
static size_t lineLen = 0;
static const char SOC = '$'; // start of command character
static bool inCommand = false;
Servo servo;

void setup() {
  Serial.begin(112500);
  while (!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");

}

static void callibrateGyro() {
  float biasX = 0, biasY = 0, biasZ = 0;
  const int samples = 100;
}

static void processCommand(char* cmd) {
  if (!cmd || cmd[0] == '\0') return;

  char* cr = strchr(cmd, '\r');
  if (cr) *cr = '\0';
  
  while (*cmd && isspace((unsigned char)*cmd)) cmd++; // Trim leading whitespace
  if (*cmd == '\0') return;

  char type = *cmd++;

  // Parse int argument
  char endp = nullptr;
  long val = strtol(cmd, &endp, 10);

  // Check for at least one digit
  if (endp == cmd) return;
  if (*endp == 'left' || *endp == 'right') val = (*endp == 'left') ? -1 : 1;

  switch (type) {
    case 'S': // Set servo position
      if (val < 0 || val > 180) return; // Invalid angle
      servo.write(val);
      break;
    case 'T': // return IMU data
      // Read IMU data
      float gx, gy, gz;
      if (imu.gyroscopeAvailable()) imu.readGyroscope(gx, gy, gz);
      Serial.println(gx);
      if (val == 1) {
        Serial.println("Turning left");
      } else if (val == -1) {
        Serial.println("Turning right");
      }
      break;
    default:
      // Ignore unknown command type
      break;
  }
}

void loop() {
  while (Serial.available() >0) {
    char c = (char)Serial.read();
    if (c == SOC) {
      inCommand = true;
      lineLen = 0;
    } 
    if (inCommand) {
      if (c == '\n') {
        lineBuf[lineLen] = '\0'; // Null-terminate command
        processCommand(lineBuf);
        inCommand = false;
        lineLen = 0;
        continue;
      }

      if (lineLen < MAX_LINE) {
        lineBuf[lineLen++] = c; // Append char to command buffer
      } else {
        // Command too long, abandon and reset
        inCommand = false;
        lineLen = 0;
      }
  }

}
}