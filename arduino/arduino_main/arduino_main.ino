#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>
#include <Arduino_BMI270_BMM150.h>

static unsigned long lastTime = 0;
static float gbx=0, gby=0, gbz=0;

static const size_t MAX_LINE = 16;  // Max command line length, excluding EOL
static char lineBuf[MAX_LINE + 1];
static size_t lineLen = 0;
static const char SOC = '$';  // start of command character
static bool inCommand = false;
static float biasX = 0.0, biasY = 0.0, biasZ = 0.0;
Servo servo;
Servo motor;
#define SERVO_PIN 8
#define MOTOR_PIN 9

// Kp = how fast it corrects, Ki = slowly kills gyro bias
#define Kp  2.0f
#define Ki  0.05f

static float heading = 0.0f;

void calibrateGyro() {
  Serial.println("Keep still — calibrating gyro...");
  float sx=0, sy=0, sz=0;
  int n = 2000;
  for (int i=0; i<n; i++) {
    float gx,gy,gz;
    while (!IMU.gyroscopeAvailable());
    IMU.readGyroscope(gx,gy,gz);
    sx+=gx; sy+=gy; sz+=gz;
    delay(3);
  }
  gbx=sx/n; gby=sy/n; gbz=sz/n;
  Serial.print("Gyro bias: ");
  Serial.print(gbx,4); Serial.print(" ");
  Serial.print(gby,4); Serial.print(" ");
  Serial.println(gbz,4);
}


void startupCalibration() {
  calibrateGyro();
  lastTime = micros();
  heading = 0.0f;
  Serial.println("Ready!");
}

static void processCommand(char* cmd, float heading=heading) {
  if (!cmd || cmd[0] == '\0') return;

  char* cr = strchr(cmd, '\r');
  if (cr) *cr = '\0';

  while (*cmd && isspace((unsigned char)*cmd)) cmd++;  // Trim leading whitespace
  if (*cmd == '\0') return;

  char type = *cmd++;

  // Parse int argument
  char* endp = nullptr;
  long val = strtol(cmd, &endp, 10);

  if (type == 'C') {  // (re-)calibrate gyro and zero heading
      startupCalibration();
  } else if (type == 'S') {                            // Set servo position
      if (val < 25 || val > 130) return;  // Invalid angle
      //Serial.println(val);
      servo.write(val);
  } else if (type == 'M') {  // return IMU data and turn
    if (val < 1000 || val > 2000) return;  // Invalid speed
    motor.writeMicroseconds(val);
    //Serial.println(val);
  } else if (type == 'L') {
    if (val < 22 || val > 24) return;  // Invalid LED
    digitalWrite(22, LOW);
    digitalWrite(23, LOW);
    digitalWrite(24, LOW);
    digitalWrite(val, HIGH);
  } else if (type == 'I') {
    Serial.println(heading);
  }
}

void setup() {
  pinMode(22, OUTPUT); // blue (straight)
  pinMode(23, OUTPUT); // red (turning)
  pinMode(24, OUTPUT); // green

  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
  digitalWrite(24, LOW);

  servo.attach(SERVO_PIN, 900, 2100);
  motor.attach(MOTOR_PIN, 1000, 2000);
  servo.write(80);
  motor.writeMicroseconds(1500);

  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");

  // uncomment once to get mag offsets, paste above, then comment out again
  //calibrateMag();

  startupCalibration();
}

void loop() {
  // CMD Processing
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == SOC) {
      inCommand = true;
      lineLen = 0;
    }
    else if (inCommand) {
      if (c == '\n') {
        lineBuf[lineLen] = '\0';  // Null-terminate command
        processCommand(lineBuf, heading);
        inCommand = false;
        lineLen = 0;
      } else if (lineLen < MAX_LINE) {
        lineBuf[lineLen++] = c;  // Append char to command buffer
      } else {
        // Command too long
        inCommand = false;
        lineLen = 0;
      }
    }
  }

  //IMU Background Processing
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    z -= gbz;

    if (abs(z) < 0.1) z = 0.0;  // ignores insignificant gyro readings/noise
      
    unsigned long now = micros();
    float dt = (now-lastTime)*1e-6f;  // calc dt (elapsed time)
    lastTime = now;

    heading -= dt * z;  // integrate gyro's angular velocity to get heading
  }
}
