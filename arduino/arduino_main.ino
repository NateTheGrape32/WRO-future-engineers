#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>
#include <Arduino_BMI270_BMM150.h>

static float q0=1, q1=0, q2=0, q3=0;
static float eIx=0, eIy=0, eIz=0;
static unsigned long lastTime = 0;
static float gbx=0, gby=0, gbz=0;

// reference quaternion everything is relative to this after zeroing
static float r0=1, r1=0, r2=0, r3=0;

// cached mag updated only when new data arrives, used every frame
static float cachedMx=0, cachedMy=0, cachedMz=0;

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

#define DECLINATION_DEG  -10.1f

// This need to be calibrated board to board. Please uncomment calibrateMag() under void loop to do so.
#define MAG_X_OFFSET  39.0f
#define MAG_Y_OFFSET  30.0f
#define MAG_Z_OFFSET  13.5f

// Kp = how fast it corrects, Ki = slowly kills gyro bias
#define Kp  2.0f
#define Ki  0.05f

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

// run once, rotate board in all directions 30s, paste offsets above
void calibrateMag() {
  Serial.println("Rotate board in all directions for 30s...");
  float xmin=1e9,xmax=-1e9,ymin=1e9,ymax=-1e9,zmin=1e9,zmax=-1e9;
  unsigned long start = millis();
  while (millis()-start < 30000) {
    if (!IMU.magneticFieldAvailable()) continue;
    float mx,my,mz;
    IMU.readMagneticField(mx,my,mz);
    xmin=min(xmin,mx); xmax=max(xmax,mx);
    ymin=min(ymin,my); ymax=max(ymax,my);
    zmin=min(zmin,mz); zmax=max(zmax,mz);
    delay(10);
  }
  Serial.println("Done! Paste these into your #defines:");
  Serial.print("#define MAG_X_OFFSET  "); Serial.println((xmin+xmax)/2, 4);
  Serial.print("#define MAG_Y_OFFSET  "); Serial.println((ymin+ymax)/2, 4);
  Serial.print("#define MAG_Z_OFFSET  "); Serial.println((zmin+zmax)/2, 4);
  while(1);
}

void mahonyUpdate(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float mx, float my, float mz, float dt) {
  float norm;

  norm = sqrtf(ax*ax+ay*ay+az*az);
  if (norm == 0) return;
  ax/=norm; ay/=norm; az/=norm;

  norm = sqrtf(mx*mx+my*my+mz*mz);
  if (norm == 0) return;
  mx/=norm; my/=norm; mz/=norm;

  float hx = 2*(mx*(0.5f-q2*q2-q3*q3) + my*(q1*q2-q0*q3) + mz*(q1*q3+q0*q2));
  float hy = 2*(mx*(q1*q2+q0*q3) + my*(0.5f-q1*q1-q3*q3) + mz*(q2*q3-q0*q1));
  float bx = sqrtf(hx*hx+hy*hy);
  float bz = 2*(mx*(q1*q3-q0*q2) + my*(q2*q3+q0*q1) + mz*(0.5f-q1*q1-q2*q2));

  float vx = 2*(q1*q3-q0*q2);
  float vy = 2*(q0*q1+q2*q3);
  float vz = q0*q0-q1*q1-q2*q2+q3*q3;
  float wx = 2*(bx*(0.5f-q2*q2-q3*q3) + bz*(q1*q3-q0*q2));
  float wy = 2*(bx*(q1*q2-q0*q3)       + bz*(q0*q1+q2*q3));
  float wz = 2*(bx*(q0*q2+q1*q3)       + bz*(0.5f-q1*q1-q2*q2));

  float ex = (ay*vz-az*vy) + (my*wz-mz*wy);
  float ey = (az*vx-ax*vz) + (mz*wx-mx*wz);
  float ez = (ax*vy-ay*vx) + (mx*wy-my*wx);

  eIx += Ki*ex*dt;
  eIy += Ki*ey*dt;
  eIz += Ki*ez*dt;

  gx += Kp*ex + eIx;
  gy += Kp*ey + eIy;
  gz += Kp*ez + eIz;

  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  float dq0 = 0.5f*(-q1*gx-q2*gy-q3*gz)*dt;
  float dq1 = 0.5f*( q0*gx+q2*gz-q3*gy)*dt;
  float dq2 = 0.5f*( q0*gy-q1*gz+q3*gx)*dt;
  float dq3 = 0.5f*( q0*gz+q1*gy-q2*gx)*dt;

  q0+=dq0; q1+=dq1; q2+=dq2; q3+=dq3;

  norm = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;
}

void readAndFuse(float dt) {
  // update mag cache only when fresh data is ready (it's slow ~10Hz)
  if (IMU.magneticFieldAvailable()) {
    float mx,my,mz;
    IMU.readMagneticField(mx,my,mz);
    // BMI270 rotated 90deg on board + align to same frame
    cachedMx =  my - MAG_X_OFFSET;
    cachedMy = -mx - MAG_Y_OFFSET;
    cachedMz = -mz - MAG_Z_OFFSET;
  }

  // always run filter at full speed with last known mag
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  float ax,ay,az,gx,gy,gz;
  IMU.readAcceleration(ax,ay,az);
  IMU.readGyroscope(gx,gy,gz);
  gx -= gbx; gy -= gby; gz -= gbz;

  float rax=-ax, ray=ay, raz=az;
  float rgx=-gx, rgy=gy, rgz=gz;

  mahonyUpdate(rax,ray,raz, rgx,rgy,rgz, cachedMx,cachedMy,cachedMz, dt);
}

void warmupAndZero() {
  Serial.println("Warming up — keep still...");

  // seed the mag cache before warmup starts
  while (!IMU.magneticFieldAvailable());
  float mx,my,mz;
  IMU.readMagneticField(mx,my,mz);
  cachedMx =  my - MAG_X_OFFSET;
  cachedMy = -mx - MAG_Y_OFFSET;
  cachedMz = -mz - MAG_Z_OFFSET;

  unsigned long start = millis();
  lastTime = micros();
  while (millis()-start < 3000) {
    unsigned long now = micros();
    float dt = (now-lastTime)*1e-6f;
    if (dt < 0.002f) continue;
    lastTime = now;
    if (dt > 0.05f) dt = 0.05f;
    readAndFuse(dt);
  }

  // store inverse of settled quaternion so all angles are relative to startup pose
  r0= q0; r1=-q1; r2=-q2; r3=-q3;
  Serial.println("Zeroed. Go!");
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
      if (val < 0 || val > 180) return;  // Invalid angle
      servo.write(val);
      break;
    case 'M':  // return IMU data and turn
      if (val < 1000 || val > 2000) return;  // Invalid speed
      motor.writeMicroseconds(val);
      break;
    case 'I':
      unsigned long now = micros();
      float dt = (now-lastTime)*1e-6f;
      if (dt < 0.002f) return;
      lastTime = now;
      if (dt > 0.05f) dt = 0.05f;

      readAndFuse(dt);

      // rotate by reference quaternion so output is relative to startup pose
      float rq0 = r0*q0 - r1*q1 - r2*q2 - r3*q3;
      float rq1 = r0*q1 + r1*q0 + r2*q3 - r3*q2;
      float rq2 = r0*q2 - r1*q3 + r2*q0 + r3*q1;
      float rq3 = r0*q3 + r1*q2 - r2*q1 + r3*q0;

      float roll    = atan2f(2*(rq0*rq1+rq2*rq3), 1-2*(rq1*rq1+rq2*rq2)) * RAD_TO_DEG;
      float pitch   = asinf (constrain(2*(rq0*rq2-rq3*rq1), -1.0f, 1.0f)) * RAD_TO_DEG;
      float heading = atan2f(2*(rq0*rq3+rq1*rq2), 1-2*(rq2*rq2+rq3*rq3)) * RAD_TO_DEG;

      // Y axis = forward
      heading += DECLINATION_DEG - 90.0f;
      if (heading < 0)   heading += 360.0f;
      if (heading > 360) heading -= 360.0f;
      Serial.println(heading + 90.0f, 4)
      break;
    default:
      // Ignore unknown command type
      break;
  }
}

void setup() {
  servo.attach(SERVO_PIN, 900, 2100);
  motor.attach(MOTOR_PIN, 1000, 2000);
  servo.write(90);
  motor.writeMicroseconds(1500);

  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");

  // uncomment once to get mag offsets, paste above, then comment out again
  calibrateMag();

  calibrateGyro();
  warmupAndZero();
  lastTime = micros();
}

void loop() {
  // CMD Processing
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == SOC) {
      inCommand = true;
      lineLen = 0;
      continue;
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
