#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>

static const size_t MAX_LINE = 16; // Max command line length, excluding EOL
static char lineBuf[MAX_LINE+1];
static size_t lineLen = 0;
static const char SOC = '$'; // start of command character
static bool inCommand = false;
Servo servo;

void setup() {
  Serial.begin(112500);

}

static void processCommand(char* cmd) {
  if (!cmd || cmd[0] == '\0') return;

  char* cr = strchr(cmd, '\r');
  if (cr) *cr = '\0';
  
  while (*cmd && isspace((unsigned char)*cmd)) cmd++;
  if (*cmd == '\0') return;
}

void loop() {
  // put your main code here, to run repeatedly:

}
