#include <EEPROM.h>
#include <math.h>

#define NUM_CHANNELS 4
#define CAL_DEFAULT 2047
#define CURR_DEFAULT 1.0
#define OVER_DEFAULT 1000 // Default: 1000 ms over threshold
#define SERIAL_BUFFER_SIZE 32

int calibration[NUM_CHANNELS] = {CAL_DEFAULT, CAL_DEFAULT, CAL_DEFAULT, CAL_DEFAULT};
float currentThreshold[NUM_CHANNELS] = {CURR_DEFAULT, CURR_DEFAULT, CURR_DEFAULT, CURR_DEFAULT};
unsigned long overThresholdTime[NUM_CHANNELS] = {OVER_DEFAULT, OVER_DEFAULT, OVER_DEFAULT, OVER_DEFAULT}; // ms

unsigned long overStart[NUM_CHANNELS] = {0, 0, 0, 0};
bool thresholdExceeded[NUM_CHANNELS] = {false, false, false, false};

char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialIndex = 0;

// EEPROM addresses
const int EEPROM_CALIB_ADDR = 0;
const int EEPROM_THRESH_ADDR = EEPROM_CALIB_ADDR + sizeof(int) * NUM_CHANNELS;
const int EEPROM_OVER_ADDR = EEPROM_THRESH_ADDR + sizeof(float) * NUM_CHANNELS;
const int EEPROM_FLAG_ADDR = EEPROM_OVER_ADDR + sizeof(unsigned long) * NUM_CHANNELS;
const byte EEPROM_INIT_FLAG = 0xA6;

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
  delay(300);
  loadFromEEPROM();
  //calibrateADC(); // Perform calibration at startup
}

void calibrateADC() {
  const unsigned long calibrationTime = 1000; // 1 second
  unsigned long startTime = millis();
  long sum[NUM_CHANNELS] = {0};
  int samples = 0;

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      sum[i] += analogRead(i);
    }
    samples++;
    delay(1); // Small delay to avoid flooding
  }

  for (int i = 0; i < NUM_CHANNELS; i++) {
    calibration[i] = sum[i] / samples;
  }

  saveToEEPROM();

  Serial.print("Calibration values: ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(calibration[i]);
    if (i < NUM_CHANNELS - 1) Serial.print(", ");
  }
  Serial.println();
}

int readAverage(int pin, int samples = 32) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return sum / samples;
}

float getCurrent(int adcDiff, float vRef = 5.0, int adcResolution = 4096) {
  // Convert ADC difference to voltage
  float voltage = (adcDiff * vRef) / adcResolution;
  // 1A per 0.04V (40mV)
  float current = voltage / 0.04;
  return current;
}

void checkSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      serialBuffer[serialIndex] = '\0';
      if (serialIndex > 0) {
        parseCommand(serialBuffer);
        serialIndex = 0;
      }
    } else if (serialIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialIndex++] = c;
    }
  }
}

void parseCommand(const char* cmd) {
  // Example: C0=1.2
  if (cmd[0] == 'C' && cmd[2] == '=') {
    int ch = cmd[1] - '0';
    if (ch >= 0 && ch < NUM_CHANNELS) {
      float val = atof(cmd + 3);
      currentThreshold[ch] = val;
      saveToEEPROM();
      Serial.print("Threshold for channel ");
      Serial.print(ch);
      Serial.print(" set to ");
      Serial.println(val, 3);
      Serial.print("ACK\r\n");
    }
  }
  // Example: T0=1500 (set overThresholdTime for channel 0 to 1500 ms)
  else if (cmd[0] == 'T' && cmd[2] == '=') {
    int ch = cmd[1] - '0';
    if (ch >= 0 && ch < NUM_CHANNELS) {
      unsigned long val = atol(cmd + 3);
      overThresholdTime[ch] = val;
      saveToEEPROM();
      Serial.print("Over-threshold time for channel ");
      Serial.print(ch);
      Serial.print(" set to ");
      Serial.print(val);
      Serial.println(" ms");
      Serial.print("ACK\r\n");
    }
  }
  // Calibration command: CAL
  else if (strcmp(cmd, "CAL") == 0) {
    calibrateADC();
    Serial.print("ACK\r\n");
  }
}

void loadFromEEPROM() {
  byte flag = EEPROM.read(EEPROM_FLAG_ADDR);
  if (flag == EEPROM_INIT_FLAG) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      EEPROM.get(EEPROM_CALIB_ADDR + i * sizeof(int), calibration[i]);
      EEPROM.get(EEPROM_THRESH_ADDR + i * sizeof(float), currentThreshold[i]);
      EEPROM.get(EEPROM_OVER_ADDR + i * sizeof(unsigned long), overThresholdTime[i]);
    }
  } else {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      calibration[i] = CAL_DEFAULT;
      currentThreshold[i] = CURR_DEFAULT;
      overThresholdTime[i] = OVER_DEFAULT;
    }
    saveToEEPROM();
  }
}

void saveToEEPROM() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    EEPROM.put(EEPROM_CALIB_ADDR + i * sizeof(int), calibration[i]);
    EEPROM.put(EEPROM_THRESH_ADDR + i * sizeof(float), currentThreshold[i]);
    EEPROM.put(EEPROM_OVER_ADDR + i * sizeof(unsigned long), overThresholdTime[i]);
  }
  EEPROM.write(EEPROM_FLAG_ADDR, EEPROM_INIT_FLAG);
}

void loop() {
  checkSerialCommand();
  int values[NUM_CHANNELS];
  int diff[NUM_CHANNELS];
  float currents[NUM_CHANNELS];
  unsigned long now = millis();

  for (int i = 0; i < NUM_CHANNELS; i++) {
    values[i] = readAverage(i);
    diff[i] = values[i] - calibration[i];
    currents[i] = getCurrent(diff[i]);
  }

  // Threshold logic (use absolute value of current)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    float absCurrent = fabs(currents[i]);
    if (absCurrent > currentThreshold[i]) {
      if (!thresholdExceeded[i]) {
        if (overStart[i] == 0) overStart[i] = now;
        if (now - overStart[i] >= overThresholdTime[i]) {
          thresholdExceeded[i] = true;
          overStart[i] = 0; // Reset for next under-threshold timing
        }
      } else {
        overStart[i] = 0; // Reset under-threshold timer if flag is set and still over threshold
      }
    } else {
      if (thresholdExceeded[i]) {
        if (overStart[i] == 0) overStart[i] = now;
        if (now - overStart[i] >= overThresholdTime[i]) {
          thresholdExceeded[i] = false;
          overStart[i] = 0; // Reset for next over-threshold timing
        }
      } else {
        overStart[i] = 0; // Not over threshold, flag not set, keep timer reset
      }
    }
  }

  // Send current values as CSV
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (thresholdExceeded[i]) {
      Serial.print("!");
    }
    Serial.print(currents[i], 3);
    if (i < NUM_CHANNELS - 1) Serial.print(", ");
  }
  Serial.println();

  delay(500);
}
