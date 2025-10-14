#include <EEPROM.h>
#include <math.h>

#define NUM_CHANNELS 4
#define CAL_DEFAULT 2047
#define CURR_DEFAULT 1.0
#define OVER_DEFAULT 1000 // Default: 1000 ms over threshold
#define SERIAL_BUFFER_SIZE 32
#define ADC_SIGLE_READ_AVERAGE_SAMPLES 5 // Number of single ADC reads in readAverage
#define ADC_SAMPLE_INTERVAL 20 // Time in ms between ADC samples in main loop
#define SERIAL_SEND_INTERVAL 500 // Time in ms between sending current values over serial
#define EMA_ALPHA 0.1 // Smoothing factor for EMA (0 < EMA_ALPHA <= 1)
#define OVERHOLD_DEFAULT 0 // Default: 0 ms to keep thresholdExceeded after current drops below threshold

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
const int EEPROM_OVERHOLD_ADDR = EEPROM_OVER_ADDR + sizeof(unsigned long) * NUM_CHANNELS;
const int EEPROM_FLAG_ADDR = EEPROM_OVERHOLD_ADDR + sizeof(unsigned long);
const byte EEPROM_INIT_FLAG = 0xA6;

unsigned long overThresholdHoldTime = OVERHOLD_DEFAULT; // ms
unsigned long overHoldStart[NUM_CHANNELS] = {0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
  delay(300);
  loadFromEEPROM();

  // Set D2-D5 as outputs
  for (int pin = 2; pin <= 11; pin++) {
    pinMode(pin, OUTPUT);
  }
  //calibrateADC(); // Perform calibration at startup
}

int readAverage(int pin, int samples = ADC_SIGLE_READ_AVERAGE_SAMPLES) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return sum / samples;
}

void calibrateADC() {
  const unsigned long calibrationTime = 2000; // 1 second
  unsigned long startTime = millis();
  long sum[NUM_CHANNELS] = {0};
  int samples = 0;

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      sum[i] += readAverage(i);
    }
    samples++;
    delay(20);
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
    Serial.print("ACK\r\n");
    int ch = cmd[1] - '0';
    if (ch >= 0 && ch < NUM_CHANNELS) {
      float val = atof(cmd + 3);
      currentThreshold[ch] = val;
      saveToEEPROM();
      Serial.print("Threshold for channel ");
      Serial.print(ch);
      Serial.print(" set to ");
      Serial.println(val, 3);
    }
  }
  // Example: T0=1500 (set overThresholdTime for channel 0 to 1500 ms)
  else if (cmd[0] == 'T' && cmd[2] == '=') {
    Serial.print("ACK\r\n");
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
    }
  }
  // Example: H=500 (set overThresholdHoldTime for all channels to 500 ms)
  if (cmd[0] == 'H' && cmd[1] == '=') {
    Serial.print("ACK\r\n");
    unsigned long val = atol(cmd + 2);
    overThresholdHoldTime = val;
    saveToEEPROM();
    Serial.print("Over-threshold hold time set to ");
    Serial.print(val);
    Serial.println(" ms");
  }
  // Calibration command: CAL
  else if (strcmp(cmd, "CAL") == 0) {
    Serial.print("ACK\r\n");
    calibrateADC();
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
    EEPROM.get(EEPROM_OVERHOLD_ADDR, overThresholdHoldTime);
  } else {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      calibration[i] = CAL_DEFAULT;
      currentThreshold[i] = CURR_DEFAULT;
      overThresholdTime[i] = OVER_DEFAULT;
    }
    overThresholdHoldTime = OVERHOLD_DEFAULT;
    saveToEEPROM();
  }
}

void saveToEEPROM() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    EEPROM.put(EEPROM_CALIB_ADDR + i * sizeof(int), calibration[i]);
    EEPROM.put(EEPROM_THRESH_ADDR + i * sizeof(float), currentThreshold[i]);
    EEPROM.put(EEPROM_OVER_ADDR + i * sizeof(unsigned long), overThresholdTime[i]);
  }
  EEPROM.put(EEPROM_OVERHOLD_ADDR, overThresholdHoldTime);
  EEPROM.write(EEPROM_FLAG_ADDR, EEPROM_INIT_FLAG);
}

void loop() {
  static unsigned long lastSampleTime = 0;
  static unsigned long lastSerialSendTime = 0;
  static float currents[NUM_CHANNELS];
  int adcRawReadout = 0;
  int adcCalibratedReadout = 0;
  float adcCurrent = 0;

  unsigned long now = millis();
  checkSerialCommand();

  if (now - lastSampleTime >= ADC_SAMPLE_INTERVAL) {
    lastSampleTime = now;

    // Get actual currents
    for (int i = 0; i < NUM_CHANNELS; i++) {
      adcRawReadout = readAverage(i);
      adcCalibratedReadout = adcRawReadout - calibration[i];
      adcCurrent = getCurrent(adcCalibratedReadout);
      currents[i] = EMA_ALPHA * adcCurrent + (1.0 - EMA_ALPHA) * currents[i];
    }

    // Threshold logic (use absolute value of current)
    for (int i = 0; i < NUM_CHANNELS; i++) {
      float absCurrent = fabs(currents[i]);
      if (absCurrent > currentThreshold[i]) {
        // Over threshold logic
        if (!thresholdExceeded[i]) {
          if (overStart[i] == 0) overStart[i] = now;
          if (now - overStart[i] >= overThresholdTime[i]) {
            thresholdExceeded[i] = true;
            overStart[i] = 0;
            overHoldStart[i] = 0;
          }
        } else {
          overStart[i] = 0;
          overHoldStart[i] = 0;
        }
      } else {
        // Under threshold logic
        if (thresholdExceeded[i]) {
          if (overHoldStart[i] == 0) overHoldStart[i] = now;
          if (now - overHoldStart[i] >= overThresholdHoldTime) {
            thresholdExceeded[i] = false;
            overHoldStart[i] = 0;
            overStart[i] = 0;
          }
        } else {
          overHoldStart[i] = 0;
          overStart[i] = 0;
        }
      }
    }

    // Update output pins based on thresholdExceeded states
    for (int i = 0; i < NUM_CHANNELS; i++) {
      digitalWrite(2 + i, thresholdExceeded[i] ? LOW : HIGH);
      digitalWrite(8 + i, thresholdExceeded[i] ? HIGH : LOW);
    }
  }



  // Serial print actual current values
  if (now - lastSerialSendTime >= SERIAL_SEND_INTERVAL) {
    lastSerialSendTime = now;

    Serial.print("CUR: ");
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (thresholdExceeded[i]) {
        Serial.print("!");
      }
      Serial.print(currents[i], 3);
      if (i < NUM_CHANNELS - 1) Serial.print(", ");
    }
    Serial.println();
  }

  delay(10);
}
