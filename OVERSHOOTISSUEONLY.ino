#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <ModbusRTUSlave.h>
#include <PID_v1.h>

#define MAX31865_MISO 19
#define MAX31865_MOSI 23
#define MAX31865_CLK 18
#define MAX31865_CS 5

#define MAX485_DE 32
#define MAX485_RE 33
#define MAX485_RX 16
#define MAX485_TX 17

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

ModbusRTUSlave modbus_slave(Serial1);
const uint8_t slaveID = 1;
const uint32_t baud = 9600;
uint16_t holdingRegisters[5] = {0};  // Modbus holding registers

const char* ssid = "equichem_5";
const char* password = "equichem@2023";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int daylightOffset_sec = 0;

#define R0 100.0
#define Rref 430.0

#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

#define SSR_PIN 22  // Solid State Relay control pin

double setpoint = 55.0;  // Setpoint

#define totalReadingsMAX31865 2
double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

unsigned long lastReadingTime = 0;
unsigned long lastControlCycleTime = 0;
const long readingInterval = 500;  // Interval between readings (500ms)
const long controlCycleInterval = 5000;  // Control cycle time (5000ms)

float previousEMA = 0.0;
const float alpha = 0.1;  // EMA smoothing factor

// PID control variables
double inputTemperature, output;
double Kp = 0.8, Ki = 0.1, Kd = 1.6;  // Normal PID parameters
PID myPID(&inputTemperature, &output, &setpoint, Kp, Ki, Kd, DIRECT);  // PID controller initialization

// Aggressive PID tuning for overshoot mode
double gaP = 1.2, gaI = 0.05, gaD = 2.0;  

// Integral windup protection variables
double prevError = 0.0;  // To track error history for windup
double integral = 0.0;  // Accumulated error for integral term
double maxIntegral = 50.0;  // Maximum integral value to prevent windup

float aggressiveHeatingThreshold = 5.0;  // Trigger aggressive heating when the temperature is below 5°C from the setpoint

void setup() {
  Serial.begin(115200);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);  // SSR is off initially

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected.");

  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("Time synchronized");
      break;
    } else {
      Serial.println("Failed to get time, retrying...");
      delay(1000);  // Retry after 1 second
    }
  }

  // Initialize MAX31865
  thermo.begin(MAX31865_3WIRE);

  // Initialize PID control
  myPID.SetMode(AUTOMATIC);  // Start PID control
  myPID.SetOutputLimits(0, 255);  // Limit PID output to control SSR (0-255)

  // Initialize Modbus RTU (RS-485)
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);  // Set DE low for receiver mode
  digitalWrite(MAX485_RE, HIGH); // Set RE high for receiver mode
  Serial1.begin(baud, SERIAL_8N1, MAX485_RX, MAX485_TX);

  // Configure Modbus slave
  modbus_slave.configureHoldingRegisters(holdingRegisters, 5);
  modbus_slave.begin(slaveID, baud, SERIAL_8N1);

  Serial.println("MAX31865 Temperature Sensor Initialized");
}

void loop() {
  // Temperature reading every 500ms
  if (millis() - lastReadingTime >= readingInterval) {
    lastReadingTime = millis();  // Update reading time

    // Get current time for time-stamping
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      unsigned long ms = millis() % 1000;  // Get current milliseconds
      char formattedTime[20];
      snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

      Serial.print(formattedTime);  // Print formatted time
      Serial.print(" , ");

      // Read from MAX31865 sensor (multiple readings for smoothing)
      for (int i = 0; i < totalReadingsMAX31865; i++) {
        uint16_t adcCode = thermo.readRTD();
        float Rt = (adcCode * Rref) / 32768.0;
        temperatureReadingsMAX31865[i] = Rt;
      }

      // Calculate average temperature
      double tempSumMax31865 = 0;
      for (int i = 0; i < totalReadingsMAX31865; i++) {
        tempSumMax31865 += temperatureReadingsMAX31865[i];
      }
      float averagedRtMax31865 = tempSumMax31865 / totalReadingsMAX31865;
      float temperatureMax31865 = calculateTemperature(averagedRtMax31865);

      // Apply Exponential Moving Average (EMA)
      previousEMA = alpha * temperatureMax31865 + (1 - alpha) * previousEMA;

      // Apply the -0.1°C offset after EMA
      float adjustedTemperature = previousEMA - 0.1;

      // Update Modbus holding registers with the adjusted temperature
      holdingRegisters[4] = (uint16_t)(adjustedTemperature * 10);  // Modbus register holds value in tenths of a degree
      
      // Update time values in Modbus registers
      holdingRegisters[0] = timeinfo.tm_hour;  
      holdingRegisters[1] = timeinfo.tm_min;   
      holdingRegisters[2] = timeinfo.tm_sec; 
      holdingRegisters[3] = ms;  // Milliseconds

      Serial.print("Temperature: ");
      Serial.print(previousEMA, 2);
      Serial.print("°C,  ");

       Serial.print("Temperature: ");
      Serial.print(previousEMA, 1);
      Serial.print("°C,  ");

      Serial.print("Temperature (EMA - 0.1°C): ");
      Serial.print(adjustedTemperature, 1);
      Serial.println("°C");

      // Transmit Modbus data
      transmitMode();
      modbus_slave.poll();  // Poll Modbus slave
      receiveMode();  // Switch back to receive mode
    } else {
      Serial.println("Failed to obtain time");
    }
  }

  // Control loop every 5000ms
  if (millis() - lastControlCycleTime >= controlCycleInterval) {
    lastControlCycleTime = millis();  // Update control cycle time

    // Update PID control input with the most recent temperature reading
    inputTemperature = previousEMA;

    // Check if the temperature is more than 5°C below the setpoint for aggressive heating
    if (setpoint - inputTemperature > aggressiveHeatingThreshold) {
      // Use aggressive heating mode (more aggressive PID tuning)
      myPID.SetTunings(gaP, gaI, gaD);  // Use aggressive PID parameters
      // Reset integral if the temperature is far from the setpoint
      integral = 0;
    } else {
      // Normal PID control
      myPID.SetTunings(Kp, Ki, Kd);  // Use normal PID parameters
    }

    // Prevent integral windup: Update integral only if error is small or if we're not in aggressive mode
    double error = setpoint - inputTemperature;
    if (abs(error) < aggressiveHeatingThreshold) {
      integral += error;  // Accumulate the error for the integral term
      if (integral > maxIntegral) integral = maxIntegral;  // Prevent integral windup
    } else {
      integral = 0;  // Reset integral if large error exists
    }

    // Compute the PID output
    myPID.Compute();

     // Print PID output value to Serial Monitor
    Serial.print("PID Output: ");
    Serial.println(output);

    // Control SSR based on PID output
    if (output > 0) {
      digitalWrite(SSR_PIN, HIGH);  // Turn heater ON
      Serial.println("Heater: ON");
    } else {
      digitalWrite(SSR_PIN, LOW);   // Turn heater OFF
      Serial.println("Heater: OFF");
    }
  }
}

// Calculate temperature from RTD resistance
float calculateTemperature(float Rt) {
  float t;
  if (Rt >= R0) {
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    float tolerance = 0.001;
    int maxIterations = 100;
    int iteration = 0;
    float diff;
    t = Rt;

    do {
      float fValue = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - Rt;
      float fDerivative = R0 * (A + 2 * B * t + 3 * C * (t - 100) * t * t + C * t * t * t);
      float nextT = t - fValue / fDerivative;
      diff = abs(nextT - t);
      t = nextT;
      iteration++;
    } while (diff > tolerance && iteration < maxIterations);

    if (iteration == maxIterations) {
      Serial.println("Warning: Temperature calculation failed to converge.");
    }
  }
  return t;
}

void transmitMode() {
  digitalWrite(MAX485_DE, HIGH);  // Enable Driver (Transmit mode)
  digitalWrite(MAX485_RE, HIGH);  // Disable Receiver
}

void receiveMode() {
  digitalWrite(MAX485_DE, LOW);   // Disable Driver (Receive mode)
  digitalWrite(MAX485_RE, LOW);   // Enable Receiver
}
