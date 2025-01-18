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
uint16_t holdingRegisters[20] = {0};  // Modbus holding registers

const char* ssid = "VIRAT";
const char* password = "12345678";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int daylightOffset_sec = 0;

#define R0 100.0
#define Rref 430.0
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

#define SSR_PIN 22  // Solid State Relay (SSR) control pin

double setpoint = 39.0;  //  setpoint

#define totalReadingsMAX31865 2
double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

// Timing variables for sampling rate
unsigned long totalReadingTime = 0;
unsigned long totalSleepTime = 0;
int readingCount = 0;
unsigned long lastReadingTime = 0;
const long readingInterval = 500;  // Interval between readings (500ms)

// EMA (Exponential Moving Average) variables
float previousEMA = 0.0;
const float alpha = 0.1;  // Smoothing factor for EMA

// PID control variables
double inputTemperature, output;
double Kp = 1.0, Ki = 0.5, Kd = 0.5;  // Normal PID parameters
PID myPID(&inputTemperature, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

boolean overShootMode = false;
double gOvershoot = 1.0;  // Temperature threshold for overshoot mode (1°C)
double gaP = 1.5, gaI = 1.0, gaD = 0.7;  // Aggressive PID parameters for overshoot mode
double gP = Kp, gI = Ki, gD = Kd;  // Revert to normal PID parameters

void setup() {
  Serial.begin(115200);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected.");

  // Initialize NTP time synchronization
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
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Limit PID output to control SSR (0-255)

  // Set up Modbus RTU communication (RS-485)
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);  // Set DE low for receiver mode
  digitalWrite(MAX485_RE, HIGH); // Set RE high for receiver mode
  Serial1.begin(baud, SERIAL_8N1, MAX485_RX, MAX485_TX);

  // Configure Modbus slave
  modbus_slave.configureHoldingRegisters(holdingRegisters, 20);
  modbus_slave.begin(slaveID, baud, SERIAL_8N1);

  Serial.println("MAX31865 Temperature Sensor Initialized");
}

void loop() {
  unsigned long startTime = millis();

  // Get current time for time-stamping
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    unsigned long ms = millis() % 1000;  // Get the current milliseconds
    char formattedTime[20];
    snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

    Serial.print(formattedTime); // Print formatted time
    Serial.print(" , ");

    // Update Modbus holding registers with time values
    holdingRegisters[0] = timeinfo.tm_hour;  
    holdingRegisters[1] = timeinfo.tm_min;   
    holdingRegisters[2] = timeinfo.tm_sec; 
    holdingRegisters[3] = ms;              

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

    // Update PID control input with the new temperature
    inputTemperature = temperatureMax31865;

    // Switch to aggressive PID if overshoot mode is needed
    if (abs(setpoint - inputTemperature) >= gOvershoot) {
      if (!overShootMode) {
        myPID.SetTunings(gaP, gaI, gaD);  // Use aggressive PID parameters
        overShootMode = true;
      }
    } else {
      if (overShootMode) {
        myPID.SetTunings(gP, gI, gD);  // Use normal PID parameters
        overShootMode = false;
      }
    }

    // Apply Exponential Moving Average (EMA) to smooth temperature readings
    if (readingCount == 0) {
      previousEMA = temperatureMax31865;
    } else {
      previousEMA = alpha * temperatureMax31865 + (1 - alpha) * previousEMA;  // EMA calculation
    }

    // Update Modbus holding registers with the smoothed temperature
    holdingRegisters[4] = (uint16_t)(previousEMA * 10);

    // Transmit mode for Modbus communication
    transmitMode();
    modbus_slave.poll();  // Poll Modbus slave for communication
    receiveMode();  // Switch back to receive mode

    // Compute PID output
    myPID.Compute();

    // Control SSR based on PID output (heater control)
    if (output > 0) {
      digitalWrite(SSR_PIN, HIGH);  // Turn heater ON
    } else {
      digitalWrite(SSR_PIN, LOW);   // Turn heater OFF
    }

    // Print temperature and SSR status to serial
    Serial.print("Temperature: ");
    Serial.print(previousEMA, 2);
    Serial.print("°C, ");
    Serial.print("Heater: ");
    Serial.println(output > 0 ? "ON" : "OFF");

    // Calculate time for reading and sleep to maintain the 500ms interval
    unsigned long endTime = millis();
    unsigned long timeTakenForReadings = endTime - startTime;
    totalReadingTime += timeTakenForReadings;
    readingCount++;

    unsigned long remainingSleepTime = readingInterval - timeTakenForReadings;
    if (remainingSleepTime > 0) {
      delay(remainingSleepTime);  // Sleep to maintain 500ms interval
      totalSleepTime += remainingSleepTime;
    }
  } else {
    Serial.println("Failed to obtain time");
  }
}

// Function to calculate temperature from RTD resistance using Callendar-Van Dusen equation
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

// Modbus mode switch functions for RS-485 communication
void transmitMode() {
  digitalWrite(MAX485_DE, HIGH);  // Enable Driver (Transmit mode)
  digitalWrite(MAX485_RE, HIGH);  // Disable Receiver
}

void receiveMode() {
  digitalWrite(MAX485_DE, LOW);   // Disable Driver (Receive mode)
  digitalWrite(MAX485_RE, LOW);   // Enable Receiver
}
