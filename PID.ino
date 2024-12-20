#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

// Wi-Fi Credentials
const char* ssid = "VIRAT";
const char* password = "12345678";

// MAX31865 Configuration
#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

// PID Configuration
double currentTemp = 0.0;    // Current temperature
double setpoint = 100.0;     // Desired temperature (setpoint)
double outputPower = 0.0;    // PID output to control SSR

double Kp = 2.0, Ki = 0.5, Kd = 1.0; // PID constants (to be tuned)
PID tempPID(&currentTemp, &outputPower, &setpoint, Kp, Ki, Kd, DIRECT);

// SSR Control
const int ssrPin = 5;          // SSR control pin
const int ssrCycleDuration = 1000; // SSR cycle duration in milliseconds
unsigned long ssrLastUpdate = 0;

const int totalReadings = 2;   // Number of readings for averaging
float alpha = 0.1;             // EMA smoothing factor
float previousEMA = 0.0;

// NTP Time Sync
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int daylightOffset_sec = 0;

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi Connected.");

  // Initialize MAX31865
  thermo.begin(MAX31865_3WIRE);
  Serial.println("MAX31865 Temperature Sensor Initialized");

  // Synchronize Time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("Time synchronized");
  } else {
    Serial.println("Failed to synchronize time.");
  }

  // Configure SSR Pin
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);

  // Initialize PID
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, ssrCycleDuration); // Limit output to match SSR cycle
  tempPID.SetSampleTime(200);                  // PID sample time in milliseconds
}

void loop() {
  // Read Temperature from MAX31865
  currentTemp = getSmoothedTemperature();

  // Compute PID
  tempPID.Compute();

  // Control SSR based on PID output
  unsigned long now = millis();
  if (now - ssrLastUpdate >= ssrCycleDuration) {
    ssrLastUpdate = now;
  }
  int onDuration = (int)(outputPower * ssrCycleDuration / 100.0); // Calculate ON duration as percentage
  if (now - ssrLastUpdate < onDuration) {
    digitalWrite(ssrPin, HIGH);
  } else {
    digitalWrite(ssrPin, LOW);
  }

  // Print to Serial Monitor
  Serial.print("Current Temp: ");
  Serial.print(currentTemp, 2);
  Serial.print("°C, Setpoint: ");
  Serial.print(setpoint);
  Serial.print("°C, Output Power: ");
  Serial.print(outputPower);
  Serial.println("%");

  delay(100);
}

// Function to read and smooth temperature using EMA
float getSmoothedTemperature() {
  uint16_t adcCode = thermo.readRTD();
  float Rt = (adcCode * 430.0) / 32768.0; // Resistance calculation
  float temp = calculateTemperature(Rt);

  if (previousEMA == 0.0) {
    previousEMA = temp;
  } else {
    previousEMA = alpha * temp + (1 - alpha) * previousEMA;
  }
  return previousEMA;
}

// Function to calculate temperature from resistance (Callendar-Van Dusen equation)
float calculateTemperature(float Rt) {
  const float R0 = 100.0; // Resistance at 0°C
  const float A = 3.9083e-3;
  const float B = -5.775e-7;
  const float C = -4.183e-12;

  float temp;
  if (Rt >= R0) {
    temp = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    float t = Rt; // Initial guess
    for (int i = 0; i < 100; i++) { // Iterative solution
      float f = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - Rt;
      float f_prime = R0 * (A + 2 * B * t + 3 * C * (t - 100) * t * t + C * t * t * t);
      t -= f / f_prime;
      if (abs(f) < 0.001) break;
    }
    temp = t;
  }
  return temp;
}
