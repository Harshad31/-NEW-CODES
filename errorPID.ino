#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <PID_v1.h>  

#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

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

#define SSR_PIN 35  // Pin for SSR 

double setpoint = 37.0;  // Setpoint 

const int totalReadingsMAX31865 = 2;
double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

unsigned long lastReadingTime = 0;
const long readingInterval = 500;   // Interval between readings (500ms)

unsigned long lastControlCycleTime = 0;  // To store last control cycle time
const long controlCycleInterval = 5000;  // Control cycle

// PID variables
double inputTemperature, output;
double Kp = 1.0, Ki = 0.5, Kd = 0.5;  // PID parameters

PID myPID(&inputTemperature, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

void setup() {
  // Start serial communication
  Serial.begin(115200);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);  

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {  
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
  } else {
    Serial.println("Failed to connect to WiFi.");
  }

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("Time synchronized");
      break;
    } else {
      Serial.println("Failed to get time, retrying...");
      delay(1000); // Retry after 1 second
    }
  }

  thermo.begin(MAX31865_3WIRE);
  Serial.println("MAX31865 Temperature Sensor Initialized");

  myPID.SetMode(AUTOMATIC);  // Enable PID controller
  myPID.SetOutputLimits(0, 255);  // Limit output to control SSR (0-255)
}

void loop() {
  unsigned long startTime = millis();

  // Get current time
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    unsigned long ms = millis() % 1000;  // Milliseconds

    // Format time as HH:MM:SS:MS
    char formattedTime[20];
    snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

    // Start constructing the output string
    String outputString = String(formattedTime) + " , ";

    for (int i = 0; i < totalReadingsMAX31865; i++) {
      uint16_t adcCode = thermo.readRTD();
      float Rt = (adcCode * Rref) / 32768.0;
      temperatureReadingsMAX31865[i] = Rt;
    }

    double tempSumMax31865 = 0;
    for (int i = 0; i < totalReadingsMAX31865; i++) {
      tempSumMax31865 += temperatureReadingsMAX31865[i];
    }
    float averagedRtMax31865 = tempSumMax31865 / totalReadingsMAX31865;
    float temperatureMax31865 = calculateTemperature(averagedRtMax31865);

    // Add temperature to the output string
    outputString += String(temperatureMax31865, 2) + "°C, ";

    // Update PID input
    inputTemperature = temperatureMax31865;

    // Check if it's time for the control cycle (every 3000ms)
    unsigned long currentMillis = millis();
    if (currentMillis - lastControlCycleTime >= controlCycleInterval) {
      lastControlCycleTime = currentMillis;  // Update the last control cycle time

      // Compute PID output
      myPID.Compute();

      // Control the SSR based on PID output
      if (output > 0) {
        digitalWrite(SSR_PIN, HIGH);  // Turn heater ON 
        outputString += "Heater: ON, ";
      } else {
        digitalWrite(SSR_PIN, LOW);   // Turn heater OFF 
        outputString += "Heater: OFF, ";
      }
    }

    // Print the complete one-line output
    Serial.println(outputString);

    // Delay to maintain reading interval
    unsigned long endTime = millis();
    unsigned long timeTakenForReadings = endTime - startTime;
    unsigned long remainingSleepTime = readingInterval - timeTakenForReadings;
    if (remainingSleepTime > 0) {
      delay(remainingSleepTime);
    }
  } else {
    Serial.println("Failed to obtain time");
  }
}

// Function to calculate temperature from MAX31865 RTD value
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
