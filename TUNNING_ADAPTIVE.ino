#include <Adafruit_MAX31865.h>
#include <WiFi.h>

#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

const char* ssid     = "VIRAT";
const char* password = "12345678";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int   daylightOffset_sec = 0;

#define R0 100.0
#define Rref 430.0

#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

const int totalReadingsMAX31865 = 2;
double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

unsigned long lastReadingTime = 0; 
const long readingInterval = 500;   // Interval between readings 

// PID Control variables
float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 1.0;  // Derivative gain

float lastError = 0.0;  // To store the previous error for derivative calculation
float integral = 0.0;   // To accumulate the integral term

float setPoint = 37.0;    

float output = 0.0;     // PID output, used to control SSR
int SSR_PIN = 35;       // Pin to control SSR 

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected.");

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Check if time is synchronized
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

  pinMode(SSR_PIN, OUTPUT);  // Initialize SSR control pin
  digitalWrite(SSR_PIN, LOW);  // Turn off initially
}

void loop() {
  unsigned long startTime = millis(); 

  // Get current time
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
   
    unsigned long ms = millis() % 1000;  // Calculate the milliseconds

    // Format time as HH:MM:SS:MS
    char formattedTime[20];
    snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

    Serial.print(formattedTime);  
    Serial.print(" , ");

    // MAX31865 Sensor Reading
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

    // Calculate error and PID control
    float error = setPoint - temperatureMax31865;
    integral += error;  // accumulate integral
    float derivative = error - lastError;  // calculate derivative

    // Calculate PID output
    output = Kp * error + Ki * integral + Kd * derivative;
    
    // Apply limits to avoid excessive control output
    if (output > 255) {
      output = 255;  // Maximum heater control
    } else if (output < 0) {
      output = 0;    // Minimum heater control
    }

    // Control the heater based on PID output
    if (output > 0) {
      digitalWrite(SSR_PIN, HIGH);  // Turn on the heater
      Serial.println("Heating ON");
    } else {
      digitalWrite(SSR_PIN, LOW);   // Turn off the heater
      Serial.println("Heating OFF");
    }

    // Adjust PID constants based on error magnitude (adaptive tuning)
    if (abs(error) > 5.0) {  // If error is large, speed up the response
      Kp = 1.5;
      Ki = 0.2;
      Kd = 1.2;
    } else if (abs(error) > 1.0) {  // Medium error, moderate gains
      Kp = 1.2;
      Ki = 0.15;
      Kd = 1.0;
    } else {  // Small error, fine-tune the control
      Kp = 1.0;
      Ki = 0.1;
      Kd = 0.8;
    }

    // Display the temperature
    Serial.print("Temperature: ");
    Serial.print(temperatureMax31865, 2);
    Serial.println(" °C");

    // Update last reading time
    unsigned long endTime = millis();
    unsigned long timeTakenForReadings = endTime - startTime;

    // Calculate remaining sleep time to keep 500ms interval
    unsigned long remainingSleepTime = readingInterval - timeTakenForReadings;
    if (remainingSleepTime > 0) {
      delay(remainingSleepTime);
    }

    // Update last error for next iteration
    lastError = error;
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
      Serial.println("Max iterations reached, returning approximate value");
    }
  }

  return t;
}
