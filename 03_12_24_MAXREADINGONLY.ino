#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <WiFi.h>

#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

// #define MAX31865_MISO 19
// #define MAX31865_MOSI 23
// #define MAX31865_CLK 18
// #define MAX31865_CS 5

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

const char* ssid     = "VIRAT";
const char* password = "12345678";

#define R0 100.0
#define Rref 430.0

#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

const int totalReadingsMAX31865 = 2;

double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

unsigned long totalReadingTime = 0;
unsigned long totalSleepTime = 0;
int readingCount = 0;
int sleepCount = 0;

const long readingInterval = 500;   // Interval between readings (500ms)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected.");

  thermo.begin(MAX31865_3WIRE);
  Serial.println("MAX31865 Temperature Sensor Initialized");
}

void loop() {
  unsigned long startTime = millis();  // Start time for the cycle

  // MAX31865 Sensor Reading
  for (int i = 0; i < totalReadingsMAX31865; i++) {
    uint16_t adcCode = thermo.readRTD();
    float Rt = (adcCode * Rref) / 32768.0;
    temperatureReadingsMAX31865[i] = Rt;
  }

  // Calculate average temperature from MAX31865
  double tempSumMax31865 = 0;
  for (int i = 0; i < totalReadingsMAX31865; i++) {
    tempSumMax31865 += temperatureReadingsMAX31865[i];
  }
  float averagedRtMax31865 = tempSumMax31865 / totalReadingsMAX31865;
  float temperatureMax31865 = calculateTemperature(averagedRtMax31865);

  // Print the MAX31865 temperature data
  Serial.print("RTD TEMP, ");
  Serial.print(temperatureMax31865, 2);
  Serial.println("°C");

  // End of sensor reading
  unsigned long endTime = millis();
  unsigned long timeTakenForReadings = endTime - startTime;
  totalReadingTime += timeTakenForReadings;
  readingCount++;

  // Calculate remaining sleep time to keep 500ms interval
  unsigned long remainingSleepTime = readingInterval - timeTakenForReadings;
  if (remainingSleepTime > 0) {
    delay(remainingSleepTime);
    totalSleepTime += remainingSleepTime;
    sleepCount++;
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
