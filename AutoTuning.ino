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

unsigned long totalReadingTime = 0;
unsigned long totalSleepTime = 0;
int readingCount = 0;
int sleepCount = 0;

unsigned long lastReadingTime = 0; 
const long readingInterval = 500;   // Interval between readings 

// EMA variables
float previousEMA = 0.0;  // Store the previous EMA value
const float alpha = 0.1;  // Smoothing factor ( alpha = 0.1)

// PID Control variables
float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 1.0;  // Derivative gain

float lastError = 0.0;  // To store the previous error for derivative calculation
float integral = 0.0;   // To accumulate the integral term

float setPoint = 25.0;   

float output = 0.0;     // PID output, used to control SSR
int SSR_PIN = 21;       // Pin to control SSR 

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
  digitalWrite(SSR_PIN, LOW);  // Turn off 
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

    Serial.print(formattedTime);  // Print the formatted time and sensor data in one line
    Serial.print(" , ");

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

    // Apply EMA to smooth the temperature value
    if (readingCount == 0) {
      previousEMA = temperatureMax31865;  // Initialize with the first reading
    } else {
      previousEMA = alpha * temperatureMax31865 + (1 - alpha) * previousEMA;  // Calculate EMA
    }

    // Print the smoothed temperature (EMA)
    Serial.print("TEMPERATURE, ");
    Serial.print(previousEMA, 2);
    Serial.println("°C");

    // Calculate error and PID terms
    float error = setPoint - previousEMA;  // Error between setpoint and actual temperature
    unsigned long now = millis();
    float timeChange = (now - lastReadingTime) / 1000.0;  // Time interval in seconds

    integral += error * timeChange;  // Integral term
    float derivative = (error - lastError) / timeChange;  // Derivative term

    // PID output
    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, 0, 255);  // Constrain the output to 0-255 range

    // Control SSR based on PID output
    if (output > 0) {
      digitalWrite(SSR_PIN, HIGH);  // Turn on SSR 
      Serial.println("Heating ON");
    } else {
      digitalWrite(SSR_PIN, LOW);   // Turn off SSR 
      Serial.println("Heating OFF");
    }

    // Auto-tuning the PID parameters based on output
    if (output > 200) {
      Kp += 0.1;  // Increase Kp if output is too high
      Ki -= 0.05; // Decrease Ki 
      Kd += 0.05; // Increase Kd 
    } else if (output < 50) {
      Kp -= 0.1;  // Decrease Kp if output is too low
      Ki += 0.05; // Increase Ki 
      Kd -= 0.05; // Decrease Kd 
    }

    // Constrain PID values to prevent runaway changes
    Kp = constrain(Kp, 0.1, 10.0);  
    Ki = constrain(Ki, 0.0, 1.0);   
    Kd = constrain(Kd, 0.0, 5.0);   

    // Update PID parameters
    lastError = error;
    lastReadingTime = now;

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
