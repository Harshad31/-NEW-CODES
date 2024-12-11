#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include <Adafruit_MAX31865.h>

// Create Modbus device and slave objects
modbusDevice regBank;
modbusSlave slave;

// MAX31865 connections
#define MAX31865_MISO 19
#define MAX31865_MOSI 23
#define MAX31865_CLK 18
#define MAX31865_CS 5

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

// RS485 direction control pin
#define RS485_DIR_PIN 25

// PT100 constants
#define R0 100.0
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

void setup() {
  // Initialize serial communication for Modbus (RS485)
  Serial2.begin(9600, SERIAL_8E1, 16, 17); // GPIO16 RX2, GPIO17 TX2
  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW); // Set to receive mode initially

  // Initialize MAX31865
  thermo.begin(MAX31865_3WIRE);

  // Configure Modbus slave
  regBank.setId(1);         // Set Modbus slave ID to 1
  regBank.add(40005);       // Add register address 40005
  slave._device = &regBank; // Assign regBank to the Modbus slave
}

void loop() {
  uint16_t rtd = thermo.readRTD();
  float Rt = (rtd / 32768.0 * 430.0); // Calculate RTD resistance

  float temperature = calculateTemperature(Rt);

  if (!isnan(temperature)) { // Check if temperature is valid
    int16_t scaledTemp = (int)(temperature * 10); // Scale for one decimal precision
    regBank.set(40005, scaledTemp); // Write to Modbus register
  }

  // Check and handle faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    handleFaults(fault);
    thermo.clearFault();
  }

  // Switch to send mode for Modbus communication
  digitalWrite(RS485_DIR_PIN, HIGH);
  slave.run(); // Run Modbus slave
  // Switch back to receive mode
  digitalWrite(RS485_DIR_PIN, LOW);
}

float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    // For temperatures above 0 °C
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    // For temperatures below 0 °C
    float tolerance = 0.001; // Convergence tolerance
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
      Serial.println("Max iterations reached. Convergence not achieved.");
    }
  }

  return t;
}

void handleFaults(uint8_t fault) {
  Serial.print("Fault 0x");
  Serial.println(fault, HEX);
  if (fault & MAX31865_FAULT_HIGHTHRESH) {
    Serial.println("RTD High Threshold");
  }
  if (fault & MAX31865_FAULT_LOWTHRESH) {
    Serial.println("RTD Low Threshold");
  }
  if (fault & MAX31865_FAULT_REFINLOW) {
    Serial.println("REFIN- > 0.85 x Bias");
  }
  if (fault & MAX31865_FAULT_REFINHIGH) {
    Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
  }
  if (fault & MAX31865_FAULT_RTDINLOW) {
    Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
  }
  if (fault & MAX31865_FAULT_OVUV) {
    Serial.println("Under/Over voltage");
  }
}