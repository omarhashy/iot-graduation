#include <SoftwareSerial.h>

// RS485 Pins
#define RE_DE_PIN 8
#define RX_PIN 2
#define TX_PIN 3

// RS485 communication
SoftwareSerial rs485(RX_PIN, TX_PIN);

// Sensor structure
struct SensorData {
  float humidity;
  float temperature;
  int conductivity;
  float ph;
  int nitrogen;
  int phosphorus;
  int potassium;
};

void setup() {
  rs485.begin(4800);          // Sensor's default baud rate
  Serial.begin(9600);         // Communication with ESP32
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Start in receive mode
}

void loop() {
  SensorData data;

  if (readSensorData(data)) {
    sendToESP(data);
  }

  delay(5000); // Wait 10 seconds before next read
}

bool readSensorData(SensorData &data) {
  byte command[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};

  while (rs485.available()) rs485.read();

  digitalWrite(RE_DE_PIN, HIGH); delay(20);
  rs485.write(command, sizeof(command));
  rs485.flush();
  digitalWrite(RE_DE_PIN, LOW); delay(150);  // Allow sensor to respond

  if (rs485.available()) {
    byte response[30];
    int bytesRead = 0;

    while (rs485.available() && bytesRead < 30) {
      response[bytesRead++] = rs485.read();
      delay(5);
    }

    int i = 0;
    while (i <= bytesRead - 3) {
      if (response[i] == 0x01 && response[i+1] == 0x03 && response[i+2] == 0x0E) break;
      i++;
    }

    if ((bytesRead - i) >= 17) {
      int offset = i + 3;
      data.humidity    = ((response[offset] << 8) | response[offset+1]) / 10.0;
      data.temperature = ((response[offset+2] << 8) | response[offset+3]) / 10.0;
      data.conductivity= (response[offset+4] << 8) | response[offset+5];
      data.ph          = ((response[offset+6] << 8) | response[offset+7]) / 10.0;
      data.nitrogen    = (response[offset+8] << 8) | response[offset+9];
      data.phosphorus  = (response[offset+10] << 8) | response[offset+11];
      data.potassium   = (response[offset+12] << 8) | response[offset+13];
      return true;
    }
  }

  return false;
}

void sendToESP(const SensorData &data) {
  Serial.print(data.humidity);     Serial.print(",");
  Serial.print(data.temperature);  Serial.print(",");
  Serial.print(data.conductivity); Serial.print(",");
  Serial.print(data.ph);           Serial.print(",");
  Serial.print(data.nitrogen);     Serial.print(",");
  Serial.print(data.phosphorus);   Serial.print(",");
  Serial.println(data.potassium);  // End with newline
}
