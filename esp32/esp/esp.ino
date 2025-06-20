#define RELAY_PIN 23
#define HUMIDITY_THRESHOLD 30
#define SERIAL1_BAUD 9600

void setup() {
  Serial.begin(115200);                         // USB serial monitor
  Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, 16, 17); // RX: GPIO16, TX: GPIO17

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("=== ESP32 Plant Monitor ===");
  Serial.println("Listening for sensor data...");
}

void loop() {
  static String buffer = "";

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        processSensorData(buffer);
        buffer = "";
      }
    } else {
      buffer += c;
    }
  }
}

void processSensorData(const String &line) {
  float humidity, temperature, ph;
  int conductivity, nitrogen, phosphorus, potassium;

  if (sscanf(line.c_str(), "%f,%f,%d,%f,%d,%d,%d",
             &humidity, &temperature, &conductivity, &ph,
             &nitrogen, &phosphorus, &potassium) == 7) {

    Serial.println("📡 Sensor Data Received:");
    Serial.printf(" - Humidity: %.1f %%\n", humidity);
    Serial.printf(" - Temperature: %.1f °C\n", temperature);
    Serial.printf(" - Conductivity: %d µS/cm\n", conductivity);
    Serial.printf(" - pH: %.1f\n", ph);
    Serial.printf(" - Nitrogen: %d mg/kg\n", nitrogen);
    Serial.printf(" - Phosphorus: %d mg/kg\n", phosphorus);
    Serial.printf(" - Potassium: %d mg/kg\n", potassium);

    if (humidity < HUMIDITY_THRESHOLD) {
      Serial.println("🚨 Humidity low — watering plant!");
      digitalWrite(RELAY_PIN, HIGH);
      delay(5000);
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("✅ Watering done.");
    } else {
      Serial.printf("✅ Humidity is OK: %.1f %%\n", humidity);
    }

    Serial.println("=====================================\n");

  } else {
    Serial.print("❌ Failed to parse: ");
    Serial.println(line);
  }
}
