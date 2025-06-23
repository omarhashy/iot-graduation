#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define RELAY_PIN 23
#define SERIAL1_BAUD 9600
#define DEVICE_SERIAL "ESP32-IOT-001"
#define WIFI_SSID "testesp"
#define WIFI_PASSWORD "1234567891011"

const String POST_URL = "https://ecoplant-backend.azurewebsites.net/api/kit/soilProperties";
const String GET_URL = "https://ecoplant-backend.azurewebsites.net/api/kits/humidity/" + String(DEVICE_SERIAL);

float humidity, temperature, ph;
int conductivity, nitrogen, phosphorus, potassium;

void setup() {
  Serial.begin(115200);
  Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, 16, 17);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected!");

  Serial.println("=== ESP32 Plant Monitor ===");
}

void loop() {
  static String buffer = "";

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        if (parseSensorData(buffer)) {
          sendSoilData();
          checkIrrigationStatus();  // Will decide watering based on humidity threshold
        }
        buffer = "";
      }
    } else {
      buffer += c;
    }
  }
}

bool parseSensorData(const String &line) {
  if (sscanf(line.c_str(), "%f,%f,%d,%f,%d,%d,%d",
             &humidity, &temperature, &conductivity, &ph,
             &nitrogen, &phosphorus, &potassium) == 7) {

    Serial.println("📡 Sensor Data Parsed:");
    Serial.printf(" - Humidity: %.1f %%\n", humidity);
    Serial.printf(" - Temperature: %.1f °C\n", temperature);
    Serial.printf(" - Conductivity: %d µS/cm\n", conductivity);
    Serial.printf(" - pH: %.1f\n", ph);
    Serial.printf(" - Nitrogen: %d mg/kg\n", nitrogen);
    Serial.printf(" - Phosphorus: %d mg/kg\n", phosphorus);
    Serial.printf(" - Potassium: %d mg/kg\n", potassium);
    return true;
  }

  Serial.println("❌ Failed to parse incoming sensor data.");
  return false;
}
void sendSoilData() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(POST_URL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<512> doc;

  // ✅ Send raw numbers (floats/ints)
  doc["serial_number"] = DEVICE_SERIAL;       // still a string
  doc["temperature"] = temperature;           // float
  doc["humidity"] = humidity;                 // float
  doc["conductivity"] = conductivity;         // int
  doc["ph"] = ph;                             // float
  doc["nitrogen"] = nitrogen;                 // int
  doc["phosphorus"] = phosphorus;             // int
  doc["potassium"] = potassium;               // int

  String payload;
  serializeJson(doc, payload);

  Serial.println("📤 Sending JSON payload:");
  Serial.println(payload);  // Optional debug

  int httpResponseCode = http.POST(payload);

  if (httpResponseCode > 0) {
    Serial.print("✅ POST response: ");
    Serial.println(httpResponseCode);
    Serial.println(http.getString());
  } else {
    Serial.print("❌ POST failed: ");
    Serial.println(http.errorToString(httpResponseCode));
  }

  http.end();
}


void checkIrrigationStatus() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(GET_URL);

  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    String response = http.getString();
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, response);

    if (err) {
      Serial.println("❌ Failed to parse GET response.");
      http.end();
      return;
    }

    bool isEnabled = doc["data"]["is_enabled"];
    float threshold = doc["data"]["humidity_threshold"].as<float>() ?: 30.0;

    Serial.print("🌿 Remote irrigation enabled: ");
    Serial.println(isEnabled ? "YES" : "NO");

    if (isEnabled && humidity < threshold) {
      Serial.println("🚨 Triggering remote-controlled watering...");
      digitalWrite(RELAY_PIN, HIGH);
      delay(5000);
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("✅ Watering complete.");
    } else {
      Serial.println("✅ No watering needed or irrigation disabled.");
    }

  } else {
    Serial.print("❌ GET request failed: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}
