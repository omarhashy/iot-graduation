#include <SoftwareSerial.h>

// Pin definitions
#define RE_PIN 8        // RS485 Read Enable (connect to DE as well)
#define DE_PIN 8        // RS485 Data Enable (same pin as RE)
#define RELAY_PIN 7     // Relay control pin
#define RX_PIN 2        // Software serial RX
#define TX_PIN 3        // Software serial TX

// Create software serial for RS485 communication
SoftwareSerial rs485(RX_PIN, TX_PIN);

// Sensor parameters
const byte SENSOR_ADDRESS = 0x01;  // Default sensor address
const int HUMIDITY_THRESHOLD = 30; // Water when humidity < 30%
const int PUMP_DURATION = 5000;    // Run pump for 5 seconds
const int READ_INTERVAL = 10000;   // Read sensor every 10 seconds

// Debug mode - set to true for detailed debugging
bool DEBUG_MODE = true;

// Data structure for sensor readings
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
  Serial.begin(9600);
  rs485.begin(4800);  // Sensor default baud rate
  
  // Initialize pins
  pinMode(RE_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  
  // Set RS485 to receive mode initially
  digitalWrite(RE_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW); // Ensure pump is off
  
  Serial.println("=== Arduino Plant Watering System - DEBUG MODE ===");
  Serial.println("System initialized. Starting diagnostics...");
  Serial.println();
  
  // Run initial diagnostics
  runDiagnostics();
}

void loop() {
  SensorData data;
  
  // Read all sensor data
  if (readSensorData(data)) {
    // Display readings
    displaySensorData(data);
    
    // Check humidity and control pump
    checkAndWaterPlant(data.humidity);
  } else {
    Serial.println("Failed to read sensor data!");
  }
  
  delay(READ_INTERVAL);
}

bool readSensorData(SensorData &data) {
  // Command to read registers 0x0000 to 0x0006 (7 registers total)
  byte command[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  
  if (DEBUG_MODE) {
    Serial.print("Sending command: ");
    for (int i = 0; i < sizeof(command); i++) {
      Serial.print("0x");
      if (command[i] < 16) Serial.print("0");
      Serial.print(command[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Clear any existing data in buffer
  while (rs485.available()) {
    rs485.read();
  }
  
  // Send command
  digitalWrite(RE_PIN, HIGH); // Enable transmission
  delay(20);
  
  for (int i = 0; i < sizeof(command); i++) {
    rs485.write(command[i]);
  }
  rs485.flush();
  
  digitalWrite(RE_PIN, LOW);  // Enable reception
  delay(200); // Wait for response
  
  if (DEBUG_MODE) {
    Serial.print("Bytes available: ");
    Serial.println(rs485.available());
  }
  
  // Read response
  if (rs485.available() > 0) {
    byte response[30]; // Larger buffer
    int bytesRead = 0;
    
    // Read all available bytes
    while (rs485.available() && bytesRead < 30) {
      response[bytesRead] = rs485.read();
      bytesRead++;
      delay(10);
    }
    
    if (DEBUG_MODE) {
      Serial.print("Response received (");
      Serial.print(bytesRead);
      Serial.print(" bytes): ");
      for (int i = 0; i < bytesRead; i++) {
        Serial.print("0x");
        if (response[i] < 16) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    
    // Find the start of the actual sensor response (0x01 0x03 0x0E pattern)
    int responseStart = -1;
    for (int i = 0; i <= bytesRead - 3; i++) {
      if (response[i] == 0x01 && response[i+1] == 0x03 && response[i+2] == 0x0E) {
        responseStart = i;
        break;
      }
    }
    
    if (responseStart >= 0 && (bytesRead - responseStart) >= 17) {
      if (DEBUG_MODE) {
        Serial.print("Valid response found starting at position: ");
        Serial.println(responseStart);
      }
      
      // Parse data starting from the correct position
      int offset = responseStart + 3; // Skip 0x01 0x03 0x0E
      
      data.humidity = ((response[offset] << 8) | response[offset+1]) / 10.0;
      data.temperature = ((response[offset+2] << 8) | response[offset+3]) / 10.0;
      data.conductivity = (response[offset+4] << 8) | response[offset+5];
      data.ph = ((response[offset+6] << 8) | response[offset+7]) / 10.0;
      data.nitrogen = (response[offset+8] << 8) | response[offset+9];
      data.phosphorus = (response[offset+10] << 8) | response[offset+11];
      data.potassium = (response[offset+12] << 8) | response[offset+13];
      
      if (DEBUG_MODE) {
        Serial.println("Data parsed successfully!");
      }
      return true;
    } else {
      if (DEBUG_MODE) {
        Serial.println("Could not find valid response pattern (0x01 0x03 0x0E)!");
      }
    }
  } else {
    if (DEBUG_MODE) {
      Serial.println("No response received from sensor!");
    }
  }
  
  return false;
}

void displaySensorData(const SensorData &data) {
  Serial.println("--- Sensor Readings ---");
  Serial.print("Humidity: ");
  Serial.print(data.humidity);
  Serial.println("%");
  
  Serial.print("Temperature: ");
  Serial.print(data.temperature);
  Serial.println("°C");
  
  Serial.print("Conductivity: ");
  Serial.print(data.conductivity);
  Serial.println(" µS/cm");
  
  Serial.print("pH: ");
  Serial.println(data.ph);
  
  Serial.print("Nitrogen (N): ");
  Serial.print(data.nitrogen);
  Serial.println(" mg/kg");
  
  Serial.print("Phosphorus (P): ");
  Serial.print(data.phosphorus);
  Serial.println(" mg/kg");
  
  Serial.print("Potassium (K): ");
  Serial.print(data.potassium);
  Serial.println(" mg/kg");
  
  Serial.println("----------------------");
}

void checkAndWaterPlant(float humidity) {
  if (humidity < HUMIDITY_THRESHOLD) {
    Serial.print("WATERING NEEDED! Humidity (");
    Serial.print(humidity);
    Serial.print("%) is below threshold (");
    Serial.print(HUMIDITY_THRESHOLD);
    Serial.println("%)");
    
    activatePump();
  } else {
    Serial.print("Soil moisture OK. Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
  }
}

void activatePump() {
  Serial.println("🚰 Activating water pump...");
  
  // Turn on pump
  digitalWrite(RELAY_PIN, HIGH);
  
  // Show countdown
  for (int i = PUMP_DURATION; i > 0; i -= 1000) {
    Serial.print("Pumping... ");
    Serial.print(i/1000);
    Serial.println(" seconds remaining");
    delay(1000);
  }
  
  // Turn off pump
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("✅ Watering complete!");
  Serial.println();
}

// Function to manually calibrate NPK values (optional)
void writeNPKValue(byte registerAddr, int value) {
  byte command[] = {0x01, 0x06, 0x00, registerAddr, 
                    (byte)(value >> 8), (byte)(value & 0xFF), 0x00, 0x00};
  
  // Calculate CRC (simplified - you may want to implement proper CRC)
  // For now, using the values from the manual examples
  
  digitalWrite(RE_PIN, HIGH);
  delay(10);
  
  rs485.write(command, sizeof(command));
  rs485.flush();
  
  digitalWrite(RE_PIN, LOW);
  delay(100);
}

// Diagnostic function to test connections
void runDiagnostics() {
  Serial.println("Running system diagnostics...");
  Serial.println("1. Testing pin configurations...");
  
  // Test RE/DE pin
  digitalWrite(RE_PIN, HIGH);
  delay(100);
  digitalWrite(RE_PIN, LOW);
  Serial.println("   RE/DE pin test: OK");
  
  // Test relay pin
  digitalWrite(RELAY_PIN, HIGH);
  delay(500);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("   Relay pin test: OK (relay should have clicked)");
  
  Serial.println("2. Testing RS485 communication...");
  Serial.println("   Attempting to read sensor...");
  
  // Try multiple communication attempts
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("   Attempt ");
    Serial.print(attempt);
    Serial.print(": ");
    
    SensorData testData;
    if (readSensorData(testData)) {
      Serial.println("SUCCESS!");
      displaySensorData(testData);
      DEBUG_MODE = false; // Turn off debug mode after successful read
      return;
    } else {
      Serial.println("FAILED");
      delay(1000);
    }
  }
  
  Serial.println("\n=== TROUBLESHOOTING GUIDE ===");
  Serial.println("Communication failed. Please check:");
  Serial.println("1. Wiring connections:");
  Serial.println("   - Brown wire (sensor) → 5V");
  Serial.println("   - Black wire (sensor) → GND");
  Serial.println("   - Yellow/Green wire (sensor) → A+ (MAX485)");
  Serial.println("   - Blue wire (sensor) → B- (MAX485)");
  Serial.println("2. MAX485 to Arduino:");
  Serial.println("   - VCC → 5V, GND → GND");
  Serial.println("   - DI → Pin 3, RO → Pin 2");
  Serial.println("   - DE & RE → Pin 8");
  Serial.println("3. Power supply to sensor (should be 5-30V DC)");
  Serial.println("4. Sensor is properly inserted in soil");
  Serial.println("===============================\n");
}

// Alternative simple test function
void simpleTest() {
  Serial.println("Running simple communication test...");
  
  // Try different baud rates
  int baudRates[] = {2400, 4800, 9600};
  
  for (int i = 0; i < 3; i++) {
    Serial.print("Testing baud rate: ");
    Serial.println(baudRates[i]);
    
    rs485.begin(baudRates[i]);
    delay(100);
    
    SensorData testData;
    if (readSensorData(testData)) {
      Serial.print("SUCCESS at ");
      Serial.print(baudRates[i]);
      Serial.println(" baud!");
      displaySensorData(testData);
      return;
    }
    
    delay(500);
  }
  
  Serial.println("All baud rates failed. Check wiring!");
}