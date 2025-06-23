# NPK Soil Sensor System with Arduino and ESP32

This project implements a smart soil monitoring system that measures soil parameters (NPK, pH, temperature, humidity, conductivity) using an RS485 NPK sensor. The system connects to WiFi to send data to a cloud API and can control irrigation based on soil humidity levels.

## System Overview

The system consists of three main components:

1. **Arduino** - Acts as an interface to the RS485 NPK sensor, reading soil data and forwarding it to the ESP32
2. **ESP32** - Connects to WiFi, processes the data, sends it to the cloud API, and controls the irrigation system
3. **RS485 NPK Sensor** - 5-pin probe that measures soil parameters including Nitrogen, Phosphorus, Potassium, pH, conductivity, temperature, and humidity

## Hardware Connections

### NPK Sensor to Arduino (via MAX485 module)

The NPK sensor uses RS485 communication protocol which requires a MAX485 converter module to interface with the Arduino:

| NPK Sensor Wire | Connection   |
| --------------- | ------------ |
| Brown           | 5V           |
| Black           | GND          |
| Yellow/Green    | A+ on MAX485 |
| Blue            | B- on MAX485 |

MAX485 module connections:

| MAX485 Pin | Connection                                  |
| ---------- | ------------------------------------------- |
| VCC        | Arduino 5V                                  |
| GND        | Arduino GND                                 |
| DI         | Arduino pin 3 (TX)                          |
| RO         | Arduino pin 2 (RX)                          |
| DE         | Arduino pin 8                               |
| RE         | Arduino pin 8 (connected to same pin as DE) |

### Arduino to ESP32 (Serial Communication)

| Arduino Pin    | ESP32 Pin    |
| -------------- | ------------ |
| Pin 5 (ESP_RX) | Pin 17 (TX2) |
| Pin 6 (ESP_TX) | Pin 16 (RX2) |

### ESP32 to Relay Module

| ESP32 Pin | Connection                            |
| --------- | ------------------------------------- |
| Pin 23    | Relay input                           |
| -         | Relay VCC to 3.3V or 5V (as required) |
| -         | Relay GND to GND                      |

## Sensor Specifications

The NPK soil sensor is a 5-pin probe based on RS485 communication protocol:

- **Operating voltage**: 5V DC
- **Current consumption**: <20mA
- **Communication protocol**: RS485 (Modbus RTU)
- **Default baud rate**: 4800 bps
- **Measurement parameters**:
  - Soil moisture/humidity (%)
  - Soil temperature (°C)
  - Soil conductivity (μS/cm)
  - Soil pH (0-14)
  - Nitrogen content (mg/kg)
  - Phosphorus content (mg/kg)
  - Potassium content (mg/kg)

## Software Description

### Arduino Code (ardoino.ino)

The Arduino code handles:

1. Setting up RS485 communication with the NPK sensor
2. Sending Modbus RTU commands to request data
3. Parsing the response to extract soil parameter values
4. Forwarding the parsed data to the ESP32 via serial communication

Key components:

- Uses SoftwareSerial for RS485 communication
- Implements Modbus RTU protocol to communicate with the sensor
- Reads 7 soil parameters and packages them for the ESP32

```cpp
// Arduino code overview
void loop() {
  SensorData data;

  if (readSensorData(data)) {
    sendToESP(data);
  }

  delay(6000); // Wait 6 seconds before next read
}
```

### ESP32 Code (esp.ino)

The ESP32 code handles:

1. Receiving data from the Arduino via Serial1
2. Connecting to WiFi network
3. Sending soil data to a cloud API endpoint
4. Querying the cloud API for irrigation control settings
5. Controlling a relay for irrigation based on soil humidity and remote settings

Key components:

- Uses WiFi for internet connectivity
- Uses HTTPClient for API communication
- Uses ArduinoJSON for parsing and creating JSON payloads
- Controls a relay for automated irrigation

```cpp
// ESP32 code overview
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
```

## Communication Protocols

### 1. Sensor to Arduino: RS485 Modbus RTU

The Arduino communicates with the NPK sensor using the Modbus RTU protocol over RS485:

- **Command format**: `01 03 00 00 00 07 04 08`

  - `01`: Slave address
  - `03`: Function code (read holding registers)
  - `00 00`: Starting register address
  - `00 07`: Number of registers to read (7 parameters)
  - `04 08`: CRC checksum

- **Response format**: The sensor returns 19 bytes including:
  - Slave address, function code, byte count
  - 14 bytes of data (7 parameters × 2 bytes each)
  - 2 bytes CRC checksum

### 2. Arduino to ESP32: Serial UART

The Arduino sends data to the ESP32 using a simple CSV format over serial UART:

```
humidity,temperature,conductivity,ph,nitrogen,phosphorus,potassium
```

### 3. ESP32 to Cloud: HTTP REST API

The ESP32 communicates with the cloud using HTTP POST and GET requests:

- POST to `/api/kit/soilProperties` with JSON payload containing all soil parameters
- GET from `/api/kits/humidity/{DEVICE_SERIAL}` to retrieve irrigation settings

## System Setup Instructions

1. **Wiring**:

   - Connect all components according to the connection diagrams above
   - Ensure proper power supply for all components

2. **Arduino Setup**:

   - Upload the `ardoino.ino` sketch to your Arduino board
   - Make sure the RS485 module is properly connected

3. **ESP32 Setup**:

   - Update WiFi credentials in the `esp.ino` sketch
   - Set the appropriate device serial number
   - Upload the sketch to your ESP32 board

4. **Testing**:
   - Monitor the serial output of both Arduino and ESP32 to verify proper operation
   - Check the cloud dashboard to confirm data is being received

## Troubleshooting

### RS485 Communication Issues

- Check the A+ and B- connections - they must be correctly oriented
- Verify the sensor is receiving power (5V and GND)
- The MAX485 module needs proper control of DE/RE pins to switch between transmit and receive modes

### Serial Communication Issues

- Verify baud rates match on both Arduino and ESP32
- Check TX/RX connections are crossed (TX→RX, RX→TX)
- Ensure proper ground connection between Arduino and ESP32

### WiFi Connection Issues

- Verify WiFi credentials in the ESP32 code
- Check if the ESP32 is within range of the WiFi network
- Ensure the WiFi network has internet access

### Relay Control Issues

- Check the relay connection to the ESP32
- Verify the relay module's power supply
- Test the relay with a simple program to confirm functionality

## API Integration

The system is designed to work with a cloud API that stores soil data and provides irrigation control settings. The ESP32 makes two types of API calls:

1. **POST** to `/api/kit/soilProperties` with the following JSON payload:

```json
{
  "serial_number": "ESP32-IOT-001",
  "temperature": 25.5,
  "humidity": 45.2,
  "conductivity": 120,
  "ph": 6.8,
  "nitrogen": 14,
  "phosphorus": 31,
  "potassium": 19
}
```

2. **GET** from `/api/kits/humidity/{DEVICE_SERIAL}` which returns:

```json
{
  "data": {
    "is_enabled": true,
    "humidity_threshold": 30.0
  }
}
```

The system uses this data to determine if irrigation should be activated based on the current soil humidity and the threshold set in the cloud.

## Power Management

- The NPK sensor requires 5V power
- The Arduino operates on 5V
- The ESP32 operates on 3.3V but usually can be powered via USB or external 5V with onboard regulation
- The relay module may require 3.3V or 5V depending on the specific module

Ensure all components receive appropriate power according to their specifications.

---

## License

This project is open-source and available under the MIT License.

## References

- NPK Type (5Pin probe) manual V1.4
- Arduino and ESP32 documentation
- Modbus RTU protocol specification
