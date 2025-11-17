// Simple ESP32 "device" to test your Web Serial app
// Protocol: SOF | LEN | TYPE | PAYLOAD | CRC | EOF
// SOF = 0xAA, EOF = 0x55
// LEN = number of bytes from TYPE through end of PAYLOAD
// CRC = XOR of bytes from LEN .. end of PAYLOAD (same as browser app)

#include <Arduino.h>

static const uint8_t SOF = 0xAA;
static const uint8_t EOF_BYTE = 0x55;

static const uint8_t CMD_READ_VAR      = 0x01;
static const uint8_t CMD_WRITE_VAR     = 0x02;
static const uint8_t CMD_READ_RESPONSE = 0x81;

static const uint32_t BAUD_RATE = 115200;

// Variable IDs
static const uint8_t VAR_TEMP_ID     = 0x10;
static const uint8_t VAR_PRESS_ID    = 0x11;
static const uint8_t VAR_FLOW_ID     = 0x12;

// "Fake" device variables
int16_t temperatureC = 25;   // °C
int16_t pressureKPa  = 101;  // kPa
int16_t flowLpm      = 5;    // L/min

// RX buffer
#define RX_BUFFER_SIZE 128
uint8_t rxBuffer[RX_BUFFER_SIZE];
size_t  rxLen = 0;

// Use UART2 (you can remap pins)
HardwareSerial SerialUART(2); // UART2

// Choose your RX and TX pins
#define RXD2 16
#define TXD2 17

// -----------------------------------------------------------------------------
// CRC: XOR of bytes from startIndex to endIndex (inclusive)
// -----------------------------------------------------------------------------
uint8_t calcCRC(const uint8_t* buf, size_t startIndex, size_t endIndexInclusive) {
  uint8_t crc = 0;
  for (size_t i = startIndex; i <= endIndexInclusive; i++) {
    crc ^= buf[i];
  }
  return crc;
}

// -----------------------------------------------------------------------------
// Send a frame: SOF | LEN | TYPE | PAYLOAD... | CRC | EOF
// LEN = TYPE + PAYLOAD length
// -----------------------------------------------------------------------------
void sendFrame(uint8_t type, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t len     = 1 + payloadLen;            // TYPE + payload
  uint8_t total   = 1 + 1 + len + 1 + 1;       // SOF + LEN + (TYPE+PAYLOAD) + CRC + EOF
  uint8_t frame[32];

  if (total > sizeof(frame)) {
    // Too big for this simple demo; ignore
    return;
  }

  size_t i = 0;
  frame[i++] = SOF;
  frame[i++] = len;
  frame[i++] = type;
  for (uint8_t j = 0; j < payloadLen; j++) {
    frame[i++] = payload[j];
  }

  // CRC over LEN..end of payload
  uint8_t crc = calcCRC(frame, 1, i - 1);
  frame[i++] = crc;
  frame[i++] = EOF_BYTE;

  SerialUART.write(frame, i);
}

// -----------------------------------------------------------------------------
// Helpers to get/set variable values
// -----------------------------------------------------------------------------
bool getVariableValue(uint8_t varId, int16_t &outVal) {
  switch (varId) {
    case VAR_TEMP_ID:  outVal = temperatureC; return true;
    case VAR_PRESS_ID: outVal = pressureKPa;  return true;
    case VAR_FLOW_ID:  outVal = flowLpm;      return true;
    default:
      return false;
  }
}

bool setVariableValue(uint8_t varId, int16_t newVal) {
  switch (varId) {
    case VAR_TEMP_ID:  temperatureC = newVal; return true;
    case VAR_PRESS_ID: pressureKPa  = newVal; return true;
    case VAR_FLOW_ID:  flowLpm      = newVal; return true;
    default:
      return false;
  }
}

// -----------------------------------------------------------------------------
// Handle a full, valid frame
// frame[0] = SOF
// frame[1] = LEN
// frame[2] = TYPE
// payload = frame[3 .. frameLen-3]
// frame[frameLen-2] = CRC
// frame[frameLen-1] = EOF
// -----------------------------------------------------------------------------
void handleFrame(const uint8_t* frame, size_t frameLen) {
  uint8_t len = frame[1];
  uint8_t type = frame[2];
  const uint8_t* payload = &frame[3];
  size_t payloadLen = len - 1; // subtract TYPE

  if (type == CMD_READ_VAR) {
    if (payloadLen < 1) return;
    uint8_t varId = payload[0];
    int16_t value;
    if (getVariableValue(varId, value)) {
      // Build READ_RESPONSE: [varId, lo, hi]
      uint8_t outPayload[3];
      outPayload[0] = varId;
      outPayload[1] = (uint8_t)(value & 0xFF);         // lo
      outPayload[2] = (uint8_t)((value >> 8) & 0xFF);  // hi
      sendFrame(CMD_READ_RESPONSE, outPayload, 3);
    }
  }
  else if (type == CMD_WRITE_VAR) {
    if (payloadLen < 3) return;
    uint8_t varId = payload[0];
    uint8_t lo    = payload[1];
    uint8_t hi    = payload[2];
    int16_t newVal = (int16_t)((hi << 8) | lo);

    if (setVariableValue(varId, newVal)) {
      // Option: send back a READ_RESPONSE to confirm
      uint8_t outPayload[3];
      outPayload[0] = varId;
      outPayload[1] = lo;
      outPayload[2] = hi;
      sendFrame(CMD_READ_RESPONSE, outPayload, 3);
    }
  }
  else {
    // Unknown command type – ignore for this demo
  }
}

// -----------------------------------------------------------------------------
// Try to parse frames from rxBuffer
// We mimic the JS logic:
//   - find SOF
//   - check LEN
//   - compute total size = 1 + 1 + LEN + 1 + 1
//   - check EOF, CRC
// -----------------------------------------------------------------------------
void parseFramesFromBuffer() {
  const size_t MIN_FRAME = 5; // SOF + LEN + TYPE + CRC + EOF (no payload)

  while (rxLen >= MIN_FRAME) {
    // Find SOF
    size_t sofIndex = SIZE_MAX;
    for (size_t i = 0; i < rxLen; i++) {
      if (rxBuffer[i] == SOF) {
        sofIndex = i;
        break;
      }
    }
    if (sofIndex == SIZE_MAX) {
      // No SOF found; drop buffer
      rxLen = 0;
      return;
    }

    // Drop bytes before SOF
    if (sofIndex > 0) {
      memmove(rxBuffer, rxBuffer + sofIndex, rxLen - sofIndex);
      rxLen -= sofIndex;
    }

    if (rxLen < MIN_FRAME) {
      // Not enough bytes yet
      return;
    }

    uint8_t len = rxBuffer[1];
    size_t totalFrameLen = 1 + 1 + len + 1 + 1; // SOF + LEN + (TYPE+PAYLOAD) + CRC + EOF

    if (totalFrameLen > RX_BUFFER_SIZE) {
      // Something is wrong; drop this SOF and continue
      memmove(rxBuffer, rxBuffer + 1, rxLen - 1);
      rxLen -= 1;
      continue;
    }

    if (rxLen < totalFrameLen) {
      // Wait for more data
      return;
    }

    // We have a full frame
    uint8_t frame[RX_BUFFER_SIZE];
    memcpy(frame, rxBuffer, totalFrameLen);

    // Remove it from buffer
    memmove(rxBuffer, rxBuffer + totalFrameLen, rxLen - totalFrameLen);
    rxLen -= totalFrameLen;

    // Verify EOF
    if (frame[totalFrameLen - 1] != EOF_BYTE) {
      // Bad EOF; ignore
      continue;
    }

    // Verify CRC
    uint8_t crcReceived = frame[totalFrameLen - 2];
    uint8_t crcComputed = calcCRC(frame, 1, totalFrameLen - 3); // LEN..payload end
    if (crcReceived != crcComputed) {
      // CRC error; ignore for now
      continue;
    }

    // Valid frame
    handleFrame(frame, totalFrameLen);
  }
}

// -----------------------------------------------------------------------------
// Arduino setup/loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  // Optional: wait for Serial in case of debugging via USB
  SerialUART.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Serial.println("Waiting for UART data...");
  while (!Serial) {
    delay(10);
  }

}

void loop() {
  // Read any available bytes into rxBuffer
  while (SerialUART.available() > 0) {
    if (rxLen < RX_BUFFER_SIZE) {
      rxBuffer[rxLen++] = (uint8_t)SerialUART.read();
    } else {
      // Buffer overflow; reset
      rxLen = 0;
    }
  }

  // Try to parse frames
  if (rxLen > 0) {
    parseFramesFromBuffer();
  }

  // Optional: you could periodically modify variables here
  // (e.g., simulate sensor changing over time)
  // Example: every few seconds change temperature a little
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if (now - lastUpdate > 3000) {
    lastUpdate = now;
    temperatureC++;
    if (temperatureC > 35) {
      temperatureC = 25;
    }
  }
}
