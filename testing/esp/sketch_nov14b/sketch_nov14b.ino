// ESP32 test device for your Web Serial app using UART2 (TX2/RX2)
// Protocol: SOF | LEN | TYPE | PAYLOAD | CRC | EOF

#include <Arduino.h>

#define ESP32_RX2 16   // RX2 (to USB-TTL TX)
#define ESP32_TX2 17   // TX2 (to USB-TTL RX)
#define BAUD_RATE 115200

HardwareSerial DeviceSerial(2);

// Protocol constants
static const uint8_t SOF        = 0xAA;
static const uint8_t EOF_BYTE   = 0x55;

static const uint8_t CMD_READ_VAR       = 0x01;
static const uint8_t CMD_WRITE_VAR      = 0x02;
static const uint8_t CMD_NOTIFY_START   = 0x03;
static const uint8_t CMD_NOTIFY_STOP    = 0x04;

static const uint8_t CMD_READ_RESPONSE  = 0x81;
static const uint8_t CMD_NOTIFY_DATA    = 0x82;

// Variable IDs
static const uint8_t VAR_TEMP_ID  = 0x10;
static const uint8_t VAR_PRESS_ID = 0x11;
static const uint8_t VAR_FLOW_ID  = 0x12;

// Fake device variables
int16_t temperatureC = 25;
int16_t pressureKPa  = 101;
int16_t flowLpm      = 5;

// Notify state
bool     notifyEnabled    = false;
uint8_t  notifyVarId      = 0x00;
uint32_t lastNotifyMillis = 0;
const uint32_t NOTIFY_PERIOD_MS = 1000;

// RX buffer
#define RX_BUFFER_SIZE 128
uint8_t rxBuffer[RX_BUFFER_SIZE];
size_t  rxLen = 0;

// CRC: XOR of bytes from start..end
uint8_t calcCRC(const uint8_t* buf, size_t startIndex, size_t endIndexInclusive) {
  uint8_t crc = 0;
  for (size_t i = startIndex; i <= endIndexInclusive; i++) {
    crc ^= buf[i];
  }
  return crc;
}

// Send a frame: SOF | LEN | TYPE | PAYLOAD | CRC | EOF
void sendFrame(uint8_t type, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t len      = 1 + payloadLen;
  uint8_t totalLen = 1 + 1 + len + 1 + 1; // SOF + LEN + (TYPE+PAYLOAD) + CRC + EOF
  if (totalLen > 32) return; // simple guard

  uint8_t frame[32];
  size_t i = 0;
  frame[i++] = SOF;
  frame[i++] = len;
  frame[i++] = type;
  for (uint8_t j = 0; j < payloadLen; j++) {
    frame[i++] = payload[j];
  }
  uint8_t crc = calcCRC(frame, 1, i - 1);
  frame[i++] = crc;
  frame[i++] = EOF_BYTE;

  DeviceSerial.write(frame, i);
}

// Helpers to read/set variables
bool getVar(uint8_t id, int16_t &outVal) {
  switch (id) {
    case VAR_TEMP_ID:  outVal = temperatureC; return true;
    case VAR_PRESS_ID: outVal = pressureKPa;  return true;
    case VAR_FLOW_ID:  outVal = flowLpm;      return true;
    default: return false;
  }
}

bool setVar(uint8_t id, int16_t val) {
  switch (id) {
    case VAR_TEMP_ID:  temperatureC = val; return true;
    case VAR_PRESS_ID: pressureKPa  = val; return true;
    case VAR_FLOW_ID:  flowLpm      = val; return true;
    default: return false;
  }
}

// Send notify data frame for a given variable
void sendNotifyData(uint8_t varId) {
  int16_t value;
  if (!getVar(varId, value)) return;

  uint8_t payload[3];
  payload[0] = varId;
  payload[1] = (uint8_t)(value & 0xFF);        // LSB
  payload[2] = (uint8_t)((value >> 8) & 0xFF); // MSB
  sendFrame(CMD_NOTIFY_DATA, payload, 3);
}

// Handle one complete valid frame
void handleFrame(const uint8_t* frame, size_t frameLen) {
  uint8_t len = frame[1];
  uint8_t type = frame[2];
  const uint8_t* payload = &frame[3];
  size_t payloadLen = len - 1; // minus TYPE

  if (type == CMD_READ_VAR) {
    if (payloadLen < 1) return;
    uint8_t varId = payload[0];
    int16_t value;
    if (getVar(varId, value)) {
      uint8_t outPayload[3];
      outPayload[0] = varId;
      outPayload[1] = (uint8_t)(value & 0xFF);
      outPayload[2] = (uint8_t)((value >> 8) & 0xFF);
      sendFrame(CMD_READ_RESPONSE, outPayload, 3);
    }
  }
  else if (type == CMD_WRITE_VAR) {
    if (payloadLen < 3) return;
    uint8_t varId = payload[0];
    uint8_t lo    = payload[1];
    uint8_t hi    = payload[2];
    int16_t newVal = (int16_t)((hi << 8) | lo);
    if (setVar(varId, newVal)) {
      // Confirm write using READ_RESPONSE
      uint8_t outPayload[3];
      outPayload[0] = varId;
      outPayload[1] = lo;
      outPayload[2] = hi;
      sendFrame(CMD_READ_RESPONSE, outPayload, 3);
    }
  }
  else if (type == CMD_NOTIFY_START) {
    if (payloadLen < 1) return;
    uint8_t varId = payload[0];
    int16_t dummy;
    if (getVar(varId, dummy)) {
      notifyEnabled = true;
      notifyVarId = varId;
      lastNotifyMillis = millis();
      // Optionally send immediate data once:
      sendNotifyData(notifyVarId);
    }
  }
  else if (type == CMD_NOTIFY_STOP) {
    // Option: payload[0] is varId; we just stop all
    notifyEnabled = false;
  }
  else {
    // Ignore unknown commands
  }
}

// Parse frames from rxBuffer
void parseFramesFromBuffer() {
  const size_t MIN_FRAME = 5;

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
      rxLen = 0;
      return;
    }

    // Drop bytes before SOF
    if (sofIndex > 0) {
      memmove(rxBuffer, rxBuffer + sofIndex, rxLen - sofIndex);
      rxLen -= sofIndex;
    }

    if (rxLen < MIN_FRAME) return;

    uint8_t len = rxBuffer[1];
    size_t totalFrameLen = 1 + 1 + len + 1 + 1;
    if (totalFrameLen > RX_BUFFER_SIZE) {
      // Drop this SOF and continue
      memmove(rxBuffer, rxBuffer + 1, rxLen - 1);
      rxLen -= 1;
      continue;
    }

    if (rxLen < totalFrameLen) return;

    uint8_t frame[RX_BUFFER_SIZE];
    memcpy(frame, rxBuffer, totalFrameLen);

    // Remove frame bytes from buffer
    memmove(rxBuffer, rxBuffer + totalFrameLen, rxLen - totalFrameLen);
    rxLen -= totalFrameLen;

    // Check EOF
    if (frame[totalFrameLen - 1] != EOF_BYTE) {
      continue;
    }

    // CRC check
    uint8_t crcReceived = frame[totalFrameLen - 2];
    uint8_t crcComputed = calcCRC(frame, 1, totalFrameLen - 3);
    if (crcReceived != crcComputed) {
      continue;
    }

    // Valid frame
    handleFrame(frame, totalFrameLen);
  }
}

void setup() {
  // Debug USB serial (optional)
  Serial.begin(115200);
  // UART2 for device protocol
  DeviceSerial.begin(BAUD_RATE, SERIAL_8N1, ESP32_RX2, ESP32_TX2);
}

void loop() {
  // Read from UART2
  while (DeviceSerial.available() > 0) {
    if (rxLen < RX_BUFFER_SIZE) {
      rxBuffer[rxLen++] = (uint8_t)DeviceSerial.read();
    } else {
      // Overflow, reset buffer
      rxLen = 0;
    }
  }

  if (rxLen > 0) {
    parseFramesFromBuffer();
  }

  // Periodic notify
  if (notifyEnabled) {
    uint32_t now = millis();
    if (now - lastNotifyMillis >= NOTIFY_PERIOD_MS) {
      lastNotifyMillis = now;
      sendNotifyData(notifyVarId);
    }
  }

  // Optional: simulate changing temperature
  static uint32_t lastTempUpdate = 0;
  uint32_t now = millis();
  if (now - lastTempUpdate > 3000) {
    lastTempUpdate = now;
    temperatureC++;
    if (temperatureC > 35) temperatureC = 25;
  }
}

