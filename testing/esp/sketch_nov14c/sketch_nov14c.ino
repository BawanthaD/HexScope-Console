// ESP32 test device for your Web Serial app (UART2, multi-variable notify)
//
// UART2:
//   RX2 = GPIO16 (connect to USB-TTL TX)
//   TX2 = GPIO17 (connect to USB-TTL RX)
//   GND common
//
// Protocol:
//   SOF = 0xAA
//   EOF = 0x55
//   Frame = SOF | LEN | TYPE | PAYLOAD... | CRC | EOF
//   LEN = number of bytes from TYPE through end of PAYLOAD
//   CRC = XOR of bytes from LEN..end of PAYLOAD

#include <Arduino.h>

// ---------- UART2 CONFIG ----------
#define ESP32_RX2 16   // RX2 (to USB-TTL TX)
#define ESP32_TX2 17   // TX2 (to USB-TTL RX)
#define BAUD_RATE 115200

HardwareSerial DeviceSerial(2);   // UART2

// ---------- Protocol constants ----------
static const uint8_t SOF        = 0xAA;
static const uint8_t EOF_BYTE   = 0x55;

static const uint8_t CMD_READ_VAR       = 0x01;
static const uint8_t CMD_WRITE_VAR      = 0x02;
static const uint8_t CMD_NOTIFY_START   = 0x03;
static const uint8_t CMD_NOTIFY_STOP    = 0x04;

static const uint8_t CMD_READ_RESPONSE  = 0x81;
static const uint8_t CMD_NOTIFY_DATA    = 0x82;

// ---------- Variable IDs ----------
static const uint8_t VAR_TEMP_ID  = 0x10;
static const uint8_t VAR_PRESS_ID = 0x11;
static const uint8_t VAR_FLOW_ID  = 0x12;

// List of all supported variable IDs
static const uint8_t VAR_IDS[] = { VAR_TEMP_ID, VAR_PRESS_ID, VAR_FLOW_ID };
static const size_t  NUM_VARS  = sizeof(VAR_IDS) / sizeof(VAR_IDS[0]);

// ---------- Fake device variables ----------
int16_t temperatureC = 25;  // °C
int16_t pressureKPa  = 101; // kPa
int16_t flowLpm      = 5;   // L/min

// ---------- Notify state (per variable) ----------
bool     notifyFlags[NUM_VARS] = { false, false, false }; // which vars are subscribed
uint32_t lastNotifyMillis      = 0;
const uint32_t NOTIFY_PERIOD_MS = 1000; // 1 second

// ---------- RX buffer ----------
#define RX_BUFFER_SIZE 128
uint8_t rxBuffer[RX_BUFFER_SIZE];
size_t  rxLen = 0;

// ---------- Utility: CRC (XOR of bytes from start..end) ----------
uint8_t calcCRC(const uint8_t* buf, size_t startIndex, size_t endIndexInclusive) {
  uint8_t crc = 0;
  for (size_t i = startIndex; i <= endIndexInclusive; i++) {
    crc ^= buf[i];
  }
  return crc;
}

// ---------- Utility: find index of varId in VAR_IDS ----------
int findVarIndex(uint8_t varId) {
  for (size_t i = 0; i < NUM_VARS; ++i) {
    if (VAR_IDS[i] == varId) return (int)i;
  }
  return -1;
}

// ---------- Send frame: SOF | LEN | TYPE | PAYLOAD | CRC | EOF ----------
void sendFrame(uint8_t type, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t len      = 1 + payloadLen;             // TYPE + PAYLOAD
  uint8_t totalLen = 1 + 1 + len + 1 + 1;        // SOF + LEN + (TYPE+PAYLOAD) + CRC + EOF

  if (totalLen > 32) {
    // Simple guard for demo
    return;
  }

  uint8_t frame[32];
  size_t i = 0;

  frame[i++] = SOF;
  frame[i++] = len;
  frame[i++] = type;

  for (uint8_t j = 0; j < payloadLen; j++) {
    frame[i++] = payload[j];
  }

  uint8_t crc = calcCRC(frame, 1, i - 1); // LEN..end of payload
  frame[i++] = crc;
  frame[i++] = EOF_BYTE;

  DeviceSerial.write(frame, i);
}

// ---------- Variable access helpers ----------
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

// ---------- Send NOTIFY_DATA for a variable ----------
void sendNotifyData(uint8_t varId) {
  int16_t value;
  if (!getVar(varId, value)) return;

  uint8_t payload[3];
  payload[0] = varId;
  payload[1] = (uint8_t)(value & 0xFF);        // LSB
  payload[2] = (uint8_t)((value >> 8) & 0xFF); // MSB

  sendFrame(CMD_NOTIFY_DATA, payload, 3);
}

// ---------- Handle one complete valid frame ----------
void handleFrame(const uint8_t* frame, size_t frameLen) {
  uint8_t len = frame[1];
  uint8_t type = frame[2];
  const uint8_t* payload = &frame[3];
  size_t payloadLen = len - 1; // minus TYPE

  if (type == CMD_READ_VAR) {
    // Payload: [varId]
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
    // Payload: [varId, lo, hi]
    if (payloadLen < 3) return;
    uint8_t varId = payload[0];
    uint8_t lo    = payload[1];
    uint8_t hi    = payload[2];
    int16_t newVal = (int16_t)((hi << 8) | lo);
    if (setVar(varId, newVal)) {
      // Confirm write via READ_RESPONSE
      uint8_t outPayload[3];
      outPayload[0] = varId;
      outPayload[1] = lo;
      outPayload[2] = hi;
      sendFrame(CMD_READ_RESPONSE, outPayload, 3);
    }
  }
  else if (type == CMD_NOTIFY_START) {
    // Payload: [varId]
    if (payloadLen < 1) return;
    uint8_t varId = payload[0];
    int idx = findVarIndex(varId);
    if (idx >= 0) {
      notifyFlags[idx] = true;          // subscribe this variable
      lastNotifyMillis = millis();      // reset timer
      // Optional: send one immediate sample
      sendNotifyData(varId);
    }
  }
  else if (type == CMD_NOTIFY_STOP) {
    // Payload: [varId] or [0xFF] to stop all
    if (payloadLen < 1) return;
    uint8_t varId = payload[0];

    if (varId == 0xFF) {
      // stop all
      for (size_t i = 0; i < NUM_VARS; ++i) {
        notifyFlags[i] = false;
      }
    } else {
      int idx = findVarIndex(varId);
      if (idx >= 0) {
        notifyFlags[idx] = false;       // unsubscribe this variable
      }
    }
  }
  else {
    // Unknown command type – ignore in this demo
  }
}

// ---------- Parse frames from rxBuffer ----------
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

    if (rxLen < MIN_FRAME) return;

    uint8_t len = rxBuffer[1];
    size_t totalFrameLen = 1 + 1 + len + 1 + 1; // SOF + LEN + (TYPE+PAYLOAD) + CRC + EOF

    if (totalFrameLen > RX_BUFFER_SIZE) {
      // Something's wrong; drop this SOF and continue
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

    // Check EOF
    if (frame[totalFrameLen - 1] != EOF_BYTE) {
      // Bad EOF; ignore
      continue;
    }

    // Check CRC
    uint8_t crcReceived = frame[totalFrameLen - 2];
    uint8_t crcComputed = calcCRC(frame, 1, totalFrameLen - 3); // LEN..payload end
    if (crcReceived != crcComputed) {
      // CRC error; ignore
      continue;
    }

    // Valid frame → handle it
    handleFrame(frame, totalFrameLen);
  }
}

// ---------- Arduino setup/loop ----------
void setup() {
  // Optional: USB debug serial
  Serial.begin(115200);

  // UART2 for protocol
  DeviceSerial.begin(BAUD_RATE, SERIAL_8N1, ESP32_RX2, ESP32_TX2);
}

void loop() {
  // --- RX handling from UART2 ---
  while (DeviceSerial.available() > 0) {
    if (rxLen < RX_BUFFER_SIZE) {
      rxBuffer[rxLen++] = (uint8_t)DeviceSerial.read();
    } else {
      // Overflow -> reset buffer
      rxLen = 0;
    }
  }

  if (rxLen > 0) {
    parseFramesFromBuffer();
  }

  // --- Multi-variable periodic notify ---
  uint32_t now = millis();
  if (now - lastNotifyMillis >= NOTIFY_PERIOD_MS) {
    lastNotifyMillis = now;

    for (size_t i = 0; i < NUM_VARS; ++i) {
      if (notifyFlags[i]) {
        uint8_t varId = VAR_IDS[i];
        sendNotifyData(varId);
      }
    }
  }

  // --- Optional: simulate changing temperature (for graph demo) ---
  static uint32_t lastTempUpdate = 0;
  if (now - lastTempUpdate > 3000) {
    lastTempUpdate = now;
    temperatureC++;
    if (temperatureC > 35) {
      temperatureC = 25;
    }
  }
}
