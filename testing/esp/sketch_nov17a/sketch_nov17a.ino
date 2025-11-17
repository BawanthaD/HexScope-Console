// ESP32 + UART2 (TX2/RX2) protocol implementation
// Frame: SOF | LEN | TYPE | PAYLOAD... | CRC | EOF
// SOF = 0xAA, EOF = 0x55, LEN = TYPE + PAYLOAD length
// CRC = XOR of bytes from LEN through last payload byte
//
// Types:
//   0x01 CMD_READ_VAR       (PC -> ESP)   payload: [varId]
//   0x02 CMD_WRITE_VAR      (PC -> ESP)   payload: [varId, lo, hi]
//   0x81 CMD_READ_RESPONSE  (ESP -> PC)   payload: [varId, lo, hi]
//   0x82 CMD_NOTIFY_DATA    (ESP -> PC)   payload: [notifyId, data...]
//
// Example variables (int16 raw + scaling):
//   16: Voltage     (decivolts, scale 0.1)
//   17: Current     (centiamps, scale 0.01)
//   18: Temperature (°C, scale 1.0)

#include <Arduino.h>

/* -------------------- UART2 config -------------------- */

HardwareSerial Uart(2);                   // Use UART2

const uint8_t RX2_PIN = 16;               // ESP32 RX2 pin (connect to USB-UART TX)
const uint8_t TX2_PIN = 17;               // ESP32 TX2 pin (connect to USB-UART RX)

static const uint32_t SERIAL_BAUD = 115200;

/* -------------------- Protocol constants -------------------- */

static const uint8_t SOF = 0xAA;
static const uint8_t EOF_ = 0x55;

static const uint8_t CMD_READ_VAR       = 0x01;
static const uint8_t CMD_WRITE_VAR      = 0x02;
static const uint8_t CMD_READ_RESPONSE  = 0x81;
static const uint8_t CMD_NOTIFY_DATA    = 0x82;

// Example: one notify channel, ID = 1
static const uint8_t NOTIFY_ID_MAIN = 0x01;

/* -------------------- Variables & IDs -------------------- */

static const uint8_t VAR_ID_VOLTAGE     = 16;  // 0x10
static const uint8_t VAR_ID_CURRENT     = 17;  // 0x11
static const uint8_t VAR_ID_TEMPERATURE = 18;  // 0x12

// Internal storage as int16 raw
// Voltage in decivolts (0.1 V), Current in centiamps (0.01 A), Temperature in °C
int16_t voltage_raw     = 230;  // 23.0 V
int16_t current_raw     = 150;  // 1.50 A
int16_t temperature_raw = 250;  // 25.0 °C

/* -------------------- Frame builder helpers -------------------- */

// XOR CRC from data[start]..data[end]
uint8_t crcXor(const uint8_t* data, size_t start, size_t end) {
  uint8_t crc = 0;
  for (size_t i = start; i <= end; ++i) {
    crc ^= data[i];
  }
  return crc;
}

// Build: SOF | LEN | TYPE | PAYLOAD... | CRC | EOF
// LEN = TYPE + PAYLOAD length
size_t buildFrame(uint8_t type, const uint8_t* payload, size_t payloadLen,
                  uint8_t* outBuf, size_t outBufSize) {
  const size_t lenField = 1 + payloadLen;                // TYPE + PAYLOAD
  const size_t frameLen = 1 + 1 + lenField + 1 + 1;      // SOF + LEN + [TYPE+PAYLOAD] + CRC + EOF

  if (frameLen > outBufSize) {
    return 0; // not enough space
  }

  size_t i = 0;
  outBuf[i++] = SOF;
  outBuf[i++] = (uint8_t)lenField;
  outBuf[i++] = type;

  for (size_t j = 0; j < payloadLen; ++j) {
    outBuf[i++] = payload[j];
  }

  // CRC over LEN..last payload byte = indexes [1 .. 1+lenField]
  uint8_t crc = crcXor(outBuf, 1, 1 + lenField);
  outBuf[i++] = crc;
  outBuf[i++] = EOF_;

  return frameLen;
}

/* -------------------- Serial RX frame parser -------------------- */

enum RxState {
  RX_WAIT_SOF,
  RX_READ_LEN,
  RX_READ_BODY
};

RxState rxState = RX_WAIT_SOF;
uint8_t rxBuf[128];
size_t  rxIndex    = 0;
uint8_t rxLenField = 0;
size_t  rxFrameLen = 0;

void handleFrame(const uint8_t* frame, size_t len);
void processIncomingByte(uint8_t b);

void processSerial() {
  while (Uart.available() > 0) {
    uint8_t b = (uint8_t)Uart.read();
    processIncomingByte(b);
  }
}

void processIncomingByte(uint8_t b) {
  switch (rxState) {
    case RX_WAIT_SOF:
      if (b == SOF) {
        rxBuf[0] = b;
        rxIndex = 1;
        rxState = RX_READ_LEN;
      }
      break;

    case RX_READ_LEN:
      rxBuf[1] = b;
      rxLenField = b;
      // Total frame length = 1(SOF) + 1(LEN) + lenField + 1(CRC) + 1(EOF)
      rxFrameLen = 1 + 1 + rxLenField + 1 + 1;
      if (rxFrameLen > sizeof(rxBuf)) {
        // invalid, reset
        rxState = RX_WAIT_SOF;
        rxIndex = 0;
      } else {
        rxIndex = 2;
        rxState = RX_READ_BODY;
      }
      break;

    case RX_READ_BODY:
      rxBuf[rxIndex++] = b;
      if (rxIndex >= rxFrameLen) {
        // we have a full frame
        handleFrame(rxBuf, rxFrameLen);
        // reset for next frame
        rxState = RX_WAIT_SOF;
        rxIndex = 0;
      }
      break;
  }
}

void handleFrame(const uint8_t* frame, size_t len) {
  if (len < 5) return;  // too short to be valid

  // Check EOF
  if (frame[len - 1] != EOF_) {
    // bad EOF
    return;
  }

  uint8_t lenField = frame[1];
  uint8_t type = frame[2];
  size_t payloadLen = (size_t)lenField - 1;  // subtract TYPE
  if (3 + payloadLen + 2 != len) {
    // length mismatch
    return;
  }

  size_t crcIndex = len - 2;
  uint8_t crcReceived = frame[crcIndex];
  uint8_t crcComputed = crcXor(frame, 1, 1 + lenField); // [LEN .. last payload]
  if (crcReceived != crcComputed) {
    // CRC error
    return;
  }

  const uint8_t* payload = &frame[3];

  switch (type) {
    case CMD_READ_VAR:
      if (payloadLen >= 1) {
        {
          uint8_t varId = payload[0];
          int16_t value = 0;
          bool known = true;

          switch (varId) {
            case VAR_ID_VOLTAGE:     value = voltage_raw;     break;
            case VAR_ID_CURRENT:     value = current_raw;     break;
            case VAR_ID_TEMPERATURE: value = temperature_raw; break;
            default: known = false;  break;
          }

          if (known) {
            uint8_t outPayload[3];
            outPayload[0] = varId;
            outPayload[1] = (uint8_t)(value & 0xFF);        // LSB
            outPayload[2] = (uint8_t)((value >> 8) & 0xFF); // MSB

            uint8_t outFrame[16];
            size_t outLen = buildFrame(CMD_READ_RESPONSE,
                                       outPayload, 3,
                                       outFrame, sizeof(outFrame));
            if (outLen > 0) {
              Uart.write(outFrame, outLen);
            }
          }
        }
      }
      break;

    case CMD_WRITE_VAR:
      if (payloadLen >= 3) {
        uint8_t varId = payload[0];
        uint8_t lo = payload[1];
        uint8_t hi = payload[2];
        int16_t newVal = (int16_t)((hi << 8) | lo);

        switch (varId) {
          case VAR_ID_VOLTAGE:     voltage_raw     = newVal; break;
          case VAR_ID_CURRENT:     current_raw     = newVal; break;
          case VAR_ID_TEMPERATURE: temperature_raw = newVal; break;
          default: break; // unknown ID
        }

        // Optionally send back a READ_RESPONSE as ACK if you want.
      }
      break;

    default:
      // other types not handled (no PC->ESP notify control in new design)
      break;
  }
}

/* -------------------- Notify (device → PC) -------------------- */

// Send a notify frame every 1000 ms with:
// TYPE = CMD_NOTIFY_DATA (0x82)
// payload = [notifyId, Vlo, Vhi, Clo, Chi, Tlo, Thi]

unsigned long lastNotifyMs = 0;
const unsigned long NOTIFY_INTERVAL_MS = 1000;

void sendNotifyFrame() {
  // Example: simulate some changing data
  static int16_t angle = 0;
  angle = (angle + 10) % 360;

  // Change values slightly over time
  voltage_raw     = 220 + (int16_t)(10 * sin(angle * DEG_TO_RAD));            // around 22 V
  current_raw     = 100 + (int16_t)(20 * cos(angle * DEG_TO_RAD));            // around 1.0 A
  temperature_raw = 250 + (int16_t)(2 * sin(angle * 0.5f * DEG_TO_RAD));      // around 25°C

  uint8_t payload[1 + 2 + 2 + 2];
  size_t idx = 0;

  payload[idx++] = NOTIFY_ID_MAIN;

  // Voltage (LSB first)
  payload[idx++] = (uint8_t)(voltage_raw & 0xFF);
  payload[idx++] = (uint8_t)((voltage_raw >> 8) & 0xFF);

  // Current
  payload[idx++] = (uint8_t)(current_raw & 0xFF);
  payload[idx++] = (uint8_t)((current_raw >> 8) & 0xFF);

  // Temperature
  payload[idx++] = (uint8_t)(temperature_raw & 0xFF);
  payload[idx++] = (uint8_t)((temperature_raw >> 8) & 0xFF);

  uint8_t outFrame[32];
  size_t frameLen = buildFrame(CMD_NOTIFY_DATA, payload, idx, outFrame, sizeof(outFrame));
  if (frameLen > 0) {
    Uart.write(outFrame, frameLen);
  }
}

/* -------------------- Arduino setup/loop -------------------- */

void setup() {
  // UART2: RX on 16, TX on 17
  Uart.begin(SERIAL_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(500);

  // Initial values
  voltage_raw     = 230; // 23.0 V
  current_raw     = 150; // 1.50 A
  temperature_raw = 250; // 25.0 °C
}

void loop() {
  processSerial();

  unsigned long now = millis();
  if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
    lastNotifyMs = now;
    sendNotifyFrame();
  }

  // Other application code...
}
