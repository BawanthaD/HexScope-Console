#include <Arduino.h>

const uint32_t BAUD_RATE      = 115200;
const uint32_t SEND_INTERVAL  = 1000;   // 1 second

uint32_t lastSend = 0;

HardwareSerial Uart(2);   // UART2
const int RX2_PIN = 16;
const int TX2_PIN = 17;

void setup() {
  Uart.begin(BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(1000);
  randomSeed(analogRead(34));
}

void loop() {
  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    // Generate three random values (change ranges as you like)
    int val1 = random(20, 30);   // e.g. "temperature"
    int val2 = random(40, 80);   // e.g. "pressure"
    int val3 = random(0, 100);   // e.g. "some sensor"

      Uart.print(val1);
      Uart.print(",");
      Uart.print(val2);
      Uart.print(",");
      Uart.print(val3);
      Uart.print("\n");

  }


}
