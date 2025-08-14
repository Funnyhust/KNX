#include <Arduino.h>
#include "stknx.h"

#define UART_RX_BUFFER_SIZE 23
uint8_t uart_rx_buf[UART_RX_BUFFER_SIZE];

#if defined(KNX_MODE_FRAME)
void handle_knx_frame(const uint8_t *frame, uint8_t len) {
    DEBUG_SERIAL.write(frame, len);
    delay(10);
    attachInterrupt(digitalPinToInterrupt(KNX_TX_PIN), knx_exti_irq, CHANGE);
}
#elif defined(KNX_MODE_BYTE)
void handle_knx_frame(const uint8_t byte) {
  DEBUG_SERIAL.write(byte);
}
#endif

void setup() {
  DEBUG_SERIAL.begin(19200,SERIAL_8E1); //paryty is even
  knx_init(handle_knx_frame);
  enableDWT();
 }

bool read_uart_frame() {
  static uint8_t index = 0;
  static uint8_t total = 0;

  while (DEBUG_SERIAL.available()) {
    uint8_t byte = DEBUG_SERIAL.read();
    if (index < UART_RX_BUFFER_SIZE) {
      uart_rx_buf[index++] = byte;
    } else {
      // Tràn buffer, reset lại
      index = 0;
      total = 0;
      return false;
    }

    if (index == 6) {
      total = 6 + (uart_rx_buf[5] & 0x0F) + 1 + 1;
      if (total > UART_RX_BUFFER_SIZE) {
        index = 0;
        total = 0;
        return false;
      }
    }

    if (total > 0 && index >= total) {
      // Có thể kiểm tra checksum ở đây nếu muốn
      index = 0;
      total = 0;
      return true;
    }
  }
  return false;
}

void loop() {
  if (read_uart_frame()) {
    sendKNXBytes(uart_rx_buf);  // Gửi ra KNX bus
    uart_rx_buf[0] = 0; // Reset buffer sau khi gửi
  }
}



