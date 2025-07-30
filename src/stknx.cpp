#include "stknx.h"
#include <Arduino.h>

#define BIT0_MIN_US 25
#define BIT0_MAX_US 45
#define KNX_MAX_FRAME_LEN 23

static uint8_t buf[KNX_MAX_FRAME_LEN];
static uint8_t bit_idx = 0, byte_idx = 0, cur_byte = 0;
static volatile bool bit0 = false;
static uint32_t pulse_start = 0;
static knx_frame_callback_t callback_fn = nullptr;

void knx_init(knx_frame_callback_t cb) {
  callback_fn = cb;
  bit_idx = byte_idx = cur_byte = 0;
  bit0 = false;
  pulse_start = 0;
}

void knx_exti_irq(void) {
  static uint8_t last = 0;
  uint8_t lvl = digitalRead(KNX_TX_PIN); // dùng chân D2 làm KNX_RX
  uint32_t now = micros();
  if (lvl && !last) pulse_start = now;
  else if (!lvl && last) {
    uint32_t w = now >= pulse_start ? now - pulse_start : 0;
    if (w >= BIT0_MIN_US && w <= BIT0_MAX_US) bit0 = true;
  }
  last = lvl;
}

void knx_timer_tick(void) {
  uint8_t bit = bit0 ? 0 : 1;
  cur_byte = (cur_byte >> 1) | (bit << 7);
  bit_idx++;
  bit0 = false;

  if (bit_idx == 8) {
    if (byte_idx < KNX_MAX_FRAME_LEN) buf[byte_idx++] = cur_byte;
    bit_idx = 0;
    cur_byte = 0;

    if (byte_idx >= 6) {
      uint8_t payload_len = buf[5] & 0x0F;
      uint8_t total = 6 + payload_len + 1;
      if (total <= KNX_MAX_FRAME_LEN && byte_idx == total) {
        uint8_t sum = 0;
        for (int i = 0; i < total - 1; i++) sum ^= buf[i];
        sum = ~sum;
        if (sum == buf[total - 1] && callback_fn) {
          callback_fn(buf, total);
        }
        byte_idx = 0;
      }
    }
  }
}


static void sendKNXBit(bool bitVal) {
  uint32_t start = micros();

  if (bitVal == 0) {
    digitalWrite(KNX_RX_PIN, LOW);
    while (micros() - start < 35);  // giữ LOW 35 µs
    digitalWrite(KNX_RX_PIN, HIGH);
    while (micros() - start < 104); // còn lại là HIGH
  } else {
    digitalWrite(KNX_RX_PIN, HIGH);
    while (micros() - start < 104); // giữ HIGH toàn bộ 104 µs
  }
}


void sendKNXBytes(uint8_t *data, uint8_t len) {
  noInterrupts();  // tắt ngắt để giữ chính xác thời gian

  for (uint8_t i = 0; i < len; i++) {
    uint8_t byteToSend = data[i];

    // Gửi từ bit MSB đến LSB
    for (int b = 7; b >= 0; b--) {
      bool bitVal = (byteToSend >> b) & 0x01;
      sendKNXBit(bitVal);
    }
  }

  interrupts(); // bật lại ngắt sau khi gửi
}
