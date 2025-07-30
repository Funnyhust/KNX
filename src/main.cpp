#include <Arduino.h>
#include "stknx.h"

#define DEBUG_SERIAL Serial2

HardwareTimer timer(TIM2);

// Ví dụ callback xử lý KNX telegram
void handle_knx_frame(const uint8_t *frame, uint8_t len) {
  DEBUG_SERIAL.print("KNX frame len=");
  DEBUG_SERIAL.println(len);
  for (uint8_t i = 0; i < len; i++) {
    DEBUG_SERIAL.printf("%02X ", frame[i]);
  }
  DEBUG_SERIAL.println();
}

void setup() {
  Serial2.setRx(PB11);  // RX chân PB11
  Serial2.setTx(PB10);  // TX chân PB10
  DEBUG_SERIAL.begin(115200);

  // Cấu hình chân (PA10) làm input EXTI cho KNX_RX
  pinMode(KNX_TX_PIN, INPUT);

  // Cấu hình Timer2 tạo ngắt mỗi 104µs
  timer.setPrescaleFactor(72);     // 72MHz/72 = 1MHz => 1us tick
  timer.setOverflow(104);          // 104µs bit period
  timer.attachInterrupt(knx_timer_tick);
  timer.refresh();
  timer.resume();

  // Gắn callback xử lý cạnh EXTI
  attachInterrupt(digitalPinToInterrupt(KNX_TX_PIN), knx_exti_irq, CHANGE);

  // Khởi tạo thư viện với callback
  knx_init(handle_knx_frame);
}

void loop() {
  // Thư viện hoạt động ngầm, xử lý trong ISR
  // Chỉ cần loop trống
  DEBUG_SERIAL.print("Test Serial!\n");
  delay(2000);
}
  