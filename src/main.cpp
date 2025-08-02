#include <Arduino.h>
#include "stknx.h"


HardwareTimer timer(TIM2);

// Frame ON đã capture được trước đó
uint8_t on_frame[9] = {
  0xBC, 0x12, 0x01, 0x00, 0x04, 0xE1,  0x00, 0x80, 0x35
};

// Frame OFF tương ứng (ví dụ bạn capture riêng)
uint8_t off_frame[9] = {
  0xBC, 0x12, 0x01, 0x00, 0x04, 0xE1,  0x00, 0x81, 0x34
};



// Ví dụ callback xử lý KNX telegram
void handle_knx_frame(const uint8_t *frame, uint8_t len) {
  DEBUG_SERIAL.print("KNX frame len=");
  DEBUG_SERIAL.println(len);
  for (uint8_t i = 0; i < len; i++) {
    DEBUG_SERIAL.printf("%02X ", frame[i]);
  }
  DEBUG_SERIAL.println();
}

void handle_knx_frame_2(const uint8_t byte) {
  DEBUG_SERIAL.printf("%02X ", byte);
}

void setup() {
  // Serial2.setRx(PB3);  // RX chân PB11
  // Serial2.setTx(PB2);  // TX chân PB10
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.print("Start");

  // Cấu hình chân (PA10) làm input EXTI cho KNX_RX
  pinMode(KNX_TX_PIN, INPUT);
  pinMode(KNX_RX_PIN, OUTPUT);

  // // Cấu hình Timer2 tạo ngắt mỗi 104µs
  timer.setPrescaleFactor(64);     // 72MHz/72 = 1MHz => 1us tick
  timer.setOverflow(104);          // 104µs bit period
  timer.attachInterrupt(knx_timer_tick);

  //Gắn callback xử lý cạnh EXTI
  attachInterrupt(digitalPinToInterrupt(KNX_TX_PIN), knx_exti_irq, CHANGE);

  // Khởi tạo thư viện với callback
  knx_init(handle_knx_frame_2);
  // DEBUG_SERIAL.print("Start");
  // DEBUG_SERIAL.println(F_CPU);             // in ra giá trị bạn khai báo (72000000L)
  // DEBUG_SERIAL.println(SystemCoreClock);  
  enableDWT();
 }

void loop() {
  // Thư viện hoạt động ngầm, xử lý trong ISR
  // Chỉ cần loop trống
  // DEBUG_SERIAL.print("Test Serial!\n");
  // DEBUG_SERIAL.println(F_CPU);             // in ra giá trị bạn khai báo (72000000L)
  // DEBUG_SERIAL.println(SystemCoreClock);  
  sendKNXBytes(on_frame, 9);  // Bật
  delay(3000);
  sendKNXBytes(off_frame, 9); // Tắt
  delay(3000);
  // delay(2000);
}
