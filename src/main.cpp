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
    DEBUG_SERIAL.write(frame, len);
    delay(10);
    attachInterrupt(digitalPinToInterrupt(KNX_TX_PIN), knx_exti_irq, CHANGE);
}
// Ví dụ callback xử lý từng byte (nếu cần)
void handle_knx_frame_2(const uint8_t byte) {
  DEBUG_SERIAL.printf("%02X ", byte);
}

void setup() {
  // Serial2.setRx(PB3);  // RX chân PB11
  // Serial2.setTx(PB2);  // TX chân PB10
  DEBUG_SERIAL.begin(19200,SERIAL_8E1);
  timer.setPrescaleFactor(64);     // 72MHz/72 = 1MHz => 1us tick
  timer.setOverflow(104);          // 104µs bit period
  timer.attachInterrupt(knx_timer_tick);
  // Gắn callback xử lý cạnh EXTI
  attachInterrupt(digitalPinToInterrupt(KNX_TX_PIN), knx_exti_irq, CHANGE);
  
  // Cấu hình chân (PA10) làm input EXTI cho KNX_RX
  pinMode(KNX_TX_PIN, INPUT);
  pinMode(KNX_RX_PIN, OUTPUT);

  knx_init(handle_knx_frame);
  // knx_init(handle_knx_frame); // Nếu bạn muốn nhận toàn bộ frame

  enableDWT();
  // Bật DWT để sử dụng hàm delay_us_10x
 }
void loop() {
  // if (read_uart_frame()) {
  //   sendKNXBytes(uart_rx_buf);
  // }
  // delay(2000);
}

  // DEBUG_SERIAL.print("Test Serial!\n");
  // DEBUG_SERIAL.println(F_CPU);          
  // DEBUG_SERIAL.println(SystemCoreClock);  

  //Vấn đề : sau khi callback thì vẫn có xung gì đó trên đường truyề n khiến timer vẫn tiếp tục chạy chứ chưa dừng hẳn
  //cần tìm giải pháp để dừng timer sau khi nhận xong frame,