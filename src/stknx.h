#ifndef STKNX_H
#define STKNX_H

#include <stdint.h>
#include <stdbool.h>

#define KNX_TX_PIN PA10
#define KNX_RX_PIN PA9
// Callback: nhận toàn bộ telegram (frame + checksum) với độ dài hợp lệ
typedef void (*knx_frame_callback_t)(const uint8_t *frame, uint8_t len);
     
// Khởi tạo: truyền vào callback xử lý telegram
void knx_init(knx_frame_callback_t cb);

// Hàm gọi trong EXTI IRQ (knx RX edge)
void knx_exti_irq(void);

// Hàm gọi trong Timer IRQ 104µs (bit sampling)
void knx_timer_tick(void);

void sendKNXBytes(uint8_t *data, uint8_t len);

#endif // STKNX_DRIVER_H
