#ifndef STKNX_H
#define STKNX_H

#define DEBUG_SERIAL Serial2

#include <stdint.h>
#include <stdbool.h>

#define KNX_TX_PIN PA9
#define KNX_RX_PIN PA10

//#define KNX_MODE_FRAME   1
#define KNX_MODE_BYTE  1   // Bật cái này nếu muốn xử lý từng byte
// Callback: nhận toàn bộ telegram (frame + checksum) với độ dài hợp lệ
#if defined(KNX_MODE_FRAME)
typedef void (*knx_frame_callback_t)(const uint8_t *frame, uint8_t len);
#elif defined(KNX_MODE_BYTE)
typedef void (*knx_frame_callback_t)(const uint8_t byte);
#endif
     
// Khởi tạo: truyền vào callback xử lý telegram
void knx_init(knx_frame_callback_t cb);

void knx_exti_irq(void);
// Hàm gọi trong Timer IRQ 104µs (bit sampling)
void knx_timer_tick(void);

void sendKNXBytes(uint8_t *data);

void enableDWT();

#endif // STKNX_DRIVER_H
