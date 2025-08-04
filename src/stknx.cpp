#include "stknx.h"
#include <Arduino.h>

#define BIT0_MIN_US 25
#define BIT0_MAX_US 45
#define KNX_MAX_FRAME_LEN 23

#define KNX_BIT0_HIGH_US 35*64-64-16
#define KNX_BIT0_LOW_US  69*64-32-8
#define KNX_BIT1_LOW_US  104*64-64

extern HardwareTimer timer;

static uint8_t buf[KNX_MAX_FRAME_LEN];
static uint8_t bit_idx = 0, byte_idx = 0, cur_byte = 0;
static volatile bool bit0 = false;
static uint32_t pulse_start = 0;
static knx_frame_callback_t callback_fn = nullptr;
static knx_frame_callback_t_2 callback_fn_2 = nullptr;
static volatile uint8_t parity_bit = false;


static volatile bool RX_flag=false;
static uint8_t total = 0;

void enableDWT() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t knx_checksum(uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= data[i];  // XOR từng byte
    }
    return sum;
}

void delay_us_10x(uint32_t t) {
  // Mỗi đơn vị t là 0.1 µs => cần nhân với 0.1µs → tổng µs = t / 10
  uint32_t cycles = (SystemCoreClock / 64000000L) * t;  // vì 10 MHz tick = 0.1µs
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles);
}
void knx_exti_irq(void) {
  if(!RX_flag){
      RX_flag = true;
      timer.refresh();
      timer.resume(); // Bật lại timer để bắt đầu nhận dữ liệu
  }
  static uint8_t last = 0;
  uint8_t lvl = digitalRead(KNX_TX_PIN); // dùng chân D2 làm KNX_RX
  uint32_t now = micros();
  if (lvl && !last) pulse_start = now;
  else if (!lvl && last) {
    uint32_t w = now >= pulse_start ? now - pulse_start : 0;
    if (w >= BIT0_MIN_US && w <= BIT0_MAX_US){
         bit0 = true;
        //  DEBUG_SERIAL.printf("%lu",w);
    }   
  }
  last = lvl;
}
void reset_knx_receiver() {
  bit_idx = 0;
  byte_idx = 0;
  total = 0;
  cur_byte = 0;
  bit0 = false;
  RX_flag = false;
  for (uint8_t i = 0; i < KNX_MAX_FRAME_LEN; i++) {
    buf[i] = 0;
    }
}
void knx_timer_tick(void) {
  uint8_t bit = bit0 ? 0 : 1;
  bit0 = false;
  bit_idx++;

  static uint8_t parity_bit = 0;

  if (bit_idx == 1) {
    // Start bit
    cur_byte = 0;
    parity_bit = 0;
  } 
  else if (bit_idx >= 2 && bit_idx <= 9) {
    cur_byte >>= 1;
    if (bit) {
      cur_byte |= 0x80;
      parity_bit++;
    }
  } 
  else if (bit_idx == 10) {
    // Parity bit: kiểm tra bit lẻ
    if ((parity_bit & 1) != bit) {
      DEBUG_SERIAL.println("Parity error!");
      reset_knx_receiver();
      return;
    }
  } 
  else if (bit_idx == 11) {
    // Stop bit
    if (byte_idx < KNX_MAX_FRAME_LEN) {
      buf[byte_idx++] = cur_byte;

      if (byte_idx == 6) {
        total = 6 + (buf[5] & 0x0F) + 1 + 1;
        if (total == 0 || total > KNX_MAX_FRAME_LEN) {
          DEBUG_SERIAL.println("Invalid frame length!");
          reset_knx_receiver();
          return;
        }
      }

      if (total && byte_idx == total) {
        if (callback_fn) callback_fn(buf, total);
        reset_knx_receiver();
        return;
      }

    } else {
      DEBUG_SERIAL.println("Frame overflow!");
      reset_knx_receiver();
      return;
    }

    // ✅ Sau mỗi byte: dừng timer, chờ EXTI đánh thức lại
    timer.pause();
    RX_flag = false;
    bit_idx = 0;
  } 
  else if (bit_idx > 11) {
    DEBUG_SERIAL.println("Unexpected bit index!");
    reset_knx_receiver();
  }
}


void knx_init(knx_frame_callback_t cb) {
  callback_fn = cb;
  bit_idx = byte_idx = cur_byte = 0;
  bit0 = false;
  pulse_start = 0;
}

static void sendKNXBits(uint8_t bitVal){
          if (bitVal == 0) {
           GPIOA->BSRR = (1 << 10); 
           delay_us_10x(35*64-64-16);
           GPIOA->BSRR = (1 << (10 + 16));
           delay_us_10x(KNX_BIT0_LOW_US); // tổng = 104
        } else {
           GPIOA->BSRR = (1 << (10 + 16));
           delay_us_10x(104*64-64);
        }
}

void sendKNXBytes(uint8_t *data) {
  uint8_t bit_sum= 0;
  uint8_t len = 6 + data[5]&0x0F + 1 + 1; // Lấy độ dài payload từ byte thứ 6
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byteToSend = data[i];
    sendKNXBits(0);
    for (int b = 0; b <= 7; b++) {
      bool bitVal = (byteToSend >> b) & 0x01;
      sendKNXBits(bitVal);
      if(bitVal) bit_sum++;
    }
    sendKNXBits(bit_sum&1);
    bit_sum=0;
    sendKNXBits(1);
    sendKNXBits(1);
    sendKNXBits(1);
  }
}



  ///////////////////
  


        //    digitalWrite(KNX_RX_PIN, HIGH);
        //    delay_us_10x(35*64-64*3+8);
        //    digitalWrite(KNX_RX_PIN, LOW);
        //    delay_us_10x(69*64-32); // tổng = 104
        // } else {
        //    digitalWrite(KNX_RX_PIN, LOW);
        //    delay_us_10x(104*64-64*3);
        // }
