#include "stknx.h"
#include <Arduino.h>

#define BIT0_MIN_US 25
#define BIT0_MAX_US 45
#define KNX_MAX_FRAME_LEN 35

extern HardwareTimer timer;

static uint8_t buf[KNX_MAX_FRAME_LEN];
static uint8_t bit_idx = 0, byte_idx = 0, cur_byte = 0;
static volatile bool bit0 = false;
static uint32_t pulse_start = 0;
static knx_frame_callback_t callback_fn = nullptr;
static knx_frame_callback_t_2 callback_fn_2 = nullptr;


static bool RX_flag=false;
static bool total_flag=false;
static uint8_t payload_len = 0;
static uint8_t total = 0;
static uint8_t bit_sum= 0;

void enableDWT() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us_10x(uint32_t t) {
  // Mỗi đơn vị t là 0.1 µs => cần nhân với 0.1µs → tổng µs = t / 10
  uint32_t cycles = (SystemCoreClock / 64000000L) * t;  // vì 10 MHz tick = 0.1µs
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles);
}




void knx_init(knx_frame_callback_t_2 cb) {
  callback_fn_2 = cb;
  bit_idx = byte_idx = cur_byte = 0;
  bit0 = false;
  pulse_start = 0;
}
void knx_exti_irq(void) {
  if(!RX_flag){
      RX_flag = true;
      timer.refresh();
      timer.resume();
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
void knx_timer_tick(void) {
  uint8_t bit = bit0 ? 0 : 1;
  bit0 = false;  // Reset flag sau khi dùng
  bit_idx++;

  if (bit_idx >= 2 && bit_idx <= 9) {
    cur_byte = (cur_byte >> 1) | (bit << 7);  // Lấy bit dữ liệu
  }

  if (bit_idx == 11 && bit == 1) {  // Stop bit = 1
    if (callback_fn_2) callback_fn_2(cur_byte);
    cur_byte = 0;
    bit_idx = 0;
    byte_idx++;
    timer.pause();
    RX_flag = false;
  }

  if (byte_idx == 20) {
    byte_idx = 0;
    DEBUG_SERIAL.println();
  }
}



// void knx_timer_tick(void) {
//   if(RX_flag){
//   uint8_t bit = bit0 ? 0 : 1;
//   cur_byte = (cur_byte >> 1) | (bit << 7);
//   bit_idx++;
//   bit0 = false;
//   if((byte_idx==0)){
//     if(bit_idx==9)
//     buf[byte_idx++] = cur_byte;
//     bit_idx = 0;
//     cur_byte = 0;
//   }
    

//   else {
//     if (bit_idx == 8) {
//     buf[byte_idx++] = cur_byte;
//     bit_idx = 0;
//     cur_byte = 0;
//     if (byte_idx >= 6) {
//       if(!total_flag){
//       total_flag =true;
//       payload_len = buf[5] & 0x0F;
//       total = 6 + payload_len + 20;
//       }
//         if (total <= KNX_MAX_FRAME_LEN && byte_idx == total) {
//         callback_fn(buf, total);
//         byte_idx = 0;
//         // KNX_ready = true;
//         RX_flag = false;
//         total_flag=false;
//     }
//   }
//   }  
//   }

// }
// }


// static void sendKNXBit(bool bitVal) {
//   uint32_t start = micros();

//   if (bitVal == 0) {
//     digitalWrite(KNX_RX_PIN, HIGH);
//     while(micros()-start<35);
//     digitalWrite(KNX_RX_PIN, LOW);
//     while(micros()-start<69);
//   } else {
//     digitalWrite(KNX_RX_PIN, LOW);
//     while(micros()-start<104);
//   }
// }

// static void sendKNXBit(bool bitVal) {
//   if (bitVal == 0) {
//     GPIOA->BSRR = GPIO_BSRR_BS10;
//     delay_us(35);
//     GPIOA->BRR  = GPIO_BRR_BR10; 
//     delay_us(69); // tổng = 104
//   } else {
//     GPIOA->BRR  = GPIO_BRR_BR10; 
//     delay_us(104);
//   }
// }
static void sendKNXBits(uint8_t bitVal){
          if (bitVal == 0) {
           GPIOA->BSRR = (1 << 10);
           delay_us_10x(35*64-64-16);
           GPIOA->BSRR = (1 << (10 + 16));
           delay_us_10x(69*64-32-8); // tổng = 104
        } else {
           GPIOA->BSRR = (1 << (10 + 16));
           delay_us_10x(104*64-64);
        }
}

void sendKNXBytes(uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byteToSend = data[i];
    sendKNXBits(0);
    for (int b = 0; b <= 7; b++) {
      bool bitVal = (byteToSend >> b) & 0x01;
      sendKNXBits(bitVal);
      if(bitVal) bit_sum++;
    }
    sendKNXBits(bit_sum);
    bit_sum=0;
    sendKNXBits(1);
    sendKNXBits(1);
    sendKNXBits(1);
  }
}


        //    digitalWrite(KNX_RX_PIN, HIGH);
        //    delay_us_10x(35*64-64*3+8);
        //    digitalWrite(KNX_RX_PIN, LOW);
        //    delay_us_10x(69*64-32); // tổng = 104
        // } else {
        //    digitalWrite(KNX_RX_PIN, LOW);
        //    delay_us_10x(104*64-64*3);
        // }
