#pragma once

#include "ad7606.h"
#include "main.h"
#include "si5351.h"
#include "stdio.h"
#include "usart.h"
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

typedef enum {
  MONO_SHORT,
  MONO_SHIELD,
  MONO_LENGTH,
  DUAL_SHIELD,
  DUAL_CONN,
  DUAL_RES,
  DUAL_ATTEN
} RELAY_MODE;
extern volatile RELAY_MODE relay_mode;

typedef enum { CAL_UTP, CAL_SFTP } CAL_MODE;
extern volatile CAL_MODE cal_mode;

extern volatile float cable_len;
extern volatile double shield_cable_vfk, shield_cable_vfb;
extern volatile double nonshield_cable_vfk, nonshield_cable_vfb;
extern volatile double y_sftp;

typedef enum { AD7606_DMA_BUSY, AD7606_DMA_FREE } AD7606_DMA_STATUS;
extern volatile AD7606_DMA_STATUS ad7606_dma_status;

#define TX_BUF_LEN 256 /* 发送缓冲区容量，根据需要进行调整 */
void uartprint(UART_HandleTypeDef *huart, const char *__format, ...);

typedef enum { SI5351_TX = 0, SI5351_LO, SI5351_CLK2 } CLK_CHANNEL;
void setSi5351Freq(CLK_CHANNEL clk_chan, int32_t freq);
void ad7606_Sample(AD7606_Handler *ad7606, int16_t *bufferi, float *bufferf,
                   uint16_t sample_length);

void remove_dc(float *data, int n);
