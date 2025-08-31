#include "user.h"

volatile AD7606_DMA_STATUS ad7606_dma_status = AD7606_DMA_FREE;
volatile RELAY_MODE relay_mode = MONO_LENGTH;
volatile CAL_MODE cal_mode = CAL_UTP;
volatile float cable_len = 0;
volatile double shield_cable_vfk = 1.1364, shield_cable_vfb = -0.22;
volatile double nonshield_cable_vfk = 1.054, nonshield_cable_vfb = -0.22;
// volatile double nonshield_cable_vfk = 1.1364, nonshield_cable_vfb = -0.22;
volatile double y_sftp = 0;

uint8_t TxBuf[TX_BUF_LEN]; /* 发送缓冲区                       */
void uartprint(UART_HandleTypeDef *huart, const char *__format, ...) {
  va_list ap;
  va_start(ap, __format);

  /* 清空发送缓冲区 */
  memset(TxBuf, 0x0, TX_BUF_LEN);

  /* 填充发送缓冲区 */
  vsnprintf((char *)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);
  int len = strlen((const char *)TxBuf);

  /* 往串口发送数据 */
  HAL_UART_Transmit(huart, (uint8_t *)&TxBuf, len, 0xFFFF);
}

void setSi5351Freq(CLK_CHANNEL clk_chan, int32_t freq) {
  si5351PLLConfig_t pll_conf;
  si5351OutputConfig_t out_conf;

  si5351_Calc(freq, &pll_conf, &out_conf);

  switch (clk_chan) {
  case SI5351_TX: {
    si5351_SetupPLL(SI5351_PLL_B, &pll_conf);
    si5351_SetupOutput(1, SI5351_PLL_B, SI5351_DRIVE_STRENGTH_4MA, &out_conf,
                       0);
    break;
  }
  case SI5351_LO: {
    si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
    si5351_SetupOutput(0, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &out_conf,
                       0);
    break;
  }
  case SI5351_CLK2: {
    uartprint(&hlpuart1, "clk2 not used.\n");
    break;
  }
  }

  si5351_EnableOutputs((1 << 0) | (1 << 1));
}

// 数据预处理（去直流分量）
void remove_dc(float *data, int n) {
  float sum = 0;
  for (int i = 0; i < n; i++)
    sum += data[i];
  float mean = sum / n;
  for (int i = 0; i < n; i++)
    data[i] -= mean;
}

void ad7606_Sample(AD7606_Handler *ad7606, int16_t *bufferi, float *bufferf,
                   uint16_t sample_length) {

  for (uint16_t i = 0; i < sample_length; i++) {
    AD7606_StartReadBytes(ad7606, bufferi + i * 4, 4);
  }
  // AD7606_StartReadBytes(ad7606, bufferi, 4 * sample_length);
  AD7606_ConvertToVoltage(ad7606, bufferi, bufferf, sample_length * 4);
}
