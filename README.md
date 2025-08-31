# neovna

Simple vna implementation for 2025 nuedc D task.

Works with stm32h7 / ad7606b / si5351

## Features

- Fast sweep speed with si5351
- High precision data acuqusition with ad7606b (16bit 800kpbs adc)
- Simple fdr algorithm to estimate cable length

## Dependency

```
CMake
STM32HAL
CMSIS-DSP
```

## Usage

- I2C1 -> SI5351   400kbps highspeed mode (to improve frequency sweep speed)
- SPI1 -> AD7606B  12.5MHz clk, or other clk under 20Mhz
- LPUART1 -> Debug uart port

```c
static double velocity_factor = 0.66f;
static double light_speed = 299792458.0f;

AD7606_Handler ad7606;
ad7606_dma_status = AD7606_DMA_FREE;
AD7606_Init(&ad7606, &hspi1, AD7606_CONVST_GPIO_Port, AD7606_CONVST_Pin,
            AD7606_CONVST_GPIO_Port, AD7606_CONVST_Pin, AD7606_BUSY_GPIO_Port,
            AD7606_BUSY_Pin, AD7606_RST_GPIO_Port, AD7606_RST_Pin, Range_5VPP,
            2.5);

velocity_factor = velocity_factor * light_speed;
cable_len =
    S11_S21_sweep(&ad7606, 4.997e3, 1e3, 120e6, velocity_factor_corr, 0);
uartprint(&hlpuart1, "cable length: %.5f m\n", cable_len);
```

## TODO

- SOLT Calibration
- Improve ad7606b data samplerate settings