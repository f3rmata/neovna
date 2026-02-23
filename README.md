# neovna

A simple STM32-based Vector Network Analyzer (VNA) for the 2025 NUEDC D task.

- MCU: STM32H7
- RF Clock: Si5351
- ADC: AD7606B (16‑bit)
- DSP: CMSIS-DSP

本项目现已加入 SOLT 标定（短路/开路/负载/直通）支持，用于对测得的 S11/S21 进行误差修正。

## Features

- Fast frequency sweep with Si5351
- 16-bit high precision data acquisition with AD7606B
- IQ demodulation at fixed IF, robust amplitude/phase extraction
- Frequency-domain reflectometry (FDR) to estimate cable length
- NEW: SOLT calibration (1-port S11 3-term error model + S21 THRU amplitude)

## Hardware wiring

- I2C1 → Si5351 (400 kHz, high-speed mode recommended)
- SPI1 → AD7606B (tested 12.5 MHz, keep below 20 MHz)
- LPUART1 / USART2 → Debug UART
- AD7606B channel mapping (expected):
  - CH1: Reflected (reverse coupler)
  - CH2: Incident (forward coupler)
  - CH3: Transmission (S21)
  - CH4: Unused

Clocking
- TX: Si5351 one channel
- LO: Si5351 another channel
- IF is fixed at about 5 kHz in firmware

## Build dependencies

- STM32 HAL
- CMSIS-DSP
- CMake (optional; you can also integrate into a STM32CubeIDE project)

## Quick start

- Wire Si5351, AD7606B, and UART per the diagram above.
- Flash the firmware.
- The example below performs an S11/S21 sweep and prints cable length.

```c
static double velocity_factor = 0.66;
static double light_speed = 299792458.0;

AD7606_Handler ad7606;
ad7606_dma_status = AD7606_DMA_FREE;
AD7606_Init(&ad7606, &hspi1, AD7606_CONVST_GPIO_Port, AD7606_CONVST_Pin,
            AD7606_CONVST_GPIO_Port, AD7606_CONVST_Pin, AD7606_BUSY_GPIO_Port,
            AD7606_BUSY_Pin, AD7606_RST_GPIO_Port, AD7606_RST_Pin, Range_5VPP,
            2.5);

double vf_mps = velocity_factor * light_speed;
double cable_len = S11_S21_sweep(&ad7606, 4997, 1000, 120e6, vf_mps, 0);
uartprint(&hlpuart1, "cable length: %.5f m\n", cable_len);
```

## SOLT calibration

We provide a light-weight SOLT module (Inc/solt.h, Src/solt.c).

- S11 uses the classic 3-term error model (directivity Ed, source match Es, tracking Et) solved point-by-point from OPEN/SHORT/LOAD.
- S21 currently supports THRU-based amplitude tracking (divide by measured THRU amplitude). Phase calibration for S21 can be added later.

Typical procedure
1) Prepare standards
- OPEN: Open-circuit at port 1
- SHORT: Short at port 1
- LOAD: 50 Ω load at port 1
- THRU (optional for S21 amplitude): Direct connection between port 1 and port 2

2) Capture each standard
- For each standard: connect → call S11_S21_sweep → feed arrays into the calibration container.

3) Solve and apply
- Solve for S11 error terms.
- Measure DUT → apply S11 correction (and S21 amplitude scaling if THRU is provided).

Code example
```c
#include "solt.h"

// Globals filled by sweep() in neovna.c
extern float S11_amp[];
extern float S11_phase[];
extern float S21_amp[];

#define N_SWEEP 251

CalSOLT cal;
cal_solt_init(&cal, 50.0f);

// 1) OPEN
// Connect OPEN → run sweep
S11_S21_sweep(&ad7606, 4997, 1000, 120e6, vf_mps, 0);
cal_solt_set_open(&cal, S11_amp, S11_phase, N_SWEEP);

// 2) SHORT
S11_S21_sweep(&ad7606, 4997, 1000, 120e6, vf_mps, 0);
cal_solt_set_short(&cal, S11_amp, S11_phase, N_SWEEP);

// 3) LOAD (50 Ω)
S11_S21_sweep(&ad7606, 4997, 1000, 120e6, vf_mps, 0);
cal_solt_set_load(&cal, S11_amp, S11_phase, N_SWEEP);

// Optional: THRU for S21 amplitude
// Connect THRU → run sweep
S11_S21_sweep(&ad7606, 4997, 1000, 120e6, vf_mps, 0);
cal_solt_set_thru_amp(&cal, S21_amp, N_SWEEP);

// 4) Solve
cal_solt_solve(&cal);

// 5) Measure DUT → apply correction
S11_S21_sweep(&ad7606, 4997, 1000, 120e6, vf_mps, 0);

float S11_amp_corr[N_SWEEP];
float S11_phase_corr[N_SWEEP];
float S21_amp_corr[N_SWEEP];

cal_solt_apply_s11(&cal, S11_amp, S11_phase, S11_amp_corr, S11_phase_corr, N_SWEEP);
cal_solt_apply_s21_amp(&cal, S21_amp, S21_amp_corr, N_SWEEP);

// Example: use corrected S11 for cable length estimation
// (recomputing the frequency axis as needed)
```

Notes
- The OPEN/SHORT/LOAD ideal coefficients use simplified Γ values (+1/-1/0). You can extend the solver to include frequency-dependent offsets (open capacitance, short inductance) if needed.
- S21 calibration here is amplitude-only by THRU normalization.

## Algorithm overview

- Frequency plan
  - TX and LO from Si5351, fixed IF ≈ 5 kHz
- Sampling
  - AD7606B reads Incident/Reflected/Transmission channels per point
  - DC removal, complex down-conversion via recursive oscillator, I/Q accumulation
- S11/S21 extraction
  - S11 amplitude = |Ref| / |Inc|
  - S11 phase = ∠Ref − ∠Inc (unwrapped to −π..π per step)
  - S21 amplitude = |Tran| / |Inc|
  - S21 phase is now also computed (relative to ∠Inc) for future extension
- Cable length (FDR/TDR-like)
  - Create complex S11(ω) = A·e^{jφ}, mirror to full spectrum, IFFT → time domain
  - Pick first significant peak after a minimal length threshold, map delay to length

## Known limitations

- S21 calibration is amplitude-only; precise 2‑port (12‑term) correction is not included.
- The REF_COMP constant compensates the coupler ratio roughly; with SOLT in place, you can reduce its impact or set it to 1 and rely on calibration.
- Sweep timing and IF must keep the IF tone coherent across acquisitions; avoid too small sample counts.

## Roadmap

- Add frequency-dependent OPEN/SHORT models (C_open, L_short)
- Optional full 2‑port calibration (12‑term)
- Improve AD7606B samplerate selection and synchronization