#include "neovna.h"

#define SWEEP_POINT 251
#define SWEEP_POINT_SHIELD 11

// #define SAMPLE_POINT 100
#define AD7606_FS 114940.0f // 实测或标称采样率 (Hz)
#define IF_FREQ_HZ 5000.0f  // 固定中频/拍频 (Hz)
// 周期数 (可调: 4~8). N = (Fs/IF)*CYCLES; Fs/IF 应≈20, 保证整数。
#define IF_CYCLES 4
#define SAMPLE_POINT 69

#define POINT_FFT_LEN 64
#define CABLE_IFFT_LEN 4096
#define SHIELD_IFFT_LEN 1024
#define MIN_CABLE_LENGTH_M 0.6
#define MIN_SHIELD_LENGTH_M 0.95

#define REF_COMP 8.4034

float S11_amp[SWEEP_POINT];   // S11 幅度
float S11_phase[SWEEP_POINT]; // S11 相位
float S21_amp[SWEEP_POINT];   // S11 幅度

// float S21_amp[SWEEP_POINT];   // S21 幅度
// float S21_phase[SWEEP_POINT]; // S21 相位

// 生成线性扫描频点
void generate_sweep_freq(float start_freq, float end_freq, uint16_t points,
                         float *sweep_fre) {
  if (points < 2)
    return;
  float step = (end_freq - start_freq) / (points - 1);
  for (uint16_t i = 0; i < points; i++) {
    sweep_fre[i] = start_freq + i * step;
  }
}

float unwrap_phase(float phase0, float phase1) {
  float delta = phase1 - phase0;
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  } else if (delta < -M_PI) {
    delta += 2 * M_PI;
  }
  return delta;
}

void calc_s11(float *incidentf, float *reflectf, float *s11_amp,
              float *s11_phase) {

  float fft_inc[POINT_FFT_LEN] = {0};
  float fft_ref[POINT_FFT_LEN] = {0};
  float fft_mag_inc[POINT_FFT_LEN] = {0};
  float fft_mag_ref[POINT_FFT_LEN] = {0};

  // float mag_inc, mag_ref, phase_inc, phase_ref;

  // 初始化FFT
  arm_rfft_fast_instance_f32 fft_instance;
  arm_rfft_fast_init_f32(&fft_instance, POINT_FFT_LEN);

  // 对入射信号做FFT
  arm_rfft_fast_f32(&fft_instance, incidentf, fft_inc, 0);
  arm_cmplx_mag_f32(fft_inc, fft_mag_inc, POINT_FFT_LEN / 2);
  // for (uint8_t k = 0; k < POINT_FFT_LEN; k++) {
  //   uartprint(&hlpuart1, "fft_inc, fft_mag_inc [%d]: %f, %f\n", k,
  //   fft_inc[k],
  //             fft_mag_inc[k]);
  // }
  float phase0 = atan2f(fft_inc[9], fft_inc[8]);

  // 对反射信号做FFT
  arm_rfft_fast_f32(&fft_instance, reflectf, fft_ref, 0);
  arm_cmplx_mag_f32(fft_ref, fft_mag_ref, POINT_FFT_LEN / 2);
  // for (uint8_t k = 0; k < POINT_FFT_LEN; k++) {
  //   uartprint(&hlpuart1, "fft_ref, fft_mag_ref [%d]: %f, %f\n", k,
  //   fft_ref[k],
  //             fft_mag_ref[k]);
  // }
  float phase1 = atan2f(fft_ref[9], fft_ref[8]);

  *s11_amp = fft_mag_inc[3] / fft_mag_ref[3]; // theortically be 3
  *s11_phase = unwrap_phase(phase0, phase1);
}

void calc_s11_iq(float *incidentf, float *reflectf, float *transmissionf,
                 uint16_t sample_length, float *s11_amp, float *s11_phase,
                 float *s21_amp) {

  remove_dc(incidentf, sample_length);
  remove_dc(reflectf, sample_length);
  remove_dc(transmissionf, sample_length);

  // for (uint16_t i = 0; i < sample_length; i++) {
  //   uartprint(&hlpuart1, "incident, reflect: 1, %.4f, %.4f\n", incidentf[i],
  //             reflectf[i]);
  // }

  const float w = 2.0f * PI * IF_FREQ_HZ / AD7606_FS;
  float cw = arm_cos_f32(w);
  float sw = arm_sin_f32(w);
  // 递推旋转: (c,s) 初始=1,0
  float c = 1.0f, s = 0.0f;
  float Re1 = 0, Im1 = 0, Re2 = 0, Im2 = 0, Re3 = 0, Im3 = 0;
  for (uint16_t n = 0; n < sample_length; n++) {
    float x1 = incidentf[n], x2 = reflectf[n], x3 = transmissionf[n];
    Re1 += x1 * c;
    Im1 -= x1 * s;
    Re2 += x2 * c;
    Im2 -= x2 * s;
    Re3 += x3 * c;
    Im3 -= x3 * s;
    // 旋转到下一点
    float c_next = c * cw - s * sw;
    s = c * sw + s * cw;
    c = c_next;
  }

  double phase1 = atan2(Im1, Re1);
  double phase2 = atan2(Im2, Re2);

  double delta_phase = unwrap_phase(phase1, phase2);

  double amp1 = sqrt(Re1 * Re1 + Im1 * Im1);
  double amp2 = sqrt(Re2 * Re2 + Im2 * Im2);
  double amp3 = sqrt(Re3 * Re3 + Im3 * Im3);

  *s11_amp = amp2 / amp1; // 线性幅度
  *s11_phase = delta_phase;
  *s21_amp = amp3 / amp1;
}

double estimate_cable_length(float *sweep_freq, float *amp, float *phase,
                             uint16_t N, double velocity) {

  if (N < 2 || N >= CABLE_IFFT_LEN / 2)
    return 0.0f;

  float freq_step = sweep_freq[1] - sweep_freq[0];

  static float spec[2 * CABLE_IFFT_LEN];
  static float time_seq[2 * CABLE_IFFT_LEN];
  static float mag[CABLE_IFFT_LEN];

  memset(spec, 0, sizeof(spec));

  // -------- Hann 窗 (arm_hanning_f32) --------
  float win[N];
  uint16_t win_len_cached = 0;
  if (win_len_cached != N) {
    arm_hanning_f32(win, N); // 生成长度 N 的窗口系数
    win_len_cached = N;

    // // 能量归一化 (可选，使 ∑w^2 = N)
    float e = 0.f;
    for (uint16_t k = 0; k < N; k++)
      e += win[k] * win[k];
    float scale = 0;
    arm_sqrt_f32((float)N / e, &scale);
    for (uint16_t k = 0; k < N; k++)
      win[k] *= scale;
  }

  // -------- 填充正频率并加窗 --------
  for (uint16_t k = 0; k < N; k++) {
    // float w = win[k];
    float w = 1;
    float re = w * amp[k] * arm_cos_f32(phase[k]);
    float im = w * amp[k] * arm_sin_f32(phase[k]);
    spec[2 * k] = re;
    spec[2 * k + 1] = im;
  }

  // 共轭对称
  for (uint16_t k = 1; k < N; k++) {
    uint16_t mirror = CABLE_IFFT_LEN - k;
    spec[2 * mirror] = spec[2 * k];
    spec[2 * mirror + 1] = -spec[2 * k + 1];
  }

  for (uint32_t i = 0; i < 2 * CABLE_IFFT_LEN; i++)
    time_seq[i] = spec[i];

  arm_cfft_instance_f32 S;
  arm_cfft_init_f32(&S, CABLE_IFFT_LEN);
  arm_cfft_f32(&S, time_seq, 1, 1); // IFFT
  // float invN = 1.0f / (float)CABLE_IFFT_LEN;
  // for (uint32_t i = 0; i < CABLE_IFFT_LEN; i++) {
  //   time_seq[2 * i] *= invN;
  //   time_seq[2 * i + 1] *= invN;
  //   uartprint(&hlpuart1, "real, imag: %f, %f\n", time_seq[2 * i],
  //             time_seq[2 * i + 1]);
  // }

  arm_cmplx_mag_f32(time_seq, mag, CABLE_IFFT_LEN);

  // for (uint32_t i = 0; i < CABLE_IFFT_LEN / 2 - 1; i++) {
  //   uartprint(&hlpuart1, "real, imag, mag: %f, %f, %f\n", time_seq[2 * i],
  //             time_seq[2 * i + 1], mag[i]);
  // }

  uint32_t peak_index = 0;
  float peak_value = 0.f;
  for (uint32_t i = 2; i < CABLE_IFFT_LEN / 2 - 1; i++) {
    float delay = (float)i / (float)CABLE_IFFT_LEN / freq_step;
    float length = delay * velocity * 0.5f;
    if (length < MIN_CABLE_LENGTH_M)
      continue;
    if (mag[i] < mag[i - 1] || mag[i] < mag[i + 1])
      continue;
    if (mag[i] > peak_value) {
      peak_value = mag[i];
      peak_index = i;
    }
  }
  if (peak_index == 0)
    return 0.0f;

  double delay = (float)peak_index / (float)CABLE_IFFT_LEN / freq_step;
  return delay * velocity * 0.5f;
}

// S11,S21扫频测试
float S11_S21_sweep(AD7606_Handler *ad7606, int32_t lo_hz, int32_t start_hz,
                    int32_t end_hz, double velocity_factor, uint8_t axis) {

  float sweep_fre[SWEEP_POINT]; // 扫频频点
  generate_sweep_freq((float)start_hz, (float)end_hz, SWEEP_POINT, sweep_fre);

  // 设置第一次扫频点
  setSi5351Freq(SI5351_TX, (int32_t)sweep_fre[0]);
  HAL_Delay(10);
  setSi5351Freq(SI5351_LO, (int32_t)sweep_fre[0] - lo_hz);
  HAL_Delay(200);

  for (uint16_t i = 1; i < SWEEP_POINT; i++) {

    int16_t bufferi[4 * (SAMPLE_POINT + 10)] = {0};
    float bufferf[4 * (SAMPLE_POINT + 10)] = {0};

    float reflectf[SAMPLE_POINT] = {0};
    float incidentf[SAMPLE_POINT] = {0};
    float transmissionf[SAMPLE_POINT] = {0};

    // while (1) {
    ad7606_Sample(ad7606, bufferi, bufferf, SAMPLE_POINT + 10);

    // 设置下一次扫频点
    setSi5351Freq(SI5351_TX, (int32_t)sweep_fre[i]);
    // HAL_Delay(2);
    setSi5351Freq(SI5351_LO, (int32_t)sweep_fre[i] - lo_hz);
    HAL_Delay(1);

    const uint16_t start_offset = 5; // 跳过前几个点
    for (uint16_t j = 0; j < SAMPLE_POINT; j++) {
      // 计算在 bufferf 中的源索引
      uint16_t source_index = start_offset + j;
      // 从 bufferf 对应的通道复制数据
      transmissionf[j] = bufferf[source_index * 4 + 2];   // 传输信号在通道3
      incidentf[j] = bufferf[source_index * 4 + 1];       // 入射信号在通道2
      reflectf[j] = REF_COMP * bufferf[source_index * 4]; // 反射信号在通道1
      // uartprint(&hlpuart1,
      //           "sweep_freq_khz, incident, reflect: %.1f, %.4f, %.4f\n",
      //           sweep_fre[i] / 1000, incidentf[j], reflectf[j]);
    }

    // calc_s11(incidentf, reflectf, S11_amp + i - 1, S11_phase + i - 1);

    calc_s11_iq(incidentf, reflectf, transmissionf, SAMPLE_POINT,
                S11_amp + i - 1, S11_phase + i - 1, S21_amp + i - 1);

    uartprint(&huart2,
              "sweep_freq_khz, s11_amp, s11_phase, s21_amp: %f, %f, %f, %f\n",
              sweep_fre[i] / 1000, S11_amp[i - 1], S11_phase[i - 1],
              S21_amp[i - 1]);

    // HAL_Delay(20);
  }

  // 计算线缆长度（以典型同轴线缆速度2e8 m/s为例）
  // float velocity = 0.6918 * 299792458.0f;
  float cable_length = estimate_cable_length(sweep_fre, S11_amp, S11_phase,
                                             SWEEP_POINT, velocity_factor);

  return cable_length;
}

float S11_S21_sweep_shield(AD7606_Handler *ad7606, int32_t lo_hz,
                           int32_t start_hz, int32_t end_hz,
                           double velocity_factor, uint8_t axis) {

  float sweep_fre[SWEEP_POINT_SHIELD]; // 扫频频点
  generate_sweep_freq((float)start_hz, (float)end_hz, SWEEP_POINT_SHIELD,
                      sweep_fre);

  // 设置第一次扫频点
  setSi5351Freq(SI5351_TX, (int32_t)sweep_fre[0]);
  HAL_Delay(10);
  setSi5351Freq(SI5351_LO, (int32_t)sweep_fre[0] - lo_hz);
  HAL_Delay(100);

  for (uint16_t i = 1; i < SWEEP_POINT_SHIELD; i++) {

    int16_t bufferi[4 * (SAMPLE_POINT + 10)] = {0};
    float bufferf[4 * (SAMPLE_POINT + 10)] = {0};

    float reflectf[SAMPLE_POINT] = {0};
    float incidentf[SAMPLE_POINT] = {0};
    float transmissionf[SAMPLE_POINT] = {0};

    // while (1) {
    ad7606_Sample(ad7606, bufferi, bufferf, SAMPLE_POINT + 10);

    // 设置下一次扫频点
    setSi5351Freq(SI5351_TX, (int32_t)sweep_fre[i]);
    HAL_Delay(2);
    setSi5351Freq(SI5351_LO, (int32_t)sweep_fre[i] - lo_hz);
    // HAL_Delay(10);

    const uint16_t start_offset = 5; // 跳过前几个点
    for (uint16_t j = 0; j < SAMPLE_POINT; j++) {
      // 计算在 bufferf 中的源索引
      uint16_t source_index = start_offset + j;
      // 从 bufferf 对应的通道复制数据
      transmissionf[j] = bufferf[source_index * 4 + 2];   // 传输信号在通道3
      incidentf[j] = bufferf[source_index * 4 + 1];       // 入射信号在通道2
      reflectf[j] = REF_COMP * bufferf[source_index * 4]; // 反射信号在通道1
      // uartprint(&hlpuart1,
      //           "sweep_freq_khz, incident, reflect: %.1f, %.4f, %.4f\n",
      //           sweep_fre[i] / 1000, incidentf[j], reflectf[j]);
    }

    // calc_s11(incidentf, reflectf, S11_amp + i - 1, S11_phase + i - 1);

    calc_s11_iq(incidentf, reflectf, transmissionf, SAMPLE_POINT,
                S11_amp + i - 1, S11_phase + i - 1, S21_amp + i - 1);

    uartprint(&hlpuart1,
              "sweep_freq_khz, s11_amp, s11_phase, s21_amp: %f, %f, %f,%f\n",
              sweep_fre[i] / 1000, S11_amp[i - 1], S11_phase[i - 1],
              S21_amp[i - 1]);

    // HAL_Delay(20);
  }

  // 计算线缆长度（以典型同轴线缆速度2e8 m/s为例）
  // float velocity = 0.6918 * 299792458.0f;
  // float cable_length = estimate_cable_length(
  //     sweep_fre, S11_amp, S11_phase, SWEEP_POINT_SHIELD, velocity_factor);

  float s11_phase_avg = 0;
  for (uint16_t i = 0; i < SWEEP_POINT_SHIELD; i++) {
    s11_phase_avg += S11_phase[i];
  }
  s11_phase_avg = s11_phase_avg / SWEEP_POINT_SHIELD;

  // float s11_phase_std = 0;
  // arm_var_f32(S11_phase, SWEEP_POINT_SHIELD, &s11_phase_std);

  uartprint(&hlpuart1, "s11_phase_avg: %.5f m\n", s11_phase_avg);
  // uartprint(&huart2, "cable length: %.5f m\n", cable_length);
  return s11_phase_avg;
}
