#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CAL_POINTS 512

typedef struct {
  float re;
  float im;
} complex_f32;

typedef struct {
  // number of points for calibration arrays
  uint16_t N;
  // flags
  uint8_t have_open;
  uint8_t have_short_;
  uint8_t have_load;
  uint8_t have_thru;

  // reference impedance
  float Z0;

  // measured S11 of standards (complex Gamma_m)
  complex_f32 M_open[MAX_CAL_POINTS];
  complex_f32 M_short[MAX_CAL_POINTS];
  complex_f32 M_load[MAX_CAL_POINTS];

  // transmission THRU, amplitude only for now
  float M_thru_amp[MAX_CAL_POINTS];

  // solved 1-port error terms per frequency point
  complex_f32 Ed[MAX_CAL_POINTS]; // directivity
  complex_f32 Et[MAX_CAL_POINTS]; // tracking
  complex_f32 Es[MAX_CAL_POINTS]; // source match
} CalSOLT;

/**
 * Initialize a SOLT calibration container.
 * Z0: reference impedance (typically 50.0f).
 */
void cal_solt_init(CalSOLT *cal, float Z0);

/**
 * Set measured data for OPEN standard.
 * s11_amp: linear amplitude
 * s11_phase: radians
 * N: number of points (<= MAX_CAL_POINTS)
 */
void cal_solt_set_open(CalSOLT *cal, const float *s11_amp, const float *s11_phase, uint16_t N);

/**
 * Set measured data for SHORT standard.
 */
void cal_solt_set_short(CalSOLT *cal, const float *s11_amp, const float *s11_phase, uint16_t N);

/**
 * Set measured data for LOAD standard.
 */
void cal_solt_set_load(CalSOLT *cal, const float *s11_amp, const float *s11_phase, uint16_t N);

/**
 * Set measured data for THRU (S21) standard.
 * amp only for now.
 */
void cal_solt_set_thru_amp(CalSOLT *cal, const float *s21_amp, uint16_t N);

/**
 * Solve 1-port (S11) 3-term error model using the OPEN/SHORT/LOAD sets.
 * Returns 0 on success, non-zero on error.
 */
int cal_solt_solve(CalSOLT *cal);

/**
 * Apply 1-port S11 correction to a measured sweep.
 * in_amp/in_phase: measured arrays
 * out_amp/out_phase: corrected arrays
 * N: must equal cal->N
 */
void cal_solt_apply_s11(const CalSOLT *cal,
                        const float *in_amp, const float *in_phase,
                        float *out_amp, float *out_phase,
                        uint16_t N);

/**
 * Apply THRU-based amplitude scaling to S21.
 * out_amp = in_amp / THRU_amp
 */
void cal_solt_apply_s21_amp(const CalSOLT *cal,
                            const float *in_amp,
                            float *out_amp,
                            uint16_t N);

#ifdef __cplusplus
}
#endif