#include "solt.h"

#include <math.h>
#include <string.h>

static inline complex_f32 c_add(complex_f32 a, complex_f32 b) {
  complex_f32 r = {a.re + b.re, a.im + b.im};
  return r;
}

static inline complex_f32 c_sub(complex_f32 a, complex_f32 b) {
  complex_f32 r = {a.re - b.re, a.im - b.im};
  return r;
}

static inline complex_f32 c_mul(complex_f32 a, complex_f32 b) {
  complex_f32 r = {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
  return r;
}

static inline complex_f32 c_div(complex_f32 a, complex_f32 b) {
  float denom = b.re * b.re + b.im * b.im;
  complex_f32 r = {0, 0};
  if (denom != 0.0f) {
    r.re = (a.re * b.re + a.im * b.im) / denom;
    r.im = (a.im * b.re - a.re * b.im) / denom;
  }
  return r;
}

static inline float c_abs(complex_f32 a) { return sqrtf(a.re * a.re + a.im * a.im); }

static inline complex_f32 c_from_amp_phase(float amp, float phase) {
  complex_f32 r;
  r.re = amp * cosf(phase);
  r.im = amp * sinf(phase);
  return r;
}

static inline void c_to_amp_phase(complex_f32 v, float *amp, float *phase) {
  if (amp)
    *amp = sqrtf(v.re * v.re + v.im * v.im);
  if (phase)
    *phase = atan2f(v.im, v.re);
}

void cal_solt_init(CalSOLT *cal, float Z0) {
  if (!cal)
    return;
  memset(cal, 0, sizeof(*cal));
  cal->Z0 = Z0;
}

// Store measured data
void cal_solt_set_open(CalSOLT *cal, const float *s11_amp, const float *s11_phase, uint16_t N) {
  if (!cal || !s11_amp || !s11_phase || N == 0 || N > MAX_CAL_POINTS)
    return;
  cal->N = N;
  for (uint16_t i = 0; i < N; i++) {
    cal->M_open[i] = c_from_amp_phase(s11_amp[i], s11_phase[i]);
  }
  cal->have_open = 1;
}

void cal_solt_set_short(CalSOLT *cal, const float *s11_amp, const float *s11_phase, uint16_t N) {
  if (!cal || !s11_amp || !s11_phase || N == 0 || N > MAX_CAL_POINTS)
    return;
  cal->N = N;
  for (uint16_t i = 0; i < N; i++) {
    cal->M_short[i] = c_from_amp_phase(s11_amp[i], s11_phase[i]);
  }
  cal->have_short_ = 1;
}

void cal_solt_set_load(CalSOLT *cal, const float *s11_amp, const float *s11_phase, uint16_t N) {
  if (!cal || !s11_amp || !s11_phase || N == 0 || N > MAX_CAL_POINTS)
    return;
  cal->N = N;
  for (uint16_t i = 0; i < N; i++) {
    cal->M_load[i] = c_from_amp_phase(s11_amp[i], s11_phase[i]);
  }
  cal->have_load = 1;
}

void cal_solt_set_thru_amp(CalSOLT *cal, const float *s21_amp, uint16_t N) {
  if (!cal || !s21_amp || N == 0 || N > MAX_CAL_POINTS)
    return;
  cal->N = N;
  for (uint16_t i = 0; i < N; i++) {
    cal->M_thru_amp[i] = s21_amp[i];
  }
  cal->have_thru = 1;
}

// Solve 3x3 complex linear system A x = b using Gaussian elimination with partial pivoting
static int solve_3x3(complex_f32 A[3][3], complex_f32 b[3], complex_f32 x[3]) {
  // Forward elimination
  for (int col = 0; col < 2; col++) {
    // Pivot
    int pivot = col;
    float maxabs = c_abs(A[col][col]);
    for (int r = col + 1; r < 3; r++) {
      float v = c_abs(A[r][col]);
      if (v > maxabs) {
        maxabs = v;
        pivot = r;
      }
    }
    if (maxabs == 0.0f)
      return -1; // singular

    if (pivot != col) {
      for (int k = col; k < 3; k++) {
        complex_f32 tmp = A[col][k];
        A[col][k] = A[pivot][k];
        A[pivot][k] = tmp;
      }
      complex_f32 tb = b[col];
      b[col] = b[pivot];
      b[pivot] = tb;
    }

    // Eliminate below
    for (int r = col + 1; r < 3; r++) {
      complex_f32 factor = c_div(A[r][col], A[col][col]);
      // Row r = Row r - factor * Row col
      for (int k = col; k < 3; k++) {
        A[r][k] = c_sub(A[r][k], c_mul(factor, A[col][k]));
      }
      b[r] = c_sub(b[r], c_mul(factor, b[col]));
    }
  }

  // Back substitution
  for (int i = 2; i >= 0; i--) {
    complex_f32 sum = {0, 0};
    for (int j = i + 1; j < 3; j++) {
      sum = c_add(sum, c_mul(A[i][j], x[j]));
    }
    complex_f32 rhs = c_sub(b[i], sum);
    if (c_abs(A[i][i]) == 0.0f)
      return -2;
    x[i] = c_div(rhs, A[i][i]);
  }

  return 0;
}

int cal_solt_solve(CalSOLT *cal) {
  if (!cal || !cal->have_open || !cal->have_short_ || !cal->have_load)
    return -1;

  const uint16_t N = cal->N;

  // Ideal standard reflection coefficients (simplified):
  // OPEN: +1
  // SHORT: -1
  // LOAD: 0
  complex_f32 A_open = {+1.0f, 0.0f};
  complex_f32 A_short = {-1.0f, 0.0f};
  complex_f32 A_load = {0.0f, 0.0f};

  for (uint16_t i = 0; i < N; i++) {
    complex_f32 M_open = cal->M_open[i];
    complex_f32 M_short = cal->M_short[i];
    complex_f32 M_load = cal->M_load[i];

    complex_f32 A[3][3];
    complex_f32 b[3];

    // Equation: Mi*(1 + Es*Ai) = Ed + Et*Ai
    // => -Ed + (Ai)*Et + (-Mi*Ai)*Es = -Mi
    // Unknowns: [Ed, Et, Es]
    A[0][0] = (complex_f32){-1.0f, 0.0f};
    A[0][1] = A_open;
    A[0][2] = c_mul((complex_f32){-1.0f, 0.0f}, c_mul(M_open, A_open));
    b[0] = c_mul((complex_f32){-1.0f, 0.0f}, M_open);

    A[1][0] = (complex_f32){-1.0f, 0.0f};
    A[1][1] = A_short;
    A[1][2] = c_mul((complex_f32){-1.0f, 0.0f}, c_mul(M_short, A_short));
    b[1] = c_mul((complex_f32){-1.0f, 0.0f}, M_short);

    A[2][0] = (complex_f32){-1.0f, 0.0f};
    A[2][1] = A_load;
    A[2][2] = c_mul((complex_f32){-1.0f, 0.0f}, c_mul(M_load, A_load));
    b[2] = c_mul((complex_f32){-1.0f, 0.0f}, M_load);

    complex_f32 x[3];
    if (solve_3x3(A, b, x) != 0) {
      // fallback to zeros if singular
      cal->Ed[i] = (complex_f32){0, 0};
      cal->Et[i] = (complex_f32){1, 0};
      cal->Es[i] = (complex_f32){0, 0};
      continue;
    }

    cal->Ed[i] = x[0];
    cal->Et[i] = x[1];
    cal->Es[i] = x[2];
  }

  return 0;
}

void cal_solt_apply_s11(const CalSOLT *cal,
                        const float *in_amp, const float *in_phase,
                        float *out_amp, float *out_phase,
                        uint16_t N) {
  if (!cal || !in_amp || !in_phase || !out_amp || !out_phase)
    return;
  if (N != cal->N)
    return;

  for (uint16_t i = 0; i < N; i++) {
    complex_f32 m = c_from_amp_phase(in_amp[i], in_phase[i]);
    // Γ = (m - Ed) / (Et - m*Es)
    complex_f32 num = c_sub(m, cal->Ed[i]);
    complex_f32 den = c_sub(cal->Et[i], c_mul(m, cal->Es[i]));
    complex_f32 gamma = c_div(num, den);
    c_to_amp_phase(gamma, &out_amp[i], &out_phase[i]);
  }
}

void cal_solt_apply_s21_amp(const CalSOLT *cal,
                            const float *in_amp,
                            float *out_amp,
                            uint16_t N) {
  if (!cal || !in_amp || !out_amp)
    return;
  if (N != cal->N)
    return;

  for (uint16_t i = 0; i < N; i++) {
    float t = cal->M_thru_amp[i];
    if (t != 0.0f) {
      out_amp[i] = in_amp[i] / t;
    } else {
      out_amp[i] = in_amp[i];
    }
  }
}