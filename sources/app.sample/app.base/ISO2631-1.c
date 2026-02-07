// ISO2631-1.c
#include "ISO2631-1.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ISO_GRAVITY 9.81f

// ISO 2631-1 Wk 필터 주파수
#define WK_F1 0.4f
#define WK_F2 12.5f
#define WK_F3 12.5f
#define WK_F4 2.37f

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Biquad_t;

typedef struct {
    Biquad_t s1; // HPF 0.4
    Biquad_t s2; // LPF 12.5
    Biquad_t s3; // HPF 2.37
    Biquad_t s4; // LPF 12.5
} WkChain_t;

typedef struct {
    // config
    float fs;
    uint16_t warmup_samples;
    uint16_t gravity_samples;

    // measurement
    uint16_t window_samples;
    uint16_t n_total;      // 전체 들어온 샘플(워밍업 포함)
    uint16_t n_meas;       // 실제 측정(워밍업 이후)
    uint16_t idx;

    float gravity_accum;
    uint16_t gravity_cnt;
    float gravity_offset;

    // stats accum
    float sum_sq;
    float sum_4;
    float peak_abs;

    // outputs
    ISO2631_Metrics_t out;

    // flags
    uint8_t warmup_done;
    uint8_t active;

    WkChain_t wk;
} ISOState_t;

static ISOState_t g_iso;

// ---------- Biquad ----------
static void biquad_init(Biquad_t *bq, float fc, float fs, uint8_t type_hpf)
{
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;

    float omega = 2.0f * (float)M_PI * fc / fs;
    float K = tanf(omega * 0.5f);
    float K2 = K * K;
    float sqrt2 = 1.414213562f;

    float norm = 1.0f / (1.0f + sqrt2 * K + K2);

    if (!type_hpf) {
        // LPF butterworth
        bq->b0 = K2 * norm;
        bq->b1 = 2.0f * bq->b0;
        bq->b2 = bq->b0;
        bq->a1 = 2.0f * (K2 - 1.0f) * norm;
        bq->a2 = (1.0f - sqrt2 * K + K2) * norm;
    } else {
        // HPF butterworth
        bq->b0 = 1.0f * norm;
        bq->b1 = -2.0f * bq->b0;
        bq->b2 = bq->b0;
        bq->a1 = 2.0f * (K2 - 1.0f) * norm;
        bq->a2 = (1.0f - sqrt2 * K + K2) * norm;
    }
}

static float biquad_process(Biquad_t *bq, float x)
{
    float y = x * bq->b0 + bq->z1;
    bq->z1 = x * bq->b1 - y * bq->a1 + bq->z2;
    bq->z2 = x * bq->b2 - y * bq->a2;
    return y;
}

static void wk_init(WkChain_t *wk, float fs)
{
    biquad_init(&wk->s1, WK_F1, fs, 1); // HPF 0.4
    biquad_init(&wk->s2, WK_F2, fs, 0); // LPF 12.5
    biquad_init(&wk->s3, WK_F4, fs, 1); // HPF 2.37
    biquad_init(&wk->s4, WK_F3, fs, 0); // LPF 12.5
}

static float wk_process(WkChain_t *wk, float x_ms2)
{
    float y1 = biquad_process(&wk->s1, x_ms2);
    float y2 = biquad_process(&wk->s2, y1);
    float y3 = biquad_process(&wk->s3, y2);
    float y4 = biquad_process(&wk->s4, y3);
    return y4;
}

// ---------- Public API ----------
void ISO2631_Init(float fs_hz)
{
    memset(&g_iso, 0, sizeof(g_iso));
    g_iso.fs = (fs_hz > 1.0f) ? fs_hz : 100.0f;

    // 기본값: 워밍업 2초(200), 중력오프셋 1초(100) @100Hz
    g_iso.warmup_samples = (uint16_t)(2.0f * g_iso.fs);
    g_iso.gravity_samples = (uint16_t)(1.0f * g_iso.fs);

    wk_init(&g_iso.wk, g_iso.fs);
}

void ISO2631_SetWarmup(uint16_t warmup_samples, uint16_t gravity_samples)
{
    g_iso.warmup_samples = warmup_samples;
    g_iso.gravity_samples = gravity_samples;
}

void ISO2631_Start(uint16_t window_samples)
{
    g_iso.window_samples = window_samples;
    g_iso.n_total = 0;
    g_iso.n_meas = 0;
    g_iso.idx = 0;

    g_iso.gravity_accum = 0.0f;
    g_iso.gravity_cnt = 0;
    g_iso.gravity_offset = 0.0f;

    g_iso.sum_sq = 0.0f;
    g_iso.sum_4 = 0.0f;
    g_iso.peak_abs = 0.0f;

    g_iso.warmup_done = 0;
    g_iso.active = 1;

    wk_init(&g_iso.wk, g_iso.fs);

    memset(&g_iso.out, 0, sizeof(g_iso.out));
    g_iso.out.fs_hz = g_iso.fs;
}

static void compute_metrics(void)
{
    float dt = 1.0f / g_iso.fs;
    float T  = (float)g_iso.n_meas * dt;

    g_iso.out.duration_s = T;
    g_iso.out.samples = g_iso.n_meas;
    g_iso.out.fs_hz = g_iso.fs;
    g_iso.out.gravity_offset_ms2 = g_iso.gravity_offset;

    if (g_iso.n_meas == 0) {
        g_iso.out.rms = 0.0f;
        g_iso.out.vdv = 0.0f;
        g_iso.out.peak = 0.0f;
        return;
    }

    // ✅ RMS = sqrt(mean(a²))
    g_iso.out.rms = sqrtf(g_iso.sum_sq / (float)g_iso.n_meas);
    
    // ✅ VDV = (∫a⁴dt)^0.25 = (Σa⁴ · dt)^0.25
    g_iso.out.vdv = powf(g_iso.sum_4 * dt, 0.25f);
    
    g_iso.out.peak = g_iso.peak_abs;
}

uint8_t ISO2631_Update(float raw_z_ms2)
{
    if (!g_iso.active) return 0U;

    g_iso.n_total++;

    // ✅ 워밍업 중: 중력 오프셋 계산용으로 raw 값 누적
    if (!g_iso.warmup_done) {
        if (g_iso.gravity_cnt < g_iso.gravity_samples) {
            g_iso.gravity_accum += raw_z_ms2;
            g_iso.gravity_cnt++;
        }

        if (g_iso.n_total >= g_iso.warmup_samples) {
            if (g_iso.gravity_cnt > 0) {
                g_iso.gravity_offset = g_iso.gravity_accum / (float)g_iso.gravity_cnt;
            } else {
                g_iso.gravity_offset = ISO_GRAVITY;
            }
            g_iso.warmup_done = 1;
        }

        return 0U; // 워밍업 중에는 측정 안 쌓음
    }

    // ✅ 워밍업 완료 후: 중력 오프셋 제거
    float x = raw_z_ms2 - g_iso.gravity_offset;

    // ✅ Wk weighting 필터 적용
    float aw = wk_process(&g_iso.wk, x);

    // ✅ 통계 누적
    float abs_aw = fabsf(aw);
    if (abs_aw > g_iso.peak_abs) g_iso.peak_abs = abs_aw;

    g_iso.sum_sq += aw * aw;
    g_iso.sum_4  += aw * aw * aw * aw;  // ✅ a⁴ 직접 계산

    g_iso.n_meas++;
    g_iso.idx++;

    if (g_iso.n_meas >= g_iso.window_samples) {
        compute_metrics();
        g_iso.active = 0;
        return 1U; // 완료
    }

    return 0U;
}

ISO2631_Metrics_t ISO2631_GetMetrics(void)
{
    return g_iso.out;
}