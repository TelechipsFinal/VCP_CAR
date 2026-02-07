// SPDX-License-Identifier: Apache-2.0
/*
 * ISO2631-1.c
 * ISO 2631-1 (Whole-body vibration) helper for vertical (Z) acceleration.
 *
 * - Wk-like weighting implemented as a 4-section biquad chain:
 *   HPF 0.4 Hz -> LPF 12.5 Hz -> HPF 2.37 Hz -> LPF 12.5 Hz
 * - Computes RMS (a_w) and VDV over a measurement window.
 *
 * Notes for your Telechips SAL project:
 * - Call ISO2631_Init(fs_hz) once (recommended fs=100 for 10ms loop).
 * - Call ISO2631_Start(window_sec, now_ms) to begin a measurement (e.g., 7.0f).
 * - Call ISO2631_Update(az_ms2, now_ms) each sample tick.
 * - Call ISO2631_Stop() to finalize metrics (or let it auto-stop when window fills).
 */

#include "ISO2631-1.h"
#include <string.h>
#include <math.h>

/* ---------- utilities ---------- */
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ---------- biquad (Direct Form II Transposed) ---------- */
static void Biquad_Reset(Biquad_t *bq) {
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

static float Biquad_Process(Biquad_t *bq, float x) {
    float y = x * bq->b0 + bq->z1;
    bq->z1 = x * bq->b1 - y * bq->a1 + bq->z2;
    bq->z2 = x * bq->b2 - y * bq->a2;
    return y;
}

/*
 * 2nd-order Butterworth (RBJ cookbook form), Q = 1/sqrt(2)
 * fc: cutoff (Hz), fs: sample rate (Hz)
 * type_hpf: 0=LPF, 1=HPF
 */
static void Biquad_Init_Butterworth(Biquad_t *bq, float fc, float fs, uint8_t type_hpf)
{
    const float Q = 0.70710678f; /* 1/sqrt(2) */
    float w0 = 2.0f * (float)M_PI * (fc / fs);
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;

    if (!type_hpf) {
        /* LPF */
        b0 =  (1.0f - cosw0) * 0.5f;
        b1 =   1.0f - cosw0;
        b2 =  (1.0f - cosw0) * 0.5f;
    } else {
        /* HPF */
        b0 =  (1.0f + cosw0) * 0.5f;
        b1 = -(1.0f + cosw0);
        b2 =  (1.0f + cosw0) * 0.5f;
    }

    a0 = 1.0f + alpha;
    a1 = -2.0f * cosw0;
    a2 = 1.0f - alpha;

    /* normalize */
    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;

    Biquad_Reset(bq);
}

/* ---------- Wk chain ---------- */
static void Wk_Init(WkChain_t *wk, float fs_hz)
{
    /* practical approximation of ISO 2631-1 Wk for vertical vibration */
    Biquad_Init_Butterworth(&wk->sec1, 0.40f, fs_hz, 1U);  /* HPF */
    Biquad_Init_Butterworth(&wk->sec2, 12.5f, fs_hz, 0U);  /* LPF */
    Biquad_Init_Butterworth(&wk->sec3, 2.37f, fs_hz, 1U);  /* HPF */
    Biquad_Init_Butterworth(&wk->sec4, 12.5f, fs_hz, 0U);  /* LPF */
}

static float Wk_Process(WkChain_t *wk, float az_ms2)
{
    float y1 = Biquad_Process(&wk->sec1, az_ms2);
    float y2 = Biquad_Process(&wk->sec2, y1);
    float y3 = Biquad_Process(&wk->sec3, y2);
    float y4 = Biquad_Process(&wk->sec4, y3);
    return y4;
}

/* ---------- module state ---------- */
static ISO2631_State_t g_iso;

/* ---------- public API ---------- */
void ISO2631_Init(float fs_hz)
{
    memset(&g_iso, 0, sizeof(g_iso));
    g_iso.fs_hz = (fs_hz > 1.0f) ? fs_hz : 100.0f;

    g_iso.window_sec = 7.0f;
    g_iso.warmup_sec = 2.0f;
    g_iso.gravity_offset_ms2 = 9.81f;

    Wk_Init(&g_iso.wk, g_iso.fs_hz);
}

void ISO2631_SetWarmup(float warmup_sec)
{
    g_iso.warmup_sec = (warmup_sec < 0.0f) ? 0.0f : warmup_sec;
}

void ISO2631_Start(float window_sec, uint32_t now_ms)
{
    g_iso.active = 1U;

    g_iso.window_sec = (window_sec > 0.2f) ? window_sec : 7.0f;
    g_iso.window_samples = (uint32_t)(g_iso.window_sec * g_iso.fs_hz + 0.5f);
    if (g_iso.window_samples < 10U) g_iso.window_samples = 10U;
    if (g_iso.window_samples > ISO2631_MAX_SAMPLES) g_iso.window_samples = ISO2631_MAX_SAMPLES;

    g_iso.start_ms = now_ms;
    g_iso.sample_count = 0U;
    g_iso.buf_index = 0U;

    g_iso.rms = 0.0f;
    g_iso.vdv = 0.0f;
    g_iso.peak = 0.0f;

    /* warmup/gravity estimation */
    g_iso.warmup_done = 0U;
    g_iso.warmup_samples = 0U;
    g_iso.gravity_sum = 0.0f;
    g_iso.gravity_cnt = 0U;

    memset(g_iso.buf, 0, sizeof(g_iso.buf));
    Wk_Init(&g_iso.wk, g_iso.fs_hz);
}

static void ISO2631_RecalcMetrics(void)
{
    if (g_iso.sample_count == 0U) return;

    float sum2 = 0.0f;
    float sum4 = 0.0f;
    float peak = 0.0f;

    for (uint32_t i = 0; i < g_iso.sample_count; i++) {
        float a = g_iso.buf[i];
        float aa = fabsf(a);
        if (aa > peak) peak = aa;
        sum2 += a * a;
        sum4 += (a * a) * (a * a);
    }

    g_iso.rms = sqrtf(sum2 / (float)g_iso.sample_count);

    float dt = 1.0f / g_iso.fs_hz;
    g_iso.vdv = powf(sum4 * dt, 0.25f);
    g_iso.peak = peak;
}

void ISO2631_Stop(void)
{
    g_iso.active = 0U;
    ISO2631_RecalcMetrics();
}

uint8_t ISO2631_IsActive(void)
{
    return g_iso.active;
}

uint8_t ISO2631_WarmupDone(void)
{
    return g_iso.warmup_done;
}

ISO2631_Metrics_t ISO2631_GetMetrics(void)
{
    ISO2631_Metrics_t m;
    m.rms = g_iso.rms;
    m.vdv = g_iso.vdv;
    m.peak = g_iso.peak;
    m.duration_sec = (g_iso.sample_count > 0U) ? ((float)g_iso.sample_count / g_iso.fs_hz) : 0.0f;
    m.samples = g_iso.sample_count;
    m.gravity_offset_ms2 = g_iso.gravity_offset_ms2;
    return m;
}

/*
 * Update with raw vertical acceleration in m/s^2 (including gravity).
 * During warmup: estimate gravity offset (mean of raw az).
 * After warmup: (az - gravity_offset) -> Wk filter -> buffer.
 *
 * Return:
 *  0: not active
 *  1: updated
 *  2: window complete (auto-stopped)
 */
uint8_t ISO2631_Update(float az_raw_ms2, uint32_t now_ms)
{
    if (!g_iso.active) return 0U;

    uint32_t warmup_need = (uint32_t)(g_iso.warmup_sec * g_iso.fs_hz + 0.5f);

    if (!g_iso.warmup_done) {
        g_iso.warmup_samples++;

        /* gravity mean from first 1.0s (or fewer if fs lower) */
        uint32_t g_need = (uint32_t)(1.0f * g_iso.fs_hz + 0.5f);
        if (g_iso.gravity_cnt < g_need) {
            g_iso.gravity_sum += az_raw_ms2;
            g_iso.gravity_cnt++;
        }

        if (g_iso.warmup_samples >= warmup_need) {
            if (g_iso.gravity_cnt > 0U) {
                g_iso.gravity_offset_ms2 = g_iso.gravity_sum / (float)g_iso.gravity_cnt;
            } else {
                g_iso.gravity_offset_ms2 = 9.81f;
            }
            g_iso.warmup_done = 1U;

            /* start real window clean */
            g_iso.sample_count = 0U;
            g_iso.buf_index = 0U;
            memset(g_iso.buf, 0, sizeof(g_iso.buf));
            g_iso.rms = g_iso.vdv = g_iso.peak = 0.0f;

            Wk_Init(&g_iso.wk, g_iso.fs_hz);
        }

        (void)now_ms;
        return 1U;
    }

    /* remove DC/gravity */
    float az_hp = az_raw_ms2 - g_iso.gravity_offset_ms2;

    /* weighting */
    float aw = Wk_Process(&g_iso.wk, az_hp);

    /* store sequentially (simple window, not rolling) */
    if (g_iso.sample_count < g_iso.window_samples) {
        g_iso.buf[g_iso.sample_count] = aw;
        g_iso.sample_count++;
    }

    /* peak */
    float aa = fabsf(aw);
    if (aa > g_iso.peak) g_iso.peak = aa;

    /* auto stop */
    if (g_iso.sample_count >= g_iso.window_samples) {
        ISO2631_Stop();
        return 2U;
    }

    (void)now_ms;
    return 1U;
}
