/**
  ******************************************************************************
  * @file    scope.c
  * @brief   DSO 示波器引擎 — 触发/时基/垂直灵敏度/测量/显示
  ******************************************************************************
  */
#include "scope.h"
#include "ILI9488.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

float g_sample_interval_hw_us = 1.0f;  /* 硬件固定 (TIM3 → 1μs) */
float g_sample_interval_us = 1.0f;     /* 时序信息 (含降采样) */

const float scope_vdiv_mv[VERT_COUNT] = { 100.0f, 1000.0f };

#define SCOPE_BUF_LEN  2048
static int16_t scope_buf1[SCOPE_BUF_LEN];
static int16_t scope_buf2[SCOPE_BUF_LEN];

ScopeState g_scope;

void Scope_Init(void) {
    g_scope.buf1 = scope_buf1;
    g_scope.buf2 = scope_buf2;
    g_scope.buf_len = SCOPE_BUF_LEN;
    g_scope.wr_idx = 0;
    g_scope.disp_start = 0;
    g_scope.disp_len = 0;
    g_scope.frame_ready = false;
    g_scope.total_samples = 0;

    g_scope.timebase = TIMEBASE_200US_DIV;
    g_scope.vert_scale = VERT_1V_DIV;
    g_scope.decimation = 1;

    g_scope.trigger.mode = TRIG_MODE_AUTO;
    g_scope.trigger.edge = TRIG_EDGE_RISING;
    g_scope.trigger.level_raw = 0;
    g_scope.trigger.armed = false;
    g_scope.trigger.trigged = false;
    g_scope.trigger.pre_samples = SCOPE_W;

    g_scope.measure_ch1.valid = false;
    g_scope.measure_ch2.valid = false;

    memset(scope_buf1, 0, sizeof(scope_buf1));
    memset(scope_buf2, 0, sizeof(scope_buf2));

    {
        uint32_t tim_clk = HAL_RCC_GetPCLK1Freq() * 2U;
        uint32_t psc = TIM3->PSC;
        uint32_t arr = TIM3->ARR;
        float cnt_freq = (float)tim_clk / (float)(psc + 1U);
        float trgo_freq = cnt_freq / (float)(arr + 1U);
        g_sample_interval_hw_us = 1000000.0f / trgo_freq;
        g_sample_interval_us = g_sample_interval_hw_us;
        g_scope.sample_rate_hz = trgo_freq;
    }
}

/* ISR: 全速存入每个点. */
void Scope_PushSample(int16_t raw1, int16_t raw2) {
    uint32_t idx = g_scope.wr_idx % SCOPE_BUF_LEN;
    g_scope.buf1[idx] = raw1;
    g_scope.buf2[idx] = raw2;
    g_scope.wr_idx++;
    g_scope.total_samples++;

    if (!g_scope.trigger.trigged) {
        int16_t cur  = raw1;
        int16_t prev = (g_scope.wr_idx > 1)
            ? g_scope.buf1[(g_scope.wr_idx - 2) % SCOPE_BUF_LEN]
            : cur;
        int16_t level = g_scope.trigger.level_raw;

        bool edge_ok;
        if (g_scope.trigger.edge == TRIG_EDGE_RISING)
            edge_ok = (prev < level && cur >= level);
        else
            edge_ok = (prev > level && cur <= level);

        if (edge_ok) {
            g_scope.trigger.trigged = true;
            g_scope.trig_idx = idx;
        }
        if (g_scope.trigger.mode == TRIG_MODE_AUTO &&
            g_scope.wr_idx >= SCOPE_BUF_LEN) {
            g_scope.trigger.trigged = true;
            g_scope.trig_idx = idx;
        }
    }
    /* 不移除 frame_ready — 让主循环控制显示帧 */
}

/* 帧处理: 全速缓冲, 按 decimation 跳点显示 */
void Scope_ProcessFrame(void) {
    uint32_t step = g_scope.decimation; if (step < 1) step = 1;
    uint32_t need = SCOPE_W * step + 10;
    uint32_t total = (g_scope.wr_idx < SCOPE_BUF_LEN)
        ? g_scope.wr_idx : SCOPE_BUF_LEN;

    if (total < need) {
        g_scope.frame_ready = false;
        return;
    }

    if (g_scope.trigger.trigged) {
        uint32_t center = g_scope.trig_idx;
        uint32_t pre = g_scope.trigger.pre_samples * step;
        g_scope.disp_start = (center >= pre) ? (center - pre) : 0;
        g_scope.disp_len = SCOPE_W;
        g_scope.frame_ready = true;
        if (g_scope.trigger.mode != TRIG_MODE_SINGLE)
            g_scope.trigger.trigged = false;
    } else if (g_scope.trigger.mode == TRIG_MODE_AUTO) {
        g_scope.disp_start = (g_scope.wr_idx + SCOPE_BUF_LEN - SCOPE_W * step) % SCOPE_BUF_LEN;
        g_scope.disp_len = SCOPE_W;
        g_scope.frame_ready = true;
    }
}

/* 测量: 按 decimation 跳点 */
static void do_measure(const int16_t *buf, uint32_t start, uint32_t len,
                       ScopeMeasure *m) {
    if (len < 4) { m->valid = false; return; }
    uint32_t step = g_scope.decimation; if (step < 1) step = 1;

    int16_t vmin = 32767, vmax = -32768;
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        int16_t v = buf[(start + i * step) % SCOPE_BUF_LEN];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        sum += (float)v;
    }

    m->vmin_mv = Scope_Raw2mV(vmin);
    m->vmax_mv = Scope_Raw2mV(vmax);
    m->vpp_mv  = m->vmax_mv - m->vmin_mv;

    int16_t avg_raw = (int16_t)(sum / (float)len);
    uint32_t crossings = 0;
    uint32_t first_edge = 0, last_edge = 0;
    for (uint32_t i = 1; i < len; i++) {
        int16_t prev = buf[(start + (i - 1) * step) % SCOPE_BUF_LEN];
        int16_t cur  = buf[(start + i * step) % SCOPE_BUF_LEN];
        if (prev < avg_raw && cur >= avg_raw) {
            crossings++;
            if (crossings == 1) first_edge = i;
            last_edge = i;
        }
    }

    if (crossings >= 2) {
        float dt_us = (float)(last_edge - first_edge) * g_sample_interval_hw_us * (float)step;
        float period_us = dt_us / (float)(crossings - 1);
        if (period_us > 0.0f)
            m->freq_hz = 1000000.0f / period_us;
        else
            m->freq_hz = 0.0f;
    } else {
        m->freq_hz = 0.0f;
    }
    m->valid = true;
}

void Scope_Measure(void) {
    if (!g_scope.frame_ready) return;
    do_measure(g_scope.buf1, g_scope.disp_start, g_scope.disp_len, &g_scope.measure_ch1);
    do_measure(g_scope.buf2, g_scope.disp_start, g_scope.disp_len, &g_scope.measure_ch2);
}

void Scope_Clear(void) {
    g_scope.wr_idx = 0;
    g_scope.disp_start = 0;
    g_scope.disp_len = 0;
    g_scope.frame_ready = false;
    g_scope.total_samples = 0;
    g_scope.trigger.trigged = false;
    g_scope.trigger.armed = (g_scope.trigger.mode == TRIG_MODE_SINGLE);
    memset(scope_buf1, 0, sizeof(scope_buf1));
    memset(scope_buf2, 0, sizeof(scope_buf2));
}

/* 波形页 */
void Scope_Draw(void) {
    if (!g_scope.frame_ready) {
        /* 仅首帧: 画一个红色方块验证 LCD 能显示 */
        static int once = 0;
        if (!once) { LCD_FillRect(0, 0, 50, 50, LCD_RED); once = 1; }
        return;
    }

    uint32_t start = g_scope.disp_start;
    uint32_t len   = g_scope.disp_len;
    uint32_t step  = g_scope.decimation; if (step < 1) step = 1;
    float mv_per_px = scope_vdiv_mv[g_scope.vert_scale] / (float)SCOPE_LVL_PER_DIV;
    int zero_y = SCOPE_Y0 + SCOPE_H / 2;

    LCD_FillRect(SCOPE_X0, SCOPE_Y0, SCOPE_X0 + SCOPE_W - 1, SCOPE_Y0 + SCOPE_H - 1, LCD_BLACK);

    for (int d = 0; d <= SCOPE_GRID_DIV_Y; d++) {
        int y = SCOPE_Y0 + d * SCOPE_LVL_PER_DIV;
        uint16_t c = (d == SCOPE_GRID_DIV_Y / 2) ? 0xC618 : 0x2104;
        if (y < SCOPE_Y0 + SCOPE_H)
            LCD_FillRect(SCOPE_X0, y, SCOPE_X0 + SCOPE_W - 1, y, c);
    }
    for (int d = 0; d <= SCOPE_GRID_DIV_X; d++) {
        int x = SCOPE_X0 + d * SCOPE_PTS_PER_DIV;
        uint16_t c = (d == SCOPE_GRID_DIV_X / 2) ? 0xC618 : 0x2104;
        if (x < SCOPE_X0 + SCOPE_W)
            LCD_FillRect(x, SCOPE_Y0, x, SCOPE_Y0 + SCOPE_H - 1, c);
    }

    int prev_y1 = SCOPE_Y0 + SCOPE_H + 1;
    int prev_y2 = SCOPE_Y0 + SCOPE_H + 1;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t bi = (start + i * step) % SCOPE_BUF_LEN;
        int16_t r1 = g_scope.buf1[bi];
        int16_t r2 = g_scope.buf2[bi];
        int y1 = zero_y - (int)(Scope_Raw2mV(r1) / mv_per_px);
        int y2 = zero_y - (int)(Scope_Raw2mV(r2) / mv_per_px);

        if (y1 < SCOPE_Y0) y1 = SCOPE_Y0;
        if (y1 >= SCOPE_Y0 + SCOPE_H) y1 = SCOPE_Y0 + SCOPE_H - 1;
        if (y2 < SCOPE_Y0) y2 = SCOPE_Y0;
        if (y2 >= SCOPE_Y0 + SCOPE_H) y2 = SCOPE_Y0 + SCOPE_H - 1;

        int px = SCOPE_X0 + i;
        if (prev_y1 < SCOPE_Y0 + SCOPE_H) {
            int ya = y1 < prev_y1 ? y1 : prev_y1;
            int yb = y1 > prev_y1 ? y1 : prev_y1;
            LCD_FillRect(px, ya, px, yb, LCD_YELLOW);
        }
        if (prev_y2 < SCOPE_Y0 + SCOPE_H) {
            int ya = y2 < prev_y2 ? y2 : prev_y2;
            int yb = y2 > prev_y2 ? y2 : prev_y2;
            LCD_FillRect(px, ya, px, yb, LCD_CYAN);
        }
        prev_y1 = y1; prev_y2 = y2;
    }

    /* 面板 */
    int px_p = SCOPE_X0 + SCOPE_W + 4;
    int py = 4;
    char b[40];
    LCD_DrawString(px_p, py, "DSO", LCD_WHITE, LCD_BLACK); py += 14;

    const char *tb[] = {"20us/div","0.2ms/div","0.2s/div"};
    snprintf(b, 40, "TB:%s", tb[g_scope.timebase]);
    LCD_DrawString(px_p, py, b, LCD_YELLOW, LCD_BLACK); py += 14;

    const char *vs[] = {"0.1V","1V"};
    snprintf(b, 40, "VS:%s", vs[g_scope.vert_scale]);
    LCD_DrawString(px_p, py, b, 0x8410, LCD_BLACK); py += 16;

    const char *tm[] = {"Auto","Norm","Sing"};
    snprintf(b, 40, "Trg:%s", tm[g_scope.trigger.mode]);
    LCD_DrawString(px_p, py, b, (g_scope.trigger.mode==TRIG_MODE_SINGLE)?LCD_YELLOW:LCD_WHITE, LCD_BLACK);
    py += 16;

    if (g_scope.measure_ch1.valid) {
        ScopeMeasure *m = &g_scope.measure_ch1;
        LCD_DrawString(px_p, py, "CH1:", LCD_YELLOW, LCD_BLACK); py += 12;
        snprintf(b, 40, "f:%.1fHz", (double)m->freq_hz);
        LCD_DrawString(px_p, py, b, LCD_YELLOW, LCD_BLACK); py += 12;
        snprintf(b, 40, "Vpp:%dmV", (int)m->vpp_mv);
        LCD_DrawString(px_p, py, b, LCD_YELLOW, LCD_BLACK); py += 12;
        snprintf(b, 40, "V:%dmV", (int)m->vmin_mv);
        LCD_DrawString(px_p, py, b, LCD_YELLOW, LCD_BLACK); py += 14;
    }
    if (g_scope.measure_ch2.valid) {
        ScopeMeasure *m = &g_scope.measure_ch2;
        LCD_DrawString(px_p, py, "CH2:", LCD_CYAN, LCD_BLACK); py += 12;
        snprintf(b, 40, "f:%.1fHz", (double)m->freq_hz);
        LCD_DrawString(px_p, py, b, LCD_CYAN, LCD_BLACK); py += 12;
        snprintf(b, 40, "Vpp:%dmV", (int)m->vpp_mv);
        LCD_DrawString(px_p, py, b, LCD_CYAN, LCD_BLACK); py += 12;
    }
}

/* 测量页 (不依赖 frame_ready, 直接读取上次测量值) */
void Scope_DrawParams(void) {
    LCD_FillRect(0, 0, 319, 255, LCD_BLACK);
    int py = 10;
    char b[40];
    LCD_DrawString(10, py, "MEASURE", LCD_WHITE, LCD_BLACK); py += 20;

    const char *tb[] = {"20us","0.2ms","0.2s"};
    const char *vs[] = {"0.1V","1V"};
    snprintf(b, 40, "TB:%s  VS:%s", tb[g_scope.timebase], vs[g_scope.vert_scale]);
    LCD_DrawString(10, py, b, 0x8410, LCD_BLACK); py += 24;

    if (g_scope.measure_ch1.valid) {
        LCD_FillRect(10, py, 309, py+1, 0x2104); py += 6;
        ScopeMeasure *m = &g_scope.measure_ch1;
        LCD_DrawString(10, py, "CH1", LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Freq : %9.1f Hz", (double)m->freq_hz);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vpp  : %9d mV", (int)m->vpp_mv);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmin : %9d mV", (int)m->vmin_mv);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmax : %9d mV", (int)m->vmax_mv);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 26;
    }
    if (g_scope.measure_ch2.valid) {
        LCD_FillRect(10, py, 309, py+1, 0x2104); py += 6;
        ScopeMeasure *m = &g_scope.measure_ch2;
        LCD_DrawString(10, py, "CH2", LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Freq : %9.1f Hz", (double)m->freq_hz);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vpp  : %9d mV", (int)m->vpp_mv);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmin : %9d mV", (int)m->vmin_mv);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmax : %9d mV", (int)m->vmax_mv);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK);
    }
    LCD_DrawString(10, 275, "Enc: Page", 0x8410, LCD_BLACK);
}

/* UI */
void Scope_SetTimebase(ScopeTimebase tb) { g_scope.timebase = tb; }
void Scope_SetVertScale(ScopeVertScale vs) { g_scope.vert_scale = vs; }
void Scope_SetTrigMode(ScopeTrigMode mode) {
    g_scope.trigger.mode = mode;
    g_scope.trigger.trigged = false;
    g_scope.trigger.armed = (mode == TRIG_MODE_SINGLE);
}
void Scope_SetTrigLevel(int16_t raw_level) { g_scope.trigger.level_raw = raw_level; }
void Scope_TrigEdge(ScopeTrigEdge edge) { g_scope.trigger.edge = edge; }
void Scope_SingleTrig(void) {
    g_scope.trigger.mode = TRIG_MODE_SINGLE;
    g_scope.trigger.armed = true;
    g_scope.trigger.trigged = false;
}