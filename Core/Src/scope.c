/**
  ******************************************************************************
  * @file    scope.c
  * @brief   DSO 示波器引擎实现 — 触发/时基/垂直灵敏度/测量/显示
  ******************************************************************************
  */
#include "scope.h"
#include "ILI9488.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ── 全局: 实际采样间隔(μs), 由 TIM3 硬件决定 ── */
float g_sample_interval_us = 51.0f;   /* 默认 19.6kHz */

/* ── 垂直灵敏度常量表 ── */
const float scope_vdiv_mv[VERT_COUNT] = {
    100.0f,     /* 0.1 V/div */
    1000.0f,    /* 1 V/div */
};

/* ── 环形缓冲 ── */
#define SCOPE_BUF_LEN  2048
static int16_t scope_buf1[SCOPE_BUF_LEN];
static int16_t scope_buf2[SCOPE_BUF_LEN];

ScopeState g_scope;

/* ═══════════════════════════════════════
 *  初始化
 * ═══════════════════════════════════════ */
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

    /* ── 从 TIM3 寄存器计算实际采样间隔 ── */
    {
        uint32_t tim_clk = HAL_RCC_GetPCLK1Freq() * 2U;
        uint32_t psc = TIM3->PSC;
        uint32_t arr = TIM3->ARR;
        float cnt_freq = (float)tim_clk / (float)(psc + 1U);
        float trgo_freq = cnt_freq / (float)(arr + 1U);
        g_sample_interval_us = 1000000.0f / trgo_freq;
        g_scope.sample_rate_hz = trgo_freq;
    }
}

/* 数据喂入 (ISR 中调用) */
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

        bool edge_ok = false;
        if (g_scope.trigger.edge == TRIG_EDGE_RISING) {
            edge_ok = (prev < level && cur >= level);
        } else {
            edge_ok = (prev > level && cur <= level);
        }

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
    g_scope.frame_ready = false;
}

/* 帧处理 */
void Scope_ProcessFrame(void) {
    uint32_t total = (g_scope.wr_idx < SCOPE_BUF_LEN)
        ? g_scope.wr_idx : SCOPE_BUF_LEN;

    if (total < SCOPE_W + 10) {
        g_scope.frame_ready = false;
        return;
    }

    if (g_scope.trigger.trigged) {
        uint32_t center = g_scope.trig_idx;
        uint32_t pre = g_scope.trigger.pre_samples;
        if (center >= pre) {
            g_scope.disp_start = center - pre;
        } else {
            g_scope.disp_start = 0;
        }
        g_scope.disp_len = SCOPE_W;
        g_scope.frame_ready = true;

        if (g_scope.trigger.mode != TRIG_MODE_SINGLE) {
            g_scope.trigger.trigged = false;
        }
    }
    else if (g_scope.trigger.mode == TRIG_MODE_AUTO) {
        g_scope.disp_start = (g_scope.wr_idx - SCOPE_W + SCOPE_BUF_LEN) % SCOPE_BUF_LEN;
        g_scope.disp_len = SCOPE_W;
        g_scope.frame_ready = true;
    }
}

/* ── 测量 (需求第6条) ── */
static void do_measure(const int16_t *buf, uint32_t start, uint32_t len,
                       ScopeMeasure *m) {
    if (len < 4) { m->valid = false; return; }

    int16_t vmin = 32767, vmax = -32768;
    float sum = 0.0f;

    for (uint32_t i = 0; i < len; i++) {
        int16_t v = buf[(start + i) % SCOPE_BUF_LEN];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        sum += (float)v;
    }

    /* 用数据手册公式, 不加人工缩放 */
    m->vmin_mv = Scope_Raw2mV(vmin);
    m->vmax_mv = Scope_Raw2mV(vmax);
    m->vpp_mv  = m->vmax_mv - m->vmin_mv;

    /* 频率 */
    int16_t avg_raw = (int16_t)(sum / (float)len);
    uint32_t crossings = 0;
    uint32_t first_edge = 0, last_edge = 0;

    for (uint32_t i = 1; i < len; i++) {
        int16_t prev = buf[(start + i - 1) % SCOPE_BUF_LEN];
        int16_t cur  = buf[(start + i) % SCOPE_BUF_LEN];
        if (prev < avg_raw && cur >= avg_raw) {
            crossings++;
            if (crossings == 1) first_edge = i;
            last_edge = i;
        }
    }

    if (crossings >= 2) {
        float dt_us = (float)(last_edge - first_edge) * g_sample_interval_us;
        float period_us = dt_us / (float)(crossings - 1);
        if (period_us > 0.0f) {
            m->freq_hz = 1000000.0f / period_us;
        } else {
            m->freq_hz = 0.0f;
        }
    } else {
        m->freq_hz = 0.0f;
    }

    m->valid = true;
}

void Scope_Measure(void) {
    if (!g_scope.frame_ready) return;
    uint32_t start = g_scope.disp_start;
    uint32_t len   = g_scope.disp_len;
    if (len < 4) return;
    do_measure(g_scope.buf1, start, len, &g_scope.measure_ch1);
    do_measure(g_scope.buf2, start, len, &g_scope.measure_ch2);
}

/* ═══════════════════════════════════════
 *  清空波形缓冲
 * ═══════════════════════════════════════ */
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

/* ── 页 1: 波形 + 面板 ── */
void Scope_Draw(void) {
    if (!g_scope.frame_ready) return;

    uint32_t start = g_scope.disp_start;
    uint32_t len   = g_scope.disp_len;
    float mv_per_px = scope_vdiv_mv[g_scope.vert_scale] / (float)SCOPE_LVL_PER_DIV;
    int zero_y = SCOPE_Y0 + SCOPE_H / 2;

    /* 1. 清波形区 */
    LCD_FillRect(SCOPE_X0, SCOPE_Y0, SCOPE_X0 + SCOPE_W - 1, SCOPE_Y0 + SCOPE_H - 1, LCD_BLACK);

    /* 2. 网格 */
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

    /* 3. 波形 */
    int prev_y1 = SCOPE_Y0 + SCOPE_H + 1;
    int prev_y2 = SCOPE_Y0 + SCOPE_H + 1;
    for (uint32_t i = 0; i < len; i++) {
        int16_t r1 = g_scope.buf1[(start + i) % SCOPE_BUF_LEN];
        int16_t r2 = g_scope.buf2[(start + i) % SCOPE_BUF_LEN];
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

    /* 4. 面板 */
    int px_panel = SCOPE_X0 + SCOPE_W + 4;
    int py = 4;
    char b[40];

    LCD_DrawString(px_panel, py, "DSO", LCD_WHITE, LCD_BLACK);
    py += 14;

    snprintf(b, 40, "Fs:%.1fkHz", (double)(g_scope.sample_rate_hz / 1000.0f));
    LCD_DrawString(px_panel, py, b, 0x8410, LCD_BLACK);
    py += 14;

    const char *vs_name[] = {"0.1V", "1V"};
    snprintf(b, 40, "VS:%s", vs_name[g_scope.vert_scale]);
    LCD_DrawString(px_panel, py, b, 0x8410, LCD_BLACK);
    py += 16;

    const char *tm_name[] = {"Auto", "Norm", "Sing"};
    snprintf(b, 40, "Trg:%s", tm_name[g_scope.trigger.mode]);
    LCD_DrawString(px_panel, py, b,
        (g_scope.trigger.mode == TRIG_MODE_SINGLE) ? LCD_YELLOW : LCD_WHITE,
        LCD_BLACK);
    py += 16;

    /* CH1 */
    if (g_scope.measure_ch1.valid) {
        ScopeMeasure *m = &g_scope.measure_ch1;
        LCD_DrawString(px_panel, py, "CH1:", LCD_YELLOW, LCD_BLACK); py += 12;
        snprintf(b, 40, "f:%.1fHz", (double)m->freq_hz);
        LCD_DrawString(px_panel, py, b, LCD_YELLOW, LCD_BLACK); py += 12;
        snprintf(b, 40, "Vpp:%dmV", (int)m->vpp_mv);
        LCD_DrawString(px_panel, py, b, LCD_YELLOW, LCD_BLACK); py += 12;
        snprintf(b, 40, "V:%dmV", (int)m->vmin_mv);
        LCD_DrawString(px_panel, py, b, LCD_YELLOW, LCD_BLACK); py += 14;
    }

    /* CH2 */
    if (g_scope.measure_ch2.valid) {
        ScopeMeasure *m = &g_scope.measure_ch2;
        LCD_DrawString(px_panel, py, "CH2:", LCD_CYAN, LCD_BLACK); py += 12;
        snprintf(b, 40, "f:%.1fHz", (double)m->freq_hz);
        LCD_DrawString(px_panel, py, b, LCD_CYAN, LCD_BLACK); py += 12;
        snprintf(b, 40, "Vpp:%dmV", (int)m->vpp_mv);
        LCD_DrawString(px_panel, py, b, LCD_CYAN, LCD_BLACK); py += 12;
    }
}

/* ── 页 2: 全屏参数测量 (利用刷新率, 显示大字体) ── */
void Scope_DrawParams(void) {
    if (!g_scope.frame_ready) {
        LCD_FillRect(0, 0, 319, 255, LCD_BLACK);
        LCD_DrawString(20, 100, "Wait...", LCD_WHITE, LCD_BLACK);
        return;
    }

    LCD_FillRect(0, 0, 319, 255, LCD_BLACK);

    int py = 10;
    char b[40];

    /* 标题栏 */
    LCD_DrawString(10, py, "MEASURE", LCD_WHITE, LCD_BLACK);
    py += 20;

    const char *tb_name[] = {"20us", "0.2ms", "0.2s"};
    const char *vs_name[] = {"0.1V", "1V"};
    snprintf(b, 40, "TB:%s  VS:%s", tb_name[g_scope.timebase], vs_name[g_scope.vert_scale]);
    LCD_DrawString(10, py, b, 0x8410, LCD_BLACK);
    py += 24;

    /* CH1 */
    if (g_scope.measure_ch1.valid) {
        LCD_FillRect(10, py, 309, py + 1, 0x2104);
        py += 6;
        ScopeMeasure *m = &g_scope.measure_ch1;

        LCD_DrawString(10, py, "CH1", LCD_YELLOW, LCD_BLACK);
        py += 18;

        snprintf(b, 40, "Freq : %9.1f Hz", (double)m->freq_hz);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vpp  : %9d mV", (int)m->vpp_mv);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmin : %9d mV", (int)m->vmin_mv);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmax : %9d mV", (int)m->vmax_mv);
        LCD_DrawString(10, py, b, LCD_YELLOW, LCD_BLACK);
        py += 26;
    }

    /* CH2 */
    if (g_scope.measure_ch2.valid) {
        LCD_FillRect(10, py, 309, py + 1, 0x2104);
        py += 6;
        ScopeMeasure *m = &g_scope.measure_ch2;

        LCD_DrawString(10, py, "CH2", LCD_CYAN, LCD_BLACK);
        py += 18;

        snprintf(b, 40, "Freq : %9.1f Hz", (double)m->freq_hz);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vpp  : %9d mV", (int)m->vpp_mv);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmin : %9d mV", (int)m->vmin_mv);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK); py += 18;
        snprintf(b, 40, "Vmax : %9d mV", (int)m->vmax_mv);
        LCD_DrawString(10, py, b, LCD_CYAN, LCD_BLACK);
    }

    /* 底部: 编码器提示 */
    LCD_DrawString(10, 275, "Enc: Page", 0x8410, LCD_BLACK);
}

/* ═══════════════════════════════════════
 *  UI 控制
 * ═══════════════════════════════════════ */
void Scope_SetTimebase(ScopeTimebase tb) {
    g_scope.timebase = tb;
}

void Scope_SetVertScale(ScopeVertScale vs) {
    g_scope.vert_scale = vs;
}

void Scope_SetTrigMode(ScopeTrigMode mode) {
    g_scope.trigger.mode = mode;
    g_scope.trigger.trigged = false;
    g_scope.trigger.armed = (mode == TRIG_MODE_SINGLE);
}

void Scope_SetTrigLevel(int16_t raw_level) {
    g_scope.trigger.level_raw = raw_level;
}

void Scope_TrigEdge(ScopeTrigEdge edge) {
    g_scope.trigger.edge = edge;
}

void Scope_SingleTrig(void) {
    g_scope.trigger.mode = TRIG_MODE_SINGLE;
    g_scope.trigger.armed = true;
    g_scope.trigger.trigged = false;
}