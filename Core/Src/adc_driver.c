#include "adc_driver.h"
#include "ILI9488.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ── 电压转换宏（差分补码 0~65535 → ±3.3V） ── */
#define ADC_FULLSCALE     32768.0f
#define ADC_RAW_TO_MV(raw)  (3.3f * ((float)(uint16_t)(raw) / ADC_FULLSCALE - 1.0f) * 1000.0f)
#define ADC_RAW_TO_V(raw)   (3.3f * ((float)(uint16_t)(raw) / ADC_FULLSCALE - 1.0f))

/* ── LCD 布局 ── */
#define SW  240
#define SH  320
#define SX  0
#define SY  0
#define SP  240
#define HS  480
#define IX  240
#define DH  24
#define DV  40

/* ── 示波器参数 ── */
static float    g_v_div = 1.0f, g_s_div = 0.2f;
static int      g_vi = 2, g_si = 0;
static const float v_tbl[] = { 0.01f, 0.1f, 1.0f };
static const float s_tbl[] = { 0.2f, 0.0002f, 0.00002f };
static const char  *s_lbl[] = { "0.2s/div", "0.2ms/div", "20us/div" };

static int32_t  g_vert_offset_mv = 0;

/* ── 波形历史缓冲区 ── */
static int16_t  g_h1[HS], g_h2[HS];
static uint32_t g_hi = 0;
static float    g_fps = 0;
static uint32_t g_fc = 0, g_ft = 0;
static int      g_meas_frame = 0;

static int      g_grid_drawn = 0;
static float    g_last_grid_v = -1.0f;

/* ── 外部 ADC 数据（由 main.c 中的 HAL_ADC_ConvCpltCallback 更新） ── */
extern uint32_t adc_dual_buffer[16];
extern float diff_voltage_1, diff_voltage_2;

/* ── 采样率 (TIM3 触发 20 Hz) ── */
#define ADC_SAMPLE_RATE  20.0f
#define SAMPLES_PER_CALL 16

/* ══════════ 波形采集 ══════════ */
static void wave_cap(void) {
    /* 每次 HAL_ADC_ConvCpltCallback 触发时提供 16 个双通道采样点 */
    for (int i = 0; i < SAMPLES_PER_CALL; i++) {
        uint32_t combined = adc_dual_buffer[i];
        uint16_t raw1 = (uint16_t)(combined & 0xFFFF);
        uint16_t raw2 = (uint16_t)((combined >> 16) & 0xFFFF);

        uint32_t hi_pos = g_hi % HS;
        g_h1[hi_pos] = (int16_t)raw1;
        g_h2[hi_pos] = (int16_t)raw2;
        g_hi++;
    }
}

/* ── 网格绘制 ── */
static void draw_grid(void) {
    if (g_grid_drawn && g_v_div == g_last_grid_v) return;
    g_last_grid_v = g_v_div;
    LCD_FillRect(SX, SY, SX + SW, SY + SH, LCD_BLACK);

    for (int d = 0; d <= 8; d++) {
        int y = SY + d * DV; if (y > SY + SH) y = SY + SH;
        uint16_t clr = (d == 4) ? 0xC618 : 0x2104;
        if (d == 4) {
            for (int dy = -1; dy <= 1; dy++) {
                int yy = y + dy; if (yy < SY || yy > SY + SH) continue;
                LCD_FillRect(SX, yy, SX + SW - 1, yy, clr);
            }
        } else {
            for (int x = SX; x < SX + SW; x += 12)
                LCD_FillRect(x, y, x + 2, y, clr);
        }
        char lb[8]; int val_mv = (int)((4 - d) * g_v_div * 1000.0f);
        snprintf(lb, 8, "%+dmV", val_mv);
        LCD_DrawString(SX + 2, y - 6, lb, 0xC618, LCD_BLACK);
    }

    for (int d = 0; d <= 10; d++) {
        int x = SX + d * DH; if (x >= SX + SW) x = SX + SW - 1;
        uint16_t clr = (d == 5) ? 0xC618 : 0x2104;
        if (d == 5) {
            for (int dx = -1; dx <= 1; dx++) {
                int xx = x + dx; if (xx < SX || xx >= SX + SW) continue;
                LCD_FillRect(xx, SY, xx, SY + SH - 1, clr);
            }
        } else {
            for (int y = SY; y < SY + SH; y += 12)
                LCD_FillRect(x, y, x, y + 2, clr);
        }
    }
    g_grid_drawn = 1;
}

/* ── 电压 → 像素 Y ── */
static int v2y(float mv) {
    int c = SY + 4 * DV;
    float mpx = (g_v_div * 1000.0f) / (float)DV;
    int y = c - (int)((mv + (float)g_vert_offset_mv) / mpx);
    if (y < SY) y = SY;
    if (y > SY + SH) y = SY + SH;
    return y;
}

/* ── 波形绘制 ── */
static void draw_wf(const int16_t *h, uint32_t st, uint32_t cnt, uint16_t clr) {
    if (cnt < 2) return;
    int px = SX;
    float mv0 = ADC_RAW_TO_MV((int32_t)h[st % HS]);
    int py = v2y(mv0);
    LCD_DrawPixel((uint16_t)px, (uint16_t)py, clr);
    for (uint32_t i = 1; i < cnt; i++) {
        float mv1 = ADC_RAW_TO_MV((int32_t)h[(st + i) % HS]);
        int cx = SX + (int)i, cy = v2y(mv1);
        int dy = cy - py, dx = cx - px;
        int s = (dy < 0 ? -dy : dy) > dx ? (dy < 0 ? -dy : dy) : dx;
        if (s < 1) s = 1;
        int last_y = -999;
        for (int k = 0; k <= s; k += 2) {
            int x = px + (dx * k) / s, y = py + (dy * k) / s;
            if (x >= SX && x < SX + SW && y >= SY && y < SY + SH) {
                if (y != last_y) {
                    LCD_DrawPixel((uint16_t)x, (uint16_t)y, clr);
                    last_y = y;
                }
            }
        }
        px = cx; py = cy;
    }
}

/* ── 测量计算 ── */
static void calc_meas(const int16_t *h, float *freq, float *ampl, float *vmin, float *vmax) {
    uint32_t n_meas = g_hi;
    int32_t minv = 32767, maxv = -32767; int zc = 0, lz = -1;
    for (uint32_t i = n_meas > 240 ? n_meas - 240 : 0; i < n_meas; i++) {
        int16_t v = h[i % HS];
        if (v < minv) minv = v; if (v > maxv) maxv = v;
        if (i > (n_meas > 240 ? n_meas - 240 : 0) && v >= 0 && h[(uint32_t)(i - 1) % HS] < 0) {
            if (lz > 0) { int p = (int)i - lz; if (p > 2) zc++; } lz = (int)i;
        }
    }
    *freq = 0; *ampl = 0;
    *vmin = ADC_RAW_TO_MV(minv);
    *vmax = ADC_RAW_TO_MV(maxv);
    if (zc > 1) {
        float ap = (float)(n_meas - (n_meas > 240 ? n_meas - 240 : 0)) / (float)(zc);
        *freq = ADC_SAMPLE_RATE * (float)SAMPLES_PER_CALL / ap;
    }
    *ampl = ADC_RAW_TO_V(maxv) - ADC_RAW_TO_V(minv);
}

/* ── 测量面板 ── */
static void draw_meas(void) {
    g_meas_frame++;
    if (g_meas_frame % 4 != 0) return;

    int x = IX, y = 4; char b[32];
    float f1, a1, f2, a2, vmin1, vmax1, vmin2, vmax2;

    LCD_FillRect(x + 1, 0, 319, SH, 0x0841);
    for (int iy = 0; iy < SH; iy += 8) LCD_DrawPixel(x, iy, 0x4208);

    LCD_DrawString(x + 4, y, "--- SCOPE ---", LCD_WHITE, 0x0841); y += 12;
    snprintf(b, 32, "Ch:BOTH");
    LCD_DrawString(x + 4, y, b, LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "V:%.4gV", (double)g_v_div);
    LCD_DrawString(x + 4, y, b, LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "T:%s", s_lbl[g_si]);
    LCD_DrawString(x + 4, y, b, LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "Rate:%.0fHz", (double)ADC_SAMPLE_RATE);
    LCD_DrawString(x + 4, y, b, LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "FPS:%5.1f", (double)g_fps);
    LCD_DrawString(x + 4, y, b, LCD_YELLOW, 0x0841); y += 14;

    calc_meas(g_h1, &f1, &a1, &vmin1, &vmax1);
    calc_meas(g_h2, &f2, &a2, &vmin2, &vmax2);

    LCD_DrawString(x + 4, y, "-- CH1(MEAS) --", LCD_YELLOW, 0x0841); y += 12;
    snprintf(b, 32, "F:%6.1fHz", (double)f1); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
    snprintf(b, 32, "Vpp:%7.2fmV", (double)(a1 * 1000.0f)); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
    snprintf(b, 32, "Min:%6.1f Max:%6.1fmV", (double)vmin1, (double)vmax1); LCD_DrawString(x + 4, y, b, 0x8410, 0x0841); y += 14;

    LCD_DrawString(x + 4, y, "-- CH2(MEAS) --", LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "F:%6.1fHz", (double)f2); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
    snprintf(b, 32, "Vpp:%7.2fmV", (double)(a2 * 1000.0f)); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
    snprintf(b, 32, "Min:%6.1f Max:%6.1fmV", (double)vmin2, (double)vmax2); LCD_DrawString(x + 4, y, b, 0x8410, 0x0841); y += 14;
}

/* ══════════ 主显示接口 ══════════ */
void ADC_DisplayOnLCD(void) {
    wave_cap();
    draw_grid();

    uint32_t t = g_hi;
    uint32_t dc = t < SP ? t : SP;
    uint32_t si = t >= SP ? t - SP : 0;

    static int g_skip_cnt = 0;
    if (g_si != 2 || g_skip_cnt == 0) {
        LCD_FillRect(SX, SY, SX + SW, SY + SH, LCD_BLACK);
        for (int d = 0; d <= 8; d++) {
            int y = SY + d * DV; if (y > SY + SH) y = SY + SH;
            uint16_t glr = (d == 4) ? 0x630C : 0x2104;
            for (int x = SX; x < SX + SW; x += 12) LCD_FillRect(x, y, x + 2, y, glr);
            char lb[8]; int val_mv = (int)((4 - d) * g_v_div * 1000.0f);
            snprintf(lb, 8, "%+dmV", val_mv); LCD_DrawString(SX + 2, y - 6, lb, 0x8410, LCD_BLACK);
        }
        for (int d = 0; d <= 10; d++) {
            int x = SX + d * DH; if (x >= SX + SW) x = SX + SW - 1;
            uint16_t glr = (d == 5) ? 0x630C : 0x2104;
            for (int y = SY; y < SY + SH; y += 12) LCD_FillRect(x, y, x, y + 2, glr);
        }
        draw_wf(g_h1, si, dc, LCD_YELLOW);
        draw_wf(g_h2, si, dc, LCD_CYAN);
    }
    g_skip_cnt = (g_skip_cnt + 1) % 3;

    draw_meas();

    g_fc++;
    uint32_t now = HAL_GetTick();
    if (now - g_ft >= 1000) { g_fps = (float)g_fc * 1000.0f / (float)(now - g_ft); g_fc = 0; g_ft = now; }
}