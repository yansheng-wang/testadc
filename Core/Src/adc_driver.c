#include "adc_driver.h"
#include "ILI9488.h"
#include <stdio.h>
#include <string.h>

/* 电压转换 (差分模式, 16-bit 有符号) */
#define RAW2MV(r)  (3.3f * ((float)(uint16_t)(r) / 32768.0f - 1.0f) * 1000.0f)

/* ── ISR 写入的全局缓冲 (定义在 main.c) ── */
#define HIST_LEN  480
extern int16_t  g_hist1[HIST_LEN];
extern int16_t  g_hist2[HIST_LEN];
extern volatile uint32_t g_hist_idx;

/* 面板用 */
extern float diff_voltage_1, diff_voltage_2;

/* 波形区 240×320 */
#define W  240
#define H  320

static int mv2y(float mv) {
    int c = H / 2;                   /* 中心 Y = 160 */
    int y = c - (int)(mv / 25.0f);   /* 25mV/px → ±1600mV*/
    if (y < 0) y = 0;
    if (y > H) y = H;
    return y;
}

void ADC_DisplayOnLCD(void) {
    uint32_t idx = g_hist_idx;  /* 原子快照 */

    /* 1. 清屏 */
    LCD_FillRect(0, 0, W - 1, H - 1, LCD_BLACK);

    /* 2. 网格 */
    for (int d = 0; d <= 8; d++) {
        int y = d * 40;
        uint16_t c = (d == 4) ? 0xC618 : 0x2104;
        LCD_FillRect(0, y, W - 1, y, c);
    }
    for (int d = 0; d <= 10; d++) {
        int x = d * 24;
        uint16_t c = (d == 5) ? 0xC618 : 0x2104;
        LCD_FillRect(x, 0, x, H - 1, c);
    }

    /* 3. 波形 — 相邻点间画竖线消散点 */
    uint32_t n = idx < W ? idx : W;
    uint32_t s = idx >= W ? idx - W : 0;
    int prev_y1 = -1, prev_y2 = -1;
    for (uint32_t i = 0; i < n; i++) {
        int y1 = mv2y(RAW2MV((int32_t)g_hist1[(s + i) % HIST_LEN]));
        int y2 = mv2y(RAW2MV((int32_t)g_hist2[(s + i) % HIST_LEN]));
        if (i > 0 && prev_y1 >= 0) {
            int ya = y1 < prev_y1 ? y1 : prev_y1;
            int yb = y1 > prev_y1 ? y1 : prev_y1;
            if (ya >= 0 && yb < H) LCD_FillRect(i, ya, i, yb, LCD_YELLOW);
        }
        if (i > 0 && prev_y2 >= 0) {
            int ya = y2 < prev_y2 ? y2 : prev_y2;
            int yb = y2 > prev_y2 ? y2 : prev_y2;
            if (ya >= 0 && yb < H) LCD_FillRect(i, ya, i, yb, LCD_CYAN);
        }
        prev_y1 = y1; prev_y2 = y2;
    }

    /* 4. 面板 */
    char b[32];
    LCD_DrawString(244, 4,   "SCOPE", LCD_WHITE, LCD_BLACK);
    snprintf(b, 32, "CH1:%dmV", (int)(diff_voltage_1 * 1000.0f));
    LCD_DrawString(244, 20,  b, LCD_YELLOW, LCD_BLACK);
    snprintf(b, 32, "CH2:%dmV", (int)(diff_voltage_2 * 1000.0f));
    LCD_DrawString(244, 34,  b, LCD_CYAN, LCD_BLACK);
    snprintf(b, 32, "Cnt:%lu", (unsigned long)idx);
    LCD_DrawString(244, 48,  b, 0x8410, LCD_BLACK);
}