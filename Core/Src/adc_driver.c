#include "adc_driver.h"
#include "ILI9488.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ────────── 电压转换宏 ────────── */
#define ADC_FULLSCALE     32768.0f
#define ADC_RAW_TO_MV(raw)  (3.3f * ((float)(uint16_t)(raw) / ADC_FULLSCALE - 1.0f) * 1000.0f)
#define ADC_RAW_TO_V(raw)   (3.3f * ((float)(uint16_t)(raw) / ADC_FULLSCALE - 1.0f))

/* ────────── 布局常量 ────────── */
#define SW  240
#define SH  320
#define SX  0
#define SY  0
#define SP  240
#define HS  480
#define IX  240
#define DH  24
#define DV  40

/* ────────── 波形状态 ────────── */
static float    g_v_div = 1.0f;
static int32_t  g_vert_offset_mv = 0;

static int16_t  g_h1[HS], g_h2[HS];
static uint32_t g_hi = 0;

extern uint32_t adc_dual_buffer[16];

/* ────────── 电压 → 屏幕 Y 坐标 ────────── */
static int v2y(float mv) {
    int c = SY + 4 * DV;
    float mpx = (g_v_div * 1000.0f) / (float)DV;
    int y = c - (int)((mv + (float)g_vert_offset_mv) / mpx);
    if (y < SY) y = SY;
    if (y > SY + SH) y = SY + SH;
    return y;
}

/* ═══════════════════════════════════════
 *  6 个屏幕驱动测试函数
 * ═══════════════════════════════════════ */

/* 测试 1 ── 红绿蓝白纯色填充 */
void LCD_Test_FillColors(void) {
    uint16_t colors[] = { LCD_RED, LCD_GREEN, LCD_BLUE, LCD_WHITE, LCD_BLACK };
    for (int i = 0; i < 5; i++) {
        LCD_FillColor(colors[i]);
        HAL_Delay(500);
    }
    LCD_FillColor(LCD_BLACK);
}

/* 测试 2 ── 四角画色块 */
void LCD_Test_CornerBlocks(void) {
    LCD_FillRect(0, 0, 50, 50, LCD_RED);
    LCD_FillRect(0, LCD_GetHeight() - 50, 50, LCD_GetHeight() - 1, LCD_GREEN);
    LCD_FillRect(LCD_GetWidth() - 50, 0, LCD_GetWidth() - 1, 50, LCD_BLUE);
    LCD_FillRect(LCD_GetWidth() - 50, LCD_GetHeight() - 50, LCD_GetWidth() - 1, LCD_GetHeight() - 1, LCD_WHITE);
}

/* 测试 3 ── 十字交叉线 */
void LCD_Test_Cross(void) {
    int w = LCD_GetWidth(), h = LCD_GetHeight();
    LCD_FillColor(LCD_BLACK);
    for (int i = 0; i < w; i++) LCD_DrawPixel(i, h / 2, LCD_RED);
    for (int i = 0; i < h; i++) LCD_DrawPixel(w / 2, i, LCD_YELLOW);
    LCD_DrawString(10, 10, "CROSS OK", LCD_GREEN, LCD_BLACK);
}

/* 测试 4 ── 字体与颜色 */
void LCD_Test_Text(void) {
    LCD_FillColor(LCD_BLACK);
    LCD_DrawString(10, 10, "Hello World!", LCD_WHITE, LCD_BLACK);
    LCD_DrawString(10, 30, "Red Text", LCD_RED, LCD_BLACK);
    LCD_DrawString(10, 50, "Green BG", LCD_WHITE, LCD_GREEN);
    LCD_DrawString(10, 70, "12345", LCD_YELLOW, LCD_BLUE);
}

/* 测试 5 ── 渐变彩条 */
void LCD_Test_Gradient(void) {
    uint16_t colors[] = { LCD_RED, LCD_YELLOW, LCD_GREEN, LCD_CYAN, LCD_BLUE, LCD_MAGENTA, LCD_WHITE };
    int band = LCD_GetHeight() / 7;
    for (int i = 0; i < 7; i++)
        LCD_FillRect(0, i * band, LCD_GetWidth() - 1, (i + 1) * band - 1, colors[i]);
}

/* 测试 6 ── 对角线 + 矩形边框 */
void LCD_Test_DiagRect(void) {
    int w = LCD_GetWidth(), h = LCD_GetHeight();
    LCD_FillColor(LCD_BLACK);
    /* 对角线 */
    for (int i = 0; i < (w < h ? w : h); i++) LCD_DrawPixel(i, i, LCD_YELLOW);
    /* 空心矩形边框 */
    for (int i = 50; i < 150; i++) {
        LCD_DrawPixel(i, 50, LCD_CYAN);
        LCD_DrawPixel(i, 100, LCD_CYAN);
        LCD_DrawPixel(50, i, LCD_CYAN);
        LCD_DrawPixel(150, i, LCD_CYAN);
    }
}

/* ═══════════════════════════════════════
 *  主示波器显示函数
 * ═══════════════════════════════════════ */
void ADC_DisplayOnLCD(void) {
    /* 读取 ADC 数据 */
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_dual_buffer, 64);
    for (int i = 0; i < 16; i++) {
        uint32_t combined = adc_dual_buffer[i];
        g_h1[g_hi % HS] = (int16_t)(combined & 0xFFFF);
        g_h2[g_hi % HS] = (int16_t)((combined >> 16) & 0xFFFF);
        g_hi++;
    }

    /* ── 网格 ── */
    static int  g_grid_drawn = 0;
    static float g_last_grid_v = -1.0f;
    if (!g_grid_drawn || g_v_div != g_last_grid_v) {
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
    } else {
        LCD_FillRect(SX, SY, SX + SW, SY + SH, LCD_BLACK);
    }

    /* ── 波形 ── */
    uint32_t dc = g_hi < SP ? g_hi : SP;
    uint32_t si = g_hi >= SP ? g_hi - SP : 0;
    for (uint32_t i = 0; i < dc; i++) {
        float mv = ADC_RAW_TO_MV((int32_t)g_h1[(si + i) % HS]);
        int y = v2y(mv);
        if (y >= SY && y < SY + SH)
            LCD_DrawPixel(SX + i, y, LCD_YELLOW);
    }
    for (uint32_t i = 0; i < dc; i++) {
        float mv = ADC_RAW_TO_MV((int32_t)g_h2[(si + i) % HS]);
        int y = v2y(mv);
        if (y >= SY && y < SY + SH)
            LCD_DrawPixel(SX + i, y, LCD_CYAN);
    }

    /* ── 面板 ── */
    {
        char b[32];
        int x = IX, y = 4;
        LCD_FillRect(x + 1, 0, 319, SH, 0x0841);

        LCD_DrawString(x + 4, y, "--- SCOPE ---", LCD_WHITE, 0x0841); y += 14;
        snprintf(b, 32, "V:%dmV", (int)(g_v_div * 1000.0f));
        LCD_DrawString(x + 4, y, b, LCD_CYAN, 0x0841); y += 14;

        /* 计算 Vpp */
        int16_t min1 = 32767, max1 = -32768;
        int16_t min2 = 32767, max2 = -32768;
        uint32_t cnt = g_hi < HS ? g_hi : HS;
        for (uint32_t i = 0; i < cnt; i++) {
            int16_t v1 = g_h1[i], v2 = g_h2[i];
            if (v1 < min1) min1 = v1;
            if (v1 > max1) max1 = v1;
            if (v2 < min2) min2 = v2;
            if (v2 > max2) max2 = v2;
        }
        snprintf(b, 32, "CH1:%+dmV", (int)(ADC_RAW_TO_V((int32_t)max1) - ADC_RAW_TO_V((int32_t)min1)) * 1000);
        LCD_DrawString(x + 4, y, b, LCD_YELLOW, 0x0841); y += 14;
        snprintf(b, 32, "CH2:%+dmV", (int)(ADC_RAW_TO_V((int32_t)max2) - ADC_RAW_TO_V((int32_t)min2)) * 1000);
        LCD_DrawString(x + 4, y, b, LCD_CYAN, 0x0841); y += 14;

        snprintf(b, 32, "Cnt:%ld", (long)g_hi);
        LCD_DrawString(x + 4, y, b, 0x8410, 0x0841);
    }
}