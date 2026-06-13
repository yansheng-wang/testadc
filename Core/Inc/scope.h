/**
  ******************************************************************************
  * @file    scope.h
  * @brief   DSO 示波器引擎 — 触发 / 时基 / 垂直灵敏度 / 参数测量
  ******************************************************************************
  */
#ifndef __SCOPE_H__
#define __SCOPE_H__

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════
 *  屏幕网格规格 (需求第2条)
 * ═══════════════════════════════════════
 *  水平: 10 div × 20 点/div = 200 点
 *  垂直:  8 div × 32 级/div = 256 级
 */
#define SCOPE_GRID_DIV_X   10
#define SCOPE_GRID_DIV_Y   8
#define SCOPE_PTS_PER_DIV  20
#define SCOPE_LVL_PER_DIV  32

#define SCOPE_W  (SCOPE_GRID_DIV_X * SCOPE_PTS_PER_DIV)  /* 200 */
#define SCOPE_H  (SCOPE_GRID_DIV_Y * SCOPE_LVL_PER_DIV)  /* 256 */

/* 波形区在 LCD 上的偏移 */
#define SCOPE_X0  0
#define SCOPE_Y0  0

/* ═══════════════════════════════════════
 *  时基 (需求第3条)
 * ═══════════════════════════════════════ */
typedef enum {
    TIMEBASE_20US_DIV = 0,   /* 20 μs/div */
    TIMEBASE_200US_DIV,      /* 0.2 ms/div */
    TIMEBASE_200MS_DIV,      /* 0.2 s/div */
    TIMEBASE_COUNT
} ScopeTimebase;

/* 当前实际采样间隔 (由 TIM3 硬件决定, 单位 μs) */
extern float g_sample_interval_us;

/* ═══════════════════════════════════════
 *  垂直灵敏度 (需求第4条)
 * ═══════════════════════════════════════ */
typedef enum {
    VERT_100MV_DIV = 0,      /* 0.1 V/div */
    VERT_1V_DIV,             /* 1 V/div */
    VERT_COUNT
} ScopeVertScale;

/* 每档对应的 mV/div */
extern const float scope_vdiv_mv[VERT_COUNT];

/* ═══════════════════════════════════════
 *  触发 (需求第5条)
 * ═══════════════════════════════════════ */
typedef enum {
    TRIG_MODE_AUTO = 0,      /* 自动触发（无触发也刷新） */
    TRIG_MODE_NORMAL,        /* 正常触发 */
    TRIG_MODE_SINGLE,        /* 单次触发 (需求第1条) */
    TRIG_MODE_COUNT
} ScopeTrigMode;

typedef enum {
    TRIG_EDGE_RISING = 0,
    TRIG_EDGE_FALLING,
} ScopeTrigEdge;

typedef struct {
    ScopeTrigMode mode;         /* 触发模式 */
    ScopeTrigEdge edge;         /* 触发边沿 */
    int16_t        level_raw;   /* 触发电平 (ADC 原始值, ±32768) */
    bool           armed;       /* 单次触发已预备 */
    bool           trigged;     /* 本轮已触发 */
    uint32_t       pre_samples; /* 触发前保留点数 */
} ScopeTrigger;

/* ═══════════════════════════════════════
 *  测量结果 (需求第6条)
 * ═══════════════════════════════════════ */
typedef struct {
    float freq_hz;             /* 频率 (Hz) */
    float vpp_mv;              /* 峰峰值 (mV) */
    float vmin_mv;             /* 最小值 (mV) */
    float vmax_mv;             /* 最大值 (mV) */
    float duty_pct;            /* 占空比 (%) — 可选 */
    bool  valid;               /* 测量数据有效 */
} ScopeMeasure;

/* ═══════════════════════════════════════
 *  主结构体
 * ═══════════════════════════════════════ */
typedef struct {
    ScopeTimebase  timebase;
    ScopeVertScale vert_scale;
    ScopeTrigger   trigger;
    ScopeMeasure   measure_ch1;
    ScopeMeasure   measure_ch2;

    /* 波形缓冲 (环形) */
    int16_t *buf1;
    int16_t *buf2;
    uint32_t buf_len;         /* 缓冲区长度 */
    uint32_t wr_idx;          /* 写入位置 */

    /* 当前帧显示参数 */
    uint32_t disp_start;      /* 显示起始索引 */
    uint32_t disp_len;        /* 显示点数 */

    /* 触发搜索 */
    uint32_t trig_idx;        /* 触发点索引 */
    bool     frame_ready;     /* 新帧就绪 */

    /* 实际采样率(Hz) 和 总采样计数 */
    float    sample_rate_hz;
    uint32_t total_samples;
} ScopeState;

extern ScopeState g_scope;

/* ═══════════════════════════════════════
 *  API
 * ═══════════════════════════════════════ */

/* 初始化 (从 TIM3 硬件寄存器读取采样率) */
void Scope_Init(void);

/* 清空波形缓冲 (切换时基时调用) */
void Scope_Clear(void);

/* 每收到一个采样对 (ISR 中调用) */
void Scope_PushSample(int16_t raw1, int16_t raw2);

/* 主循环中调用 — 搜索触发点, 标记帧就绪 */
void Scope_ProcessFrame(void);

/* 波形显示 (LCD 绘制) — 页 1 */
void Scope_Draw(void);

/* 参数测量全屏显示 — 页 2 */
void Scope_DrawParams(void);

/* 时基 / 垂直档位切换 */
void Scope_SetTimebase(ScopeTimebase tb);
void Scope_SetVertScale(ScopeVertScale vs);

/* 触发设置 */
void Scope_SetTrigMode(ScopeTrigMode mode);
void Scope_SetTrigLevel(int16_t raw_level);
void Scope_TrigEdge(ScopeTrigEdge edge);
void Scope_SingleTrig(void);     /* 单次触发按键 */

/* 测量 (自动更新 g_scope.measure_ch1/2) */
void Scope_Measure(void);

/* ── 辅助: ADC原始值 → 电压(mV) ── */
static inline float Scope_Raw2mV(int16_t raw) {
    /* 差分 ADC, 0V 输入对应 32768。
       已知: Converted_value = 32768 * (1 + Vdiff/3.3)
       反推: Vdiff = 3.3 * (raw/32768 - 1) */
    return 3.3f * ((float)(uint16_t)(raw) / 32768.0f - 1.0f) * 1000.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* __SCOPE_H__ */
