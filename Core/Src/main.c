/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9488.h"
#include "adc_driver.h"
#include "scope.h"
#include "pga.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ADC1 双模式 (保留未启用) */
#define ADC_BUF_SIZE  4
ALIGN_32BYTES(uint32_t adc_dual_buffer[ADC_BUF_SIZE]) __attribute__((section(".RAM_D2"))) = {0};

/* ADC2 单通道 DMA (CH18, 差分, 16bit) */
#define ADC2_BUF_SIZE  4
ALIGN_32BYTES(uint16_t adc2_buf[ADC2_BUF_SIZE]) __attribute__((section(".RAM_D2"))) = {0};

/* ── adc_driver.c 需要的旧版全局符号 (保留, 未使用) ── */
#define HIST_LEN_OLD  480
int16_t  g_hist1[HIST_LEN_OLD] = {0};
int16_t  g_hist2[HIST_LEN_OLD] = {0};
volatile uint32_t g_hist_idx  = 0;
float diff_voltage_1 = 0.0f;
float diff_voltage_2 = 0.0f;

/* ── 三档采样时间 (ADC_CLK=20MHz, DIV4) ── */
static const uint32_t sampling_times[] = {
    ADC_SAMPLETIME_8CYCLES_5,    /*  8.5 周期 → ~1.05µs/点 →  200µs/屏 */
    ADC_SAMPLETIME_64CYCLES_5,   /* 64.5 周期 → ~3.85µs/点 →  770µs/屏 */
    ADC_SAMPLETIME_387CYCLES_5,  /* 387.5周期 → ~20.0µs/点 → 4.0ms/屏 */
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc2_buf, ADC2_BUF_SIZE * 2);

        for (int i = 0; i < ADC2_BUF_SIZE; i++) {
            int16_t raw = (int16_t)adc2_buf[i];
            Scope_PushSample(0, raw);   /* CH1=0, CH2=raw */
        }
    }
}

static void set_adc2_sampling_time(uint32_t sampling_time)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    /* 先停止 */
    HAL_ADC_Stop_DMA(&hadc2);

    sConfig.Channel = ADC_CHANNEL_18;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampling_time;
    sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /* 重新启动 */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC2_BUF_SIZE);

    /* 计算实际 ADC 采样间隔 (ADC_CLK=20MHz, 50ns/cycle) */
    static const float adc_cycle_ns = 50.0f;
    static const float fixed_cycles = 12.5f;   /* 逐次逼近固定周期 */
    float sample_ns;
    if (sampling_time == ADC_SAMPLETIME_8CYCLES_5)    sample_ns = 8.5f * 50.0f*1.6f*1.28f*1.1f*1.06f;
    else if (sampling_time == ADC_SAMPLETIME_64CYCLES_5)  sample_ns = 64.5f * 50.0f*1.87f*1.1f;
    else if (sampling_time == ADC_SAMPLETIME_387CYCLES_5) sample_ns = 387.5f * 50.0f*2;
    else sample_ns = 8.5f * 50.0f;
    g_adc_interval_us = (sample_ns + (12.5f * 50.0f)) / 1000.0f;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  Scope_Init();
  PGA_SET(1.0);

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
      Error_Handler();

  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC2_BUF_SIZE) != HAL_OK)
      Error_Handler();

  /* 初始化 ADC 间隔 (对应默认 SamplingTime=8.5) */
  g_adc_interval_us = 1.05f;

  HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  Encoder_Init();      /* 启动 TIM4 编码器 */

  bool running = true;
  bool coupling_ac = false;
  int  edit_mode = 0;
  uint32_t btn_down_tick = 0;
  bool btn_was_down = false;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    bool btn_now = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
    if (btn_now && !btn_was_down) btn_down_tick = HAL_GetTick();
    if (!btn_now && btn_was_down) {
        uint32_t held = HAL_GetTick() - btn_down_tick;
        if (held > 800) {
            edit_mode = (edit_mode + 1) % 3;   /* 0=TB, 1=VS, 2=CPL */
        } else if (held > 50) {
            if (edit_mode == 2) {
                coupling_ac = !coupling_ac;
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,
                    coupling_ac ? GPIO_PIN_SET : GPIO_PIN_RESET);
            } else {
                running = !running;
                if (running) Scope_Clear();
            }
        }
    }
    btn_was_down = btn_now;

    if (running) {
        int32_t delta = Encoder_ReadDelta();
        if (delta != 0) {
            //Scope_Clear();
            if (edit_mode == 0) {
                /* ── 旋转修改 ADC 采样时间 (硬件时基) ── */
                int cur = (int)g_scope.timebase;
                if (delta > 0)
                    cur = (cur + 1) % TIMEBASE_COUNT;
                else
                    cur = (cur == 0) ? TIMEBASE_COUNT - 1 : (cur - 1);
                g_scope.timebase = (ScopeTimebase)cur;

                set_adc2_sampling_time(sampling_times[g_scope.timebase]);
            } else if (edit_mode == 1) {
                /* ── 旋转切换垂直灵敏度 ── */
                int cur = (int)g_scope.vert_scale;
                if (delta > 0) cur = (cur + 1) % VERT_COUNT;
                else cur = (cur == 0) ? VERT_COUNT - 1 : (cur - 1);
                g_scope.vert_scale = (ScopeVertScale)cur;
            }
        }

        /* 同步 UI 状态到 g_scope (Scope_Draw 需要) */
        g_scope.edit_mode = edit_mode;
        g_scope.coupling_ac = coupling_ac;

        Scope_ProcessFrame();
        Scope_Measure();
        if (g_scope.frame_ready) {
            Scope_Draw();
        }
    } else {
        LCD_DrawString(60, 140, "PAUSED", LCD_RED, LCD_BLACK);
    }
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
