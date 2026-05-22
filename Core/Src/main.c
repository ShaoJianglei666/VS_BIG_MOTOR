/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — IF 开环精简版
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOC.h"
#include "vf_ctrl.h"
#include "key.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USE_VF_CTRL 定义于 vf_ctrl.h（供 main.c 和 stm32g4xx_it.c 共享） */

/*--- VF 模式参数（USE_VF_CTRL=1 时生效） ---*/
#if USE_VF_CTRL
#define SPEED_RUN          200.0f   /* 目标转速 (RPM) */
#define SPEED_STEP_RPM     50.0f    /* 加减速步长 (RPM) */
#define VF_TARGET_ACCEL    200.0f   /* VF 加速度 (RPM/s) */
#endif

/*--- IF/FOC 模式参数（USE_VF_CTRL=0 时生效） ---*/
#if !USE_VF_CTRL
#define SPEED_RUN          500      /* 目标转速 (RPM) */
#define SPEED_STEP_RPM     50.0f
#endif

/*--- ADC 旋钮调速映射（两模式共用） ---*/
#define ADC_RPM_MIN_VAL    4000     /* 旋钮最小时 ADC 读数（映射起点） */
#define ADC_RPM_MAX_VAL    2100     /* 旋钮最大时 ADC 读数（映射终点） */
#define ADC_RPM_MIN_SPEED  300.0f   /* 最低目标转速 (RPM) */
#define ADC_RPM_MAX_SPEED  1250.0f  /* 最高目标转速 (RPM) */
/* USER CODE END PD

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t s_u8MotorRunning = 0;   /* 电机运行标志: 0=停止, 1=运行 */

/*--- ADC1 规则组滑动窗口滤波器 ---*/
#define ADC_FILTER_WIN     20                      /* 滑动窗口大小 */
static uint16_t s_u16AdcBuf[ADC_FILTER_WIN] = {0}; /* 采样缓冲区 */
static uint8_t  s_u8AdcIdx = 0;                    /* 当前写入位置 */
static uint32_t s_u32AdcSum = 0;                   /* 窗口内累加和 */
/* USER CODE END PV

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_Start(void);
static void Motor_Stop(void);
static uint16_t ADC_InjectedReadOnce(ADC_HandleTypeDef *hadc);
static void VOFA_SendTelemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  读取 ADC1 规则组值并做滑动窗口滤波
  * @retval 滤波后的 ADC 原始值（无符号 12-bit）
  */
static uint16_t ADC_GetFilteredValue(void)
{
    uint16_t u16NewVal = (uint16_t)HAL_ADC_GetValue(&hadc1);

    /* 减去最旧值，加入新值 */
    s_u32AdcSum -= s_u16AdcBuf[s_u8AdcIdx];
    s_u16AdcBuf[s_u8AdcIdx] = u16NewVal;
    s_u32AdcSum += u16NewVal;

    /* 环形指针步进（非 2 的幂，用取模） */
    s_u8AdcIdx = (s_u8AdcIdx + 1) % ADC_FILTER_WIN;

    return (uint16_t)(s_u32AdcSum / ADC_FILTER_WIN);
}

/**
  * @brief  启动电机：开启 PWM + ADC 中断 + 设定目标转速
  */
static void Motor_Start(void)
{
    if (s_u8MotorRunning) return;
    s_u8MotorRunning = 1;

    FOC_Init();

    /* 上桥前先给三相 50% 占空比，避免 PWM 刚启动时沿用 CCR=0。 */
    TIM1->CCR1 = PWM_HALF_CYCLE;
    TIM1->CCR2 = PWM_HALF_CYCLE;
    TIM1->CCR3 = PWM_HALF_CYCLE;
    TIM1->CNT  = 0;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    /* 启动 TIM1 六路 PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 启动 ADC 注入组（TIM1_CH4 硬件触发） */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    /* 使能 TIM1 更新中断（10kHz FOC 控制环） */
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    /* 设定目标转速 → 状态机自动进入 IF 开环 */
    g_stCtrl.f32TargetRpm = (float)SPEED_RUN;
}

/**
  * @brief  停止电机：关 PWM + 关中断 + 复位 FOC 状态
  */
static void Motor_Stop(void)
{
    if (!s_u8MotorRunning) return;
    s_u8MotorRunning = 0;

    /* 先通知状态机停止 */
    g_stCtrl.f32TargetRpm = 0.0f;

    /* 关闭 TIM1 更新中断 */
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

    TIM1->CCR1 = PWM_HALF_CYCLE;
    TIM1->CCR2 = PWM_HALF_CYCLE;
    TIM1->CCR3 = PWM_HALF_CYCLE;

    /* 关闭 ADC 注入组中断 */
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop_IT(&hadc2);

    /* 关闭 TIM1 六路 PWM 输出 */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* FOC 状态机复位 */
    FOC_Init();
}

#if USE_VF_CTRL
/**
  * @brief  VF 模式启动电机
  */
static void VF_Motor_Start(void)
{
    if (s_u8MotorRunning) return;
    s_u8MotorRunning = 1;

    VF_Init();

    /* 设置加速度 */
    VF_SetAccel(VF_TARGET_ACCEL);

    /* 上桥前先给三相 50% 占空比 */
    TIM1->CCR1 = PWM_HALF_CYCLE_VF;
    TIM1->CCR2 = PWM_HALF_CYCLE_VF;
    TIM1->CCR3 = PWM_HALF_CYCLE_VF;
    TIM1->CNT  = 0;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    /* 启动 TIM1 六路 PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 启动 ADC 注入组 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    /* 使能 TIM1 更新中断 */
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    /* 设定目标转速 → VF 状态机自动进入 ALIGN → RAMPING */
    VF_SetTargetRpm((float)SPEED_RUN);
}

/**
  * @brief  VF 模式停止电机
  */
static void VF_Motor_Stop(void)
{
    if (!s_u8MotorRunning) return;
    s_u8MotorRunning = 0;

    VF_Stop();

    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

    TIM1->CCR1 = PWM_HALF_CYCLE_VF;
    TIM1->CCR2 = PWM_HALF_CYCLE_VF;
    TIM1->CCR3 = PWM_HALF_CYCLE_VF;

    HAL_ADCEx_InjectedStop_IT(&hadc1);
    HAL_ADCEx_InjectedStop_IT(&hadc2);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    VF_Init();
}
#endif /* USE_VF_CTRL */

static uint16_t ADC_InjectedReadOnce(ADC_HandleTypeDef *hadc)
{
    uint32_t saved_jsqr = hadc->Instance->JSQR;
    uint16_t value;

    hadc->Instance->JSQR &= ~ADC_JSQR_JEXTEN;
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC | ADC_FLAG_JEOS);

    if (HAL_ADCEx_InjectedStart(hadc) == HAL_OK)
    {
        if (HAL_ADCEx_InjectedPollForConversion(hadc, 10U) != HAL_OK)
        {
            Error_Handler();
        }
        value = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        HAL_ADCEx_InjectedStop(hadc);
    }
    else
    {
        Error_Handler();
        value = 0U;
    }

    hadc->Instance->JSQR = saved_jsqr;
    return value;
}

static int32_t VOFA_FloatToMilli(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 1000.0f + 0.5f);
    }

    return (int32_t)(value * 1000.0f - 0.5f);
}

static int VOFA_AppendAmps(char *buf, int pos, int size, float value)
{
    int32_t scaled = (int32_t)(value * 1000.0f + 0.5f);
    int32_t abs_scaled = scaled;
    const char *sign = "";

    if (scaled < 0)
    {
        sign = "-";
        abs_scaled = -scaled;
    }

    return pos + snprintf(&buf[pos], (size_t)(size - pos), "%s%ld.%03ld",
                          sign,
                          (long)(abs_scaled / 1000),
                          (long)(abs_scaled % 1000));
}

static int VOFA_AppendMilli(char *buf, int pos, int size, float value)
{
    int32_t scaled = VOFA_FloatToMilli(value);
    int32_t abs_scaled = scaled;
    const char *sign = "";

    if (scaled < 0)
    {
        sign = "-";
        abs_scaled = -scaled;
    }

    return pos + snprintf(&buf[pos], (size_t)(size - pos), "%s%ld.%03ld",
                          sign,
                          (long)(abs_scaled / 1000),
                          (long)(abs_scaled % 1000));
}

static float VOFA_RadToDeg(float rad)
{
    return rad * 57.2957795f;
}

static float VOFA_WrapDeg180(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

#if USE_VF_CTRL
static void VOFA_SendTelemetry(void)
{
    char buf[256];
    int pos = 0;
    float theta_deg = VOFA_RadToDeg(g_stVFCtrl.f32Theta);
    float theta_obs_deg = VOFA_RadToDeg(g_stVFCtrl.stObs.f32ThetaObs);
    float theta_err_deg = VOFA_WrapDeg180(theta_obs_deg - theta_deg);

    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "vf:");

    /* 1. 目标转速 (RPM) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.f32TargetRpm);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 2. id_ref (pu) — IF 模式下恒为 0 */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), 0.0f);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 3. iq_ref (pu) — 电流幅值目标 */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.f32IfITarget);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 4. id (pu) — Park 变换诊断值 */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.f32Id);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 5. iq (pu) — Park 变换诊断值 */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.f32Iq);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 6. ud (pu) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.f32VdRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 7. uq (pu) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.f32VqRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 8. 生成电角度 (deg) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 9. 观测器电角度 (deg) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_obs_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 10. 角度误差 (deg) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_err_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 11. 观测器速度 (RPM) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.stObs.f32SpeedObs);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 12. 目前阶段 */
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%u", (unsigned int)g_stVFCtrl.eStage);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 13. 速度环总输出 (pu Iq) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.stPiSpeed.fOutPrev);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 14. 速度环 P 项 (pu) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.stPiSpeed.fProportional);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 15. 速度环 I 项 (pu) */
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stVFCtrl.stPiSpeed.fIntegral);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");

    /* 16. ADC1 规则组值 (滑动滤波后) */
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%u", (unsigned int)ADC_GetFilteredValue());
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "\n");

    if ((pos > 0) && (pos < (int)sizeof(buf)))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)pos, 10U);
    }
}
#else
static void VOFA_SendTelemetry(void)
{
    char buf[384];
    int pos = 0;
    float theta_open_deg = VOFA_RadToDeg(g_stMotor.f32Theta);
    float theta_obs_deg = VOFA_RadToDeg(g_stLuenberger.f32ThetaObs);
    float theta_err_deg = VOFA_WrapDeg180(theta_obs_deg - theta_open_deg);

    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "foc:");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32TargetRpm);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32RpmRamp);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendAmps(buf, pos, sizeof(buf), g_stCtrl.f32IdRef * FOC_BASE_CURRENT_A);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendAmps(buf, pos, sizeof(buf), g_stCtrl.f32IqRef * FOC_BASE_CURRENT_A);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendAmps(buf, pos, sizeof(buf), g_stMotor.f32Id * FOC_BASE_CURRENT_A);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendAmps(buf, pos, sizeof(buf), g_stMotor.f32Iq * FOC_BASE_CURRENT_A);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32UdRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32UqRef);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_open_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_obs_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), theta_err_deg);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stLuenberger.f32SpeedObs);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32RampedTargetRpm);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stPiSpeed.fOutPrev);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32SpdProportional);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32SpdIntegral);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stCtrl.f32IqBase);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendAmps(buf, pos, sizeof(buf), g_stMotor.f32Ia * FOC_BASE_CURRENT_A);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendAmps(buf, pos, sizeof(buf), g_stMotor.f32Ib * FOC_BASE_CURRENT_A);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32Ia);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), g_stMotor.f32Ib);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), FOC_fIaOffsetAdc);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos = VOFA_AppendMilli(buf, pos, sizeof(buf), FOC_fIbOffsetAdc);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%u", (unsigned int)(ADC1->JDR1));
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%u", (unsigned int)(ADC2->JDR1));
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",%u", (unsigned int)g_stCtrl.u16DiagStage);
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, ",");
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%u", (unsigned int)ADC_GetFilteredValue());
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "\n");

    if ((pos > 0) && (pos < (int)sizeof(buf)))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)pos, 10U);
    }
}
#endif /* USE_VF_CTRL */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  HAL_Delay(10);
  
  /*--- FOC 初始化 ---*/
  FOC_Init();

  /* 电流零点校准（PWM 未启动时软件触发注入转换取平均值） */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(100);
  {
      int32_t sum_a = 0, sum_b = 0;
      for (int i = 0; i < 128; i++) {
          sum_a += ADC_InjectedReadOnce(&hadc1);
          sum_b += ADC_InjectedReadOnce(&hadc2);
      }
      FOC_fIaOffsetAdc = (float)sum_a / 128.0f;
      FOC_fIbOffsetAdc = (float)sum_b / 128.0f;
  }

  /* TIM1 中断优先级（仅配置，不使能，由 Motor_Start 使能） */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  HAL_TIM_Base_Start(&htim6);//启动 TIM6 作为ADC规则组节拍（10ms）
  HAL_ADC_Start(&hadc1);           // 启动 ADC1 规则组（TIM6 硬件触发，无中断）
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*--- 按键扫描 ---*/
    KEY_Scan();

    /*--- 单键启停：释放一次切换一次状态 ---*/
    if (KEY_GetEvent(KEY_ID_RUN) == KEY_EVENT_RELEASE)
    {
        if (s_u8MotorRunning)
        {
#if USE_VF_CTRL
            VF_Motor_Stop();
#else
            Motor_Stop();
#endif
        }
        else
        {
            /* 安全启动：ADC 读数 > 4000 时才允许启动，否则无法启动 */
            if (ADC_GetFilteredValue() > ADC_RPM_MIN_VAL)
            {
#if USE_VF_CTRL
                VF_Motor_Start();
#else
                Motor_Start();
#endif
            }
        }
    }

#if USE_VF_CTRL
    /*--- VF 模式：PC9 加速 +50rpm ---*/
    if (KEY_GetEvent(KEY_ID_SPEED_UP) == KEY_EVENT_PRESS)
    {
        if (s_u8MotorRunning)
        {
            float fNewTarget = g_stVFCtrl.f32TargetRpm + SPEED_STEP_RPM;
            if (fNewTarget > 3000.0f)
            {
                fNewTarget = 3000.0f;
            }
            VF_SetTargetRpm(fNewTarget);
        }
    }
#else
    /*--- 闭环后单击 PC9：目标速度 +50rpm ---*/
    if (KEY_GetEvent(KEY_ID_SPEED_UP) == KEY_EVENT_PRESS)
    {
#if !FOC_FORCE_OPEN_LOOP
        if (s_u8MotorRunning && (g_stCtrl.eMode == FOC_MODE_CLOSED_LOOP))
        {
            float fNewTarget = g_stCtrl.f32TargetRpm + SPEED_STEP_RPM;
            if (fNewTarget > FOC_OBS_MAX_SPEED_RPM)
            {
                fNewTarget = FOC_OBS_MAX_SPEED_RPM;
            }
            g_stCtrl.f32TargetRpm = fNewTarget;
        }
#endif
    }
#endif

    /*--- ADC 旋钮连续调速（电机运行时生效） ---*/
    if (s_u8MotorRunning)
    {
        uint16_t adc_val = ADC_GetFilteredValue();
        float fTargetRpm;

        if (adc_val >= ADC_RPM_MIN_VAL)
            fTargetRpm = ADC_RPM_MIN_SPEED;
        else if (adc_val <= ADC_RPM_MAX_VAL)
            fTargetRpm = ADC_RPM_MAX_SPEED;
        else
            fTargetRpm = ADC_RPM_MIN_SPEED + (ADC_RPM_MAX_SPEED - ADC_RPM_MIN_SPEED) *
                         (float)(ADC_RPM_MIN_VAL - adc_val) / (float)(ADC_RPM_MIN_VAL - ADC_RPM_MAX_VAL);

#if USE_VF_CTRL
        VF_SetTargetRpm(fTargetRpm);
#else
        g_stCtrl.f32TargetRpm = fTargetRpm;
#endif
    }

    /*--- LED1 (PB12) 以 1Hz 闪烁 ---*/
    {
        static uint32_t s_u32LedTick = 0;
        if (HAL_GetTick() - s_u32LedTick >= 500U)
        {
            s_u32LedTick = HAL_GetTick();
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }
    }

    /*--- VOFA FireWater telemetry, 500Hz ---*/
    {
        static uint32_t s_u32VofaTick = 0;
        if (HAL_GetTick() - s_u32VofaTick >= 2U)
        {
            s_u32VofaTick = HAL_GetTick();
            VOFA_SendTelemetry();
        }
    }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
