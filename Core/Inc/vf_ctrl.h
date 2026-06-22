/**
  ******************************************************************************
  * @file    vf_ctrl.h
  * @brief   V/F（电压/频率）开环启动控制 — 独立模块
  *          不依赖电流 PI 闭环，通过 V/f 比直接输出电压矢量。
  *          适用于 PMSM/BLDC 电机开环起动至可观测反电势的转速。
  ******************************************************************************
  * @attention
  *   核心原理:
  *     1. 频率斜坡: 目标转速以恒定加速度从 0 升至设定值
  *     2. 角度生成: 电角度由频率积分得到（θ = ∫ ω·dt）
  *     3. V/f 曲线: Vq 随频率线性增长（含低频提升补偿定子电阻压降）
  *     4. 输出: Vd=0, Vq 由 V/f 曲线给出 → 反Park → SVPWM
  *
  *   电机参数来源于 Motor_Param.h（与 FOC 共用）
  ******************************************************************************
  */

#ifndef __VF_CTRL_H
#define __VF_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "Motor_Param.h"

/* Exported defines ---------------------------------------------------------*/

/**
  * @brief  USE_VF_CTRL — 控制模式全局选择
  *         = 1: 使用 V/F 开环启动（电压/频率比控制，无电流环）
  *         = 0: 使用原有 IF 开环（电流闭环 + 角度积分）
  *
  * 此宏供 stm32g4xx_it.c 和 main.c 共享。
  * 切换模式后请重新编译整个工程。
  */
#define USE_VF_CTRL                     1

/* Exported constants --------------------------------------------------------*/

/** @brief 控制频率 (Hz) — 与 FOC 共用 10kHz */
#define VF_CTRL_FREQ                    10000U
#define VF_CTRL_TS                      (1.0f / (float)VF_CTRL_FREQ)

/** @brief 圆周率 */
#define VF_MATH_PI                      3.14159265358979323846f
#define VF_2PI                          (2.0f * VF_MATH_PI)
#define VF_SQRT3                        1.7320508075688772f
#define VF_INV_SQRT3                    0.5773502691896257f

/** @brief 电压基值 (V) = Vbus / √3 */
#define VF_BASE_VOLTAGE_V               (MOTOR_BUS_VOLTAGE / VF_SQRT3)

/** @brief 电流基值 (A)，1.0 pu = 94.3A
  *        根据硬件计算：3.3V / (0.002Ω × 17.5倍) = 94.3A */
#define VF_BASE_CURRENT_A               94.3f

/**
  * @brief V/f 比率 (V/Hz)
  *        按反电势系数计算额定 V/f：
  *        Ke = MOTOR_BEMF_CONST_V_LL V_peak L-L / krpm（机械）
  *        每相电压峰值 at 1000RPM = Ke / √3
  *        电频率 at 1000RPM = 1000 * PolePairs / 60
  *        V/f_ratio = 相电压峰值 / 电频率
  *
  *        ★ 注意：BEMF V/f 仅抵消反电势。要产生电流还需额外电压克服 IR 压降。
  *          因此实际 V/f 应略高于 BEMF V/f，这里取 105% 留足转矩余量。
  */
#define VF_RATED_FREQ_HZ                (1000.0f * (float)MOTOR_POLE_PAIRS / 60.0f)  /* 1000RPM 对应电频率 */
#define VF_NOMINAL_RATIO                ((MOTOR_BEMF_CONST_V_LL / VF_SQRT3) / VF_RATED_FREQ_HZ)
#define VF_VF_RATIO                     (VF_NOMINAL_RATIO * 1.05f)  /* 105% BEMF V/f，留足 IR 压降余量 */

/**
  * @brief 低频电压提升（IR 补偿）
  *        在低频时定子电阻压降占主导，需额外提升电压。
  *        此电机 R=0.006Ω、L=35µH，极低阻抗导致低频电流对电压极敏感。
  *        提升电压帮助克服静摩擦和齿槽转矩。
  *        取 30% 基值电压 ≈ 5.2V，约对应 5.2/0.006 ≈ 867A 峰值 — 实际受
  *        电源限流和 MOSFET 导通电阻限制，不会达到此值。
  *        如仍顿挫可逐步增大此值。
  */
#define VF_BOOST_VOLTAGE_PU             0.30f

/** @brief 最大输出电压 (pu)，限制 SVPWM 不过调制 */
#define VF_MAX_VOLTAGE_PU               0.92f

/**
  * @brief V/f 拐点频率 (pu of rated freq)
  *        低于此频率 V/f 为线性，高于此频率电压保持恒定（弱磁区）
  *        默认 1.0 = 额定频率
  */
#define VF_CORNER_FREQ_PU               1.0f

/*--- 频率斜坡参数 ---*/

/** @brief 加速度 (RPM/s) — 降低加速度防止失步 */
#define VF_ACCEL_RPM_PER_SEC            60.0f

/** @brief 每帧转速增量 (RPM) = Accel * Ts */
#define VF_SPEED_STEP_RPM               (VF_ACCEL_RPM_PER_SEC * VF_CTRL_TS)

/** @brief 起始频率 (RPM) — 从该转速开始施加电压
  *        提高起始转速，让电机更快跳过齿槽转矩影响严重的极低速区
  */
#define VF_START_RPM                    80.0f

/** @brief 启动前对齐时间 (帧) — 施加直流电压对齐转子 */
#define VF_ALIGN_FRAMES                 3000U   /* 300ms @ 10kHz */

/** @brief 对齐电压 (pu) — 对齐阶段 d 轴电压幅值 */
#define VF_ALIGN_VOLTAGE_PU             0.15f

/** @brief ALIGN→RAMPING 软过渡帧数 — 从 Vd 对齐平滑切到 Vq 旋转 */
#define VF_SOFTSTART_FRAMES             3000U   /* 300ms @ 10kHz */

/** @brief 电流限制 (pu) — 超过此值则降低 Vq 防止过流
  *        基于 ADC 采样的相电流幅值做简单限幅
  *        0.53pu × 94.3A ≈ 50A */
#define VF_CURRENT_LIMIT_PU             0.12f   /* 50A@94.3A基值 */

/** @brief VF→IF 切换延时 (帧) — VF_RUNNING 后等待 2s 再切 */
#define VF_IF_SWITCH_DELAY              5000U  /* 2s @ 10kHz */

/** @brief VF→IF blend 过渡帧数 */
#define VF_IF_BLEND_FRAMES              10000U  /* 1s @ 10kHz */

/** @brief IF 电流目标 (pu) — q 轴电流 */
#define VF_IF_IQ_TARGET_PU              0.03f   /* 1.5A */

/** @brief IF_RUNNING 保持时间 (帧) — 电流环稳定后再加速 */
#define VF_IF_HOLD_FRAMES               20000U  /* 2s @ 10kHz */

/** @brief IF 开环稳定后切观测角的等待帧数 */
#define VF_OBS_SWITCH_DELAY             12500U  /* 1.25s @ 10kHz */

/** @brief 开环角度到观测角度渐进切换帧数 */
#define VF_OBS_TRANSITION_FRAMES        5000U   /* 500ms @ 10kHz */

/** @brief 速度环 PI 增益（降采样到 100Hz 运行） */
#define VF_SPEED_PI_KP                  0.004f
#define VF_SPEED_PI_KI                  0.000f

/** @brief 速度环 PI 输出限幅 (pu Iq) */
#define VF_SPEED_PI_OUT_MAX             0.05f   /* 4.7A@94.3A基值 */
#define VF_SPEED_PI_OUT_MIN             0.0f

/** @brief 速度环降采样率：每 N 帧执行一次速度 PI (10kHz/100=100Hz) */
#define VF_SPEED_DECIMATION             100U

/** @brief 速度环目标转速和加速度 */
#define VF_SPEED_TARGET_RPM             300.0f
#define VF_SPEED_ACCEL_RPM_PER_SEC      100.0f

/** @brief IF 加速度 (RPM/s) */
#define VF_IF_ACCEL_RPM_PER_SEC         50.0f

/** @brief IF 加速目标转速 (RPM) */
#define VF_IF_TARGET_RPM                300.0f

/** @brief IF dq 电流环 PI 增益 */
#define VF_IF_PI_KP                     0.02f
#define VF_IF_PI_KI                     0.0005f

/** @brief IF dq 电流环 PI 修正限幅 (pu) */
#define VF_IF_PI_CORRECTION_MAX         0.15f

/** @brief IF dq 电流环 Vq PI 输出限幅 (pu) */
#define VF_IF_PI_OUT_MAX                0.85f
#define VF_IF_PI_OUT_MIN                (-VF_IF_PI_OUT_MAX)

/** @brief IF 过流保护阈值 (pu) — 暂提高避免切换毛刺误触发 */
#define VF_IF_OC_LIMIT_PU               0.30f   /* 15A */

/*--- Luenberger BEMF observer / PLL parameters（仅观测，不参与控制）---*/
#define VF_OBS_MAX_SPEED_RPM            7400.0f
#define VF_OBS_MAX_OMEGA_ELEC           (VF_OBS_MAX_SPEED_RPM * VF_2PI \
                                         * (float)MOTOR_POLE_PAIRS / 60.0f)
#define VF_OBS_ELEC_OMEGA_TO_RPM        (60.0f / (VF_2PI * (float)MOTOR_POLE_PAIRS))
#define VF_OBS_RS_PU                    (MOTOR_PHASE_RESISTANCE * VF_BASE_CURRENT_A \
                                         / VF_BASE_VOLTAGE_V)
#define VF_OBS_LS_OVER_TS_PU            (MOTOR_PHASE_INDUCTANCE * VF_BASE_CURRENT_A \
                                         / (VF_BASE_VOLTAGE_V * VF_CTRL_TS))
#define VF_OBS_CURRENT_GAIN             0.38f
#define VF_OBS_BEMF_GAIN                0.002f
#define VF_OBS_BEMF_LPF_GAIN            0.25f
#define VF_OBS_PLL_KP                   20.0f
#define VF_OBS_PLL_KI                   300.0f
#define VF_OBS_PLL_SPEED_BLEND          0.002f
#define VF_OBS_PLL_ERR_MAX              0.25f
#define VF_OBS_MIN_BEMF_PU              0.01f
#define VF_OBS_LOCK_BEMF_SQ_THR         0.00005f
#define VF_OBS_LOCK_SPEED_RPM           150.0f

/*--- 电流采样一阶 IIR 低通滤波参数 ---*/

/**
  * @brief 一阶 IIR 低通滤波器系数
  *        y[n] = y[n-1] + alpha * (x[n] - y[n-1])
  *        alpha = 1 - exp(-2*pi*fc/fs)
  *        fs = 10kHz, fc ≈ 300Hz → alpha ≈ 0.17
  *        增大 alpha → 响应更快但滤波效果减弱
  *        减小 alpha → 滤波更强但响应变慢
  */
#define VF_CURRENT_LPF_ALPHA            0.17f

/** @brief 对齐占空比 (50%=中点) */
#define PWM_HALF_CYCLE_VF               (17000U / 4U)

/* Exported types ------------------------------------------------------------*/

/**
  * @brief VF 控制阶段
  */
typedef enum
{
    VF_STAGE_STOP      = 0,  /**< 停止 */
    VF_STAGE_ALIGN     = 1,  /**< 转子预对齐 */
    VF_STAGE_RAMPING   = 2,  /**< 频率斜坡加速 */
    VF_STAGE_RUNNING   = 3,  /**< VF 恒速运行 */
    VF_STAGE_IF_BLEND  = 4,  /**< VF→IF 过渡 blend */
    VF_STAGE_IF_RUNNING= 5,  /**< IF 电流环闭环运行 */
    VF_STAGE_OBS_TRANSITION = 6, /**< 开环角度渐进切换到观测角度 */
    VF_STAGE_OBS_RUNNING = 7, /**< 观测角度 dq 电流环运行 */
    VF_STAGE_FAULT     = 8   /**< 故障 */
} VF_Stage;

/**
  * @brief PI 控制器（增量式）
  */
typedef struct
{
    float fKp;
    float fKi;
    float fErrPrev;
    float fOutPrev;
    float fIntegral;
    float fProportional;
    float fOutMax;
    float fOutMin;
} VF_PI;

/**
  * @brief Luenberger 反电势观测器状态
  */
typedef struct
{
    float    f32IalphaHat;      /**< α 轴电流估计 (pu) */
    float    f32IbetaHat;       /**< β 轴电流估计 (pu) */
    float    f32Ealpha;         /**< α 轴反电势估计 (pu) */
    float    f32Ebeta;          /**< β 轴反电势估计 (pu) */
    float    f32ThetaObs;       /**< 观测电角度 (rad) */
    float    f32SpeedObs;       /**< 观测机械转速 (RPM) */
    float    f32OmegaElec;      /**< 观测电角速度 (rad/s) */
    float    f32ErrAlpha;       /**< α 轴电流估计误差 (pu) */
    float    f32ErrBeta;        /**< β 轴电流估计误差 (pu) */
    float    f32ErrPll;         /**< PLL 相位误差 */
    float    f32BemfMag;        /**< BEMF 幅值 (pu) */
    uint16_t u16Locked;         /**< 锁定标志 */
} VF_Luenberger;

/**
  * @brief VF 控制状态结构体
  */
typedef struct
{
    /*--- 用户设置 ---*/
    float     f32TargetRpm;     /**< 目标转速 (RPM) */
    float     f32AccelRpmPerSec;/**< 加速度 (RPM/s) */

    /*--- 运行状态 ---*/
    VF_Stage  eStage;           /**< 当前阶段 */
    float     f32CurrentRpm;    /**< 当前频率对应转速 (RPM) */
    float     f32Theta;         /**< 当前电角度 (rad) [0, 2π) */
    float     f32VdRef;         /**< d 轴电压指令 (pu) */
    float     f32VqRef;         /**< q 轴电压指令 (pu) */
    float     f32Valpha;        /**< α 轴电压 (pu) */
    float     f32Vbeta;         /**< β 轴电压 (pu) */
    uint16_t  u16Ta;            /**< A 相 PWM 比较值 */
    uint16_t  u16Tb;            /**< B 相 PWM 比较值 */
    uint16_t  u16Tc;            /**< C 相 PWM 比较值 */
    uint32_t  u32AlignCount;    /**< 对齐计数器 */
    uint32_t  u32RunFrames;     /**< 总运行帧数 */

    /*--- 软启动状态 ---*/
    uint32_t  u32SoftStartCount;/**< 软过渡计数器 */
    float     f32AlignVdRef;    /**< 对齐时的 Vd 值，软过渡用 */

    /*--- VF→IF 切换状态 ---*/
    uint32_t  u32VfRunCount;    /**< VF_RUNNING 运行帧数 */
    uint32_t  u32IfBlendCount;  /**< IF blend 计数器 */
    uint32_t  u32IfRunCount;    /**< IF_RUNNING 运行帧数 */
    uint32_t  u32ObsBlendCount; /**< 观测角切换计数器 */
    float     f32IfBlendVqStart;/**< blend 起始 Vq (V/f 值) */
    float     f32ThetaErrSave;  /**< 过渡开始时 θobs - θopen 的带符号误差 */
    float     f32ThetaOpenRef;  /**< 过渡开始时开环角度 */
    float     f32ThetaOpen;     /**< 纯开环积分角度备份 */

    /*--- IF dq 电流环状态 ---*/
    float     f32IfITarget;     /**< 电流幅值目标 (pu)，供遥测 */
    float     f32IdTarget;      /**< d 轴电流目标 (pu) */
    float     f32IqTarget;      /**< q 轴电流目标 (pu) */
    VF_PI     stPiD;            /**< d 轴电流 PI */
    VF_PI     stPiQ;            /**< q 轴电流 PI */
    float     f32VdPiOut;       /**< d 轴 PI 输出 (pu) */
    float     f32VqPiOut;       /**< q 轴 PI 输出 (pu) */

    /*--- 速度环 ---*/
    VF_PI     stPiSpeed;        /**< 速度 PI（观测器角度闭环后启用） */
    float     f32SpeedTarget;   /**< 速度环 ramp 目标 (RPM) */
    uint32_t  u32SpeedRunCount; /**< 速度环运行计数器 */
    float     f32SpeedPiOutMax; /**< 速度 PI 输出限幅，随目标转速动态变化 */
    float     f32IqBase;        /**< Iq 前馈基准值 (pu)，速度 PI 在此之上做修正 */

    /*--- dq 电流反馈 ---*/
    float     f32Id;            /**< d 轴电流反馈 (pu) */
    float     f32Iq;            /**< q 轴电流反馈 (pu) */

    /*--- 电流监控 ---*/
    float     f32Ia;            /**< A 相电流 (pu) — 一阶 LPF 滤波后 */
    float     f32Ib;            /**< B 相电流 (pu) — 一阶 LPF 滤波后 */
    float     f32IaRaw;         /**< A 相电流原始采样值 (pu)，仅诊断用 */
    float     f32IbRaw;         /**< B 相电流原始采样值 (pu)，仅诊断用 */
    float     f32Ialpha;        /**< α 轴电流 (pu) — 基于滤波后的值 */
    float     f32Ibeta;         /**< β 轴电流 (pu) — 基于滤波后的值 */
    float     f32CurrentMag;    /**< 电流空间矢量幅值 (pu) */

    /*--- 诊断 ---*/
    float     f32VfOutputPu;    /**< V/f 曲线输出电压 (pu) */
    float     f32FreqHz;        /**< 当前电频率 (Hz) */

    VF_Luenberger stObs;        /**< Luenberger 反电势观测器 */
} VF_ControlState;

/* Exported variables --------------------------------------------------------*/

extern VF_ControlState g_stVFCtrl;

/* Exported function prototypes ----------------------------------------------*/

void  VF_Init(void);
void  VF_SetTargetRpm(float fTargetRpm);
void  VF_SetTargetRpm(float fTargetRpm);
void  VF_SetAccel(float fAccelRpmPerSec);
void  VF_Stop(void);
void  VF_ControlStep(void);
void  VF_Svpwm(float fValpha, float fVbeta,
               uint16_t *pu16Ta, uint16_t *pu16Tb, uint16_t *pu16Tc);
void  VF_InvPark(float fVd, float fVq, float fTheta,
                 float *pfValpha, float *pfVbeta);
void  VF_GetPhaseCurrent(void);
void  VF_LuenbergerInit(void);
void  VF_LuenbergerRun(void);

#ifdef __cplusplus
}
#endif

#endif /* __VF_CTRL_H */
