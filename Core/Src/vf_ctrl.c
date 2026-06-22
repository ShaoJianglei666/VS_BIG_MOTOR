/**
  ******************************************************************************
  * @file    vf_ctrl.c
  * @brief   V/F（电压/频率）开环启动控制 — 实现
  *
  *          控制流程（每 10kHz 中断调用 VF_ControlStep）:
  *            1. 阶段状态机（STOP → ALIGN → RAMPING → RUNNING）
  *            2. 频率斜坡更新（恒定加速度）
  *            3. 电角度积分（θ += ω_elec * Ts）
  *            4. V/f 曲线计算 Vq（含低频提升补偿）
  *            5. 反 Park 变换 → SVPWM → 更新 TIM1 CCR
  *
  *          独立 ADC 电流采样（不依赖 FOC 模块），用于监测保护及观测器。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "vf_ctrl.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* 引用 main.c 中校准的 ADC 零点（PWM 停止时 128 次平均） */
extern float FOC_fIaOffsetAdc;
extern float FOC_fIbOffsetAdc;

/*============================================================================*/
/* 全局变量                                                                    */
/*============================================================================*/

VF_ControlState g_stVFCtrl;

/*============================================================================*/
/* 私有辅助函数                                                                */
/*============================================================================*/

/**
  * @brief 浮点数限幅
  */
static inline float vf_clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
  * @brief 角度归一化到 [0, 2π)
  */
static inline float vf_wrap_2pi(float theta)
{
    while (theta >= VF_2PI) theta -= VF_2PI;
    while (theta < 0.0f)    theta += VF_2PI;
    return theta;
}

static inline float vf_wrap_pi(float theta)
{
    while (theta >= VF_MATH_PI) theta -= VF_2PI;
    while (theta < -VF_MATH_PI) theta += VF_2PI;
    return theta;
}

/**
  * @brief 正弦查找表 (256 点, float)
  *        查表求 sin/cos
  */
#define VF_SIN_TABLE_SIZE       256U
static const float VF_SinTable_F32[VF_SIN_TABLE_SIZE] =
{
     0.000000f,  0.024541f,  0.049068f,  0.073565f,  0.098017f,  0.122411f,  0.146730f,  0.170962f,
     0.195090f,  0.219101f,  0.242980f,  0.266713f,  0.290285f,  0.313682f,  0.336890f,  0.359895f,
     0.382683f,  0.405241f,  0.427555f,  0.449611f,  0.471397f,  0.492898f,  0.514103f,  0.534998f,
     0.555570f,  0.575808f,  0.595699f,  0.615232f,  0.634393f,  0.653173f,  0.671559f,  0.689541f,
     0.707107f,  0.724247f,  0.740951f,  0.757209f,  0.773010f,  0.788346f,  0.803208f,  0.817585f,
     0.831470f,  0.844854f,  0.857729f,  0.870087f,  0.881921f,  0.893224f,  0.903989f,  0.914210f,
     0.923880f,  0.932993f,  0.941544f,  0.949528f,  0.956940f,  0.963776f,  0.970031f,  0.975702f,
     0.980785f,  0.985278f,  0.989177f,  0.992480f,  0.995185f,  0.997290f,  0.998795f,  0.999699f,
     1.000000f,  0.999699f,  0.998795f,  0.997290f,  0.995185f,  0.992480f,  0.989177f,  0.985278f,
     0.980785f,  0.975702f,  0.970031f,  0.963776f,  0.956940f,  0.949528f,  0.941544f,  0.932993f,
     0.923880f,  0.914210f,  0.903989f,  0.893224f,  0.881921f,  0.870087f,  0.857729f,  0.844854f,
     0.831470f,  0.817585f,  0.803208f,  0.788346f,  0.773010f,  0.757209f,  0.740951f,  0.724247f,
     0.707107f,  0.689541f,  0.671559f,  0.653173f,  0.634393f,  0.615232f,  0.595699f,  0.575808f,
     0.555570f,  0.534998f,  0.514103f,  0.492898f,  0.471397f,  0.449611f,  0.427555f,  0.405241f,
     0.382683f,  0.359895f,  0.336890f,  0.313682f,  0.290285f,  0.266713f,  0.242980f,  0.219101f,
     0.195090f,  0.170962f,  0.146730f,  0.122411f,  0.098017f,  0.073565f,  0.049068f,  0.024541f,
     0.000000f, -0.024541f, -0.049068f, -0.073565f, -0.098017f, -0.122411f, -0.146730f, -0.170962f,
    -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f, -0.514103f, -0.534998f,
    -0.555570f, -0.575808f, -0.595699f, -0.615232f, -0.634393f, -0.653173f, -0.671559f, -0.689541f,
    -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f, -0.903989f, -0.914210f,
    -0.923880f, -0.932993f, -0.941544f, -0.949528f, -0.956940f, -0.963776f, -0.970031f, -0.975702f,
    -0.980785f, -0.985278f, -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f, -0.989177f, -0.985278f,
    -0.980785f, -0.975702f, -0.970031f, -0.963776f, -0.956940f, -0.949528f, -0.941544f, -0.932993f,
    -0.923880f, -0.914210f, -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f, -0.740951f, -0.724247f,
    -0.707107f, -0.689541f, -0.671559f, -0.653173f, -0.634393f, -0.615232f, -0.595699f, -0.575808f,
    -0.555570f, -0.534998f, -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f, -0.242980f, -0.219101f,
    -0.195090f, -0.170962f, -0.146730f, -0.122411f, -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

#define VF_ANGLE_TO_IDX_SCALE   40.743665f   /* 256 / 2π */

static inline void vf_sincos(float theta, float *psin, float *pcos)
{
    uint8_t idx = ((uint32_t)(theta * VF_ANGLE_TO_IDX_SCALE)) & 0xFFU;
    *psin = VF_SinTable_F32[idx];
    *pcos = VF_SinTable_F32[((uint16_t)idx + 64U) & 0xFFU];
}

/*============================================================================*/
/* Park 变换: Id=Ialpha*cos+Ibeta*sin, Iq=-Ialpha*sin+Ibeta*cos              */
/*============================================================================*/
static inline void vf_park(float fIalpha, float fIbeta, float fTheta,
                           float *pfId, float *pfIq)
{
    float fSin, fCos;
    vf_sincos(fTheta, &fSin, &fCos);
    *pfId =  fIalpha * fCos + fIbeta * fSin;
    *pfIq = -fIalpha * fSin + fIbeta * fCos;
}

/*============================================================================*/
/* 矢量限幅: sqrt(x²+y²) > limit → 等比例缩小                                 */
/*============================================================================*/
static inline void vf_limit_vector(float *px, float *py, float limit)
{
    float mag_sq = (*px * *px) + (*py * *py);
    float limit_sq = limit * limit;
    if (mag_sq > limit_sq)
    {
        float scale = limit / sqrtf(mag_sq);
        *px *= scale;
        *py *= scale;
    }
}

/*============================================================================*/
/* 增量式 PI 控制器（带抗饱和）                                                */
/*============================================================================*/
static float vf_pi_run(VF_PI *pst, float fRef, float fFb)
{
    float fErr = fRef - fFb;
    float fDelta = pst->fKp * (fErr - pst->fErrPrev);
    float fKiTerm;

    pst->fErrPrev = fErr;
    pst->fProportional = pst->fKp * fErr;

    fKiTerm = pst->fKi * fErr;

    if (!((pst->fOutPrev >= pst->fOutMax && fKiTerm > 0.0f) ||
          (pst->fOutPrev <= pst->fOutMin && fKiTerm < 0.0f)))
    {
        fDelta += fKiTerm;
        pst->fIntegral += fKiTerm;
        pst->fIntegral = vf_clamp(pst->fIntegral,
                                  pst->fOutMin * 0.5f,
                                  pst->fOutMax * 0.7f);
    }

    float fOut = pst->fOutPrev + fDelta;
    fOut = vf_clamp(fOut, pst->fOutMin, pst->fOutMax * 1.1f);
    pst->fOutPrev = fOut;
    return fOut;
}

/*============================================================================*/
/* SVPWM — 共模注入法，零序分量注入                                            */
/*============================================================================*/
void VF_Svpwm(float fValpha, float fVbeta,
              uint16_t *pu16Ta, uint16_t *pu16Tb, uint16_t *pu16Tc)
{
    float fVa, fVb, fVc;
    float fVmin, fVmax, fVoffset;
    float fArr = 8500.0f;  /* TIM1 ARR/2 */

    /* 反 Clarke: 从 αβ 到三相 */
    fVa = fValpha;
    fVb = -0.5f * fValpha + 0.8660254037844386f * fVbeta;
    fVc = -0.5f * fValpha - 0.8660254037844386f * fVbeta;

    /* 共模注入（零序注入） */
    fVmin = fVa;
    if (fVb < fVmin) fVmin = fVb;
    if (fVc < fVmin) fVmin = fVc;
    fVmax = fVa;
    if (fVb > fVmax) fVmax = fVb;
    if (fVc > fVmax) fVmax = fVc;
    fVoffset = -0.5f * (fVmin + fVmax);

    fVa += fVoffset;
    fVb += fVoffset;
    fVc += fVoffset;

    /* 缩放至 [0, ARR] */
    *pu16Ta = (uint16_t)vf_clamp((fVa + 1.0f) * 0.5f * fArr, 0.0f, fArr);
    *pu16Tb = (uint16_t)vf_clamp((fVb + 1.0f) * 0.5f * fArr, 0.0f, fArr);
    *pu16Tc = (uint16_t)vf_clamp((fVc + 1.0f) * 0.5f * fArr, 0.0f, fArr);
}

/*============================================================================*/
/* 反 Park 变换                                                                */
/*============================================================================*/
void VF_InvPark(float fVd, float fVq, float fTheta,
                float *pfValpha, float *pfVbeta)
{
    float fSin, fCos;
    vf_sincos(fTheta, &fSin, &fCos);
    *pfValpha = fVd * fCos - fVq * fSin;
    *pfVbeta  = fVd * fSin + fVq * fCos;
}

/*============================================================================*/
/* V/f 曲线计算                                                               */
/*============================================================================*/

/**
  * @brief 根据当前转速（机械 RPM）计算 V/f 曲线输出电压 (pu)
  *
  *        曲线形状:
  *          f < f_corner:  V = (f/f_rated) * VF_VF_RATIO + V_boost
  *          f ≥ f_corner:  V = V_max（进入限压区）
  *
  *        其中 f 为电频率 (Hz)，f_rated = PolePairs * 1000 / 60
  *
  * @param  fRpm  当前机械转速 (RPM)
  * @return 电压幅值 (pu)
  */
static float VF_GetVoltagePu(float fRpm)
{
    float fFreqHz;         /* 电频率 (Hz) */
    float fFreqPu;         /* 频率标幺值 */
    float fVpu;            /* 输出电压 (pu) */

    /* 计算电频率 */
    fFreqHz = fRpm * (float)MOTOR_POLE_PAIRS / 60.0f;

    /* 频率标幺值（以额定频率为基值） */
    fFreqPu = fFreqHz / VF_RATED_FREQ_HZ;

    if (fFreqPu <= 0.0f)
    {
        fVpu = 0.0f;
    }
    else if (fFreqPu < VF_CORNER_FREQ_PU)
    {
        /* V/f 线性区: V = ratio * freq + boost */
        fVpu = VF_VF_RATIO * fFreqHz / VF_BASE_VOLTAGE_V + VF_BOOST_VOLTAGE_PU;
    }
    else
    {
        /* 恒压区（弱磁区）: 限幅至最大电压 */
        fVpu = VF_MAX_VOLTAGE_PU;
    }

    /* 保存诊断值 */
    g_stVFCtrl.f32FreqHz     = fFreqHz;
    g_stVFCtrl.f32VfOutputPu = fVpu;

    return vf_clamp(fVpu, 0.0f, VF_MAX_VOLTAGE_PU);
}

/*============================================================================*/
/* 一阶 IIR 低通滤波器 — y[n] = y[n-1] + alpha * (x[n] - y[n-1])             */
/*============================================================================*/
static inline void vf_lpf_update(float *pfPrev, float fRaw, float fAlpha)
{
    *pfPrev += fAlpha * (fRaw - *pfPrev);
}

/*============================================================================*/
/* 电流采样 — ADC 数据读取 → LPF 滤波 → pu                                    */
/*============================================================================*/
void VF_GetPhaseCurrent(void)
{
    uint16_t u16IaRaw, u16IbRaw;
    float fIa, fIb;

    u16IaRaw = (uint16_t)(ADC1->JDR1);
    u16IbRaw = (uint16_t)(ADC2->JDR1);

    /* 原始值（使用 main.c 校准的 ADC 零点） */
    fIa = -1.0f * ((float)u16IaRaw - FOC_fIaOffsetAdc) / 2048.0f;
    fIb = -1.0f * ((float)u16IbRaw - FOC_fIbOffsetAdc) / 2048.0f;
    g_stVFCtrl.f32IaRaw = fIa;
    g_stVFCtrl.f32IbRaw = fIb;

    /* 一阶 IIR 低通滤波（封装函数） */
    vf_lpf_update(&g_stVFCtrl.f32Ia, fIa, VF_CURRENT_LPF_ALPHA);
    vf_lpf_update(&g_stVFCtrl.f32Ib, fIb, VF_CURRENT_LPF_ALPHA);

    fIa = g_stVFCtrl.f32Ia;
    fIb = g_stVFCtrl.f32Ib;

    /* Clarke 变换得到 αβ 电流 */
    g_stVFCtrl.f32Ialpha = fIa;
    g_stVFCtrl.f32Ibeta  = (fIa + 2.0f * fIb) * VF_INV_SQRT3;

    /* 电流矢量幅值近似: max(|Ialpha|, |Ibeta|) + 0.5*min(|Ialpha|, |IBeta|) */
    {
        float fa = fIa;
        float fb = (fIa + 2.0f * fIb) * VF_INV_SQRT3;
        if (fa < 0.0f) fa = -fa;
        if (fb < 0.0f) fb = -fb;
        if (fa < fb)
        {
            float ft = fa; fa = fb; fb = ft;
        }
        g_stVFCtrl.f32CurrentMag = fa + 0.5f * fb;
    }

}

/*============================================================================*/
/* 龙伯格反电势观测器 — 仅观测诊断，不参与控制                                */
/*============================================================================*/

void VF_LuenbergerInit(void)
{
    memset(&g_stVFCtrl.stObs, 0, sizeof(g_stVFCtrl.stObs));
    g_stVFCtrl.stObs.f32ThetaObs = g_stVFCtrl.f32Theta;
}

void VF_LuenbergerRun(void)
{
    VF_Luenberger *pst = &g_stVFCtrl.stObs;
    float fIalphaPred, fIbetaPred;
    float fErrAlpha, fErrBeta;
    float fEalphaRaw, fEbetaRaw;
    float fBemfSq, fBemfMag;
    float fSin, fCos;
    float fErrPll;
    float fOmegaOpen;
    float fSpeedBlend;

    if (g_stVFCtrl.eStage == VF_STAGE_STOP)
    {
        VF_LuenbergerInit();
        return;
    }

    /* 电流预测: Î(k+1) = Î(k) + (V - Rs·Î - Ê) · Ts / Ls */
    fIalphaPred = pst->f32IalphaHat
                + (g_stVFCtrl.f32Valpha
                   - VF_OBS_RS_PU * pst->f32IalphaHat
                   - pst->f32Ealpha) / VF_OBS_LS_OVER_TS_PU;
    fIbetaPred = pst->f32IbetaHat
               + (g_stVFCtrl.f32Vbeta
                  - VF_OBS_RS_PU * pst->f32IbetaHat
                  - pst->f32Ebeta) / VF_OBS_LS_OVER_TS_PU;

    /* 电流估计误差 */
    fErrAlpha = g_stVFCtrl.f32Ialpha - fIalphaPred;
    fErrBeta  = g_stVFCtrl.f32Ibeta  - fIbetaPred;

    /* 修正电流估计 */
    pst->f32IalphaHat = fIalphaPred + VF_OBS_CURRENT_GAIN * fErrAlpha;
    pst->f32IbetaHat  = fIbetaPred  + VF_OBS_CURRENT_GAIN * fErrBeta;

    /* 反电势更新（带 LPF） */
    fEalphaRaw = pst->f32Ealpha - VF_OBS_BEMF_GAIN * fErrAlpha;
    fEbetaRaw  = pst->f32Ebeta  - VF_OBS_BEMF_GAIN * fErrBeta;

    pst->f32Ealpha += VF_OBS_BEMF_LPF_GAIN * (fEalphaRaw - pst->f32Ealpha);
    pst->f32Ebeta  += VF_OBS_BEMF_LPF_GAIN * (fEbetaRaw  - pst->f32Ebeta);

    pst->f32Ealpha = vf_clamp(pst->f32Ealpha, -VF_MAX_VOLTAGE_PU, VF_MAX_VOLTAGE_PU);
    pst->f32Ebeta  = vf_clamp(pst->f32Ebeta,  -VF_MAX_VOLTAGE_PU, VF_MAX_VOLTAGE_PU);

    /* BEMF 幅值 */
    fBemfSq = pst->f32Ealpha * pst->f32Ealpha
            + pst->f32Ebeta  * pst->f32Ebeta;
    fBemfMag = sqrtf(fBemfSq);
    pst->f32BemfMag = fBemfMag;

    /* PLL 误差 */
    vf_sincos(pst->f32ThetaObs, &fSin, &fCos);
    fErrPll = (-pst->f32Ealpha * fCos - pst->f32Ebeta * fSin);
    if (fBemfMag > VF_OBS_MIN_BEMF_PU)
    {
        fErrPll /= fBemfMag;
    }
    fErrPll = vf_clamp(fErrPll, -VF_OBS_PLL_ERR_MAX, VF_OBS_PLL_ERR_MAX);
    pst->f32ErrPll = fErrPll;

    /* 开环角速度（用于速度辅助收敛） */
    fOmegaOpen = g_stVFCtrl.f32CurrentRpm
               * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
    fOmegaOpen = vf_clamp(fOmegaOpen, 0.0f, VF_OBS_MAX_OMEGA_ELEC);

    /* PLL 速度更新（低转速时 blend 开环速度辅助收敛） */
    fSpeedBlend = (pst->u16Locked != 0U) ? 0.0f : VF_OBS_PLL_SPEED_BLEND;
    pst->f32OmegaElec = (1.0f - fSpeedBlend) * pst->f32OmegaElec
                      + fSpeedBlend * fOmegaOpen
                      + VF_OBS_PLL_KI * VF_CTRL_TS * fErrPll;
    pst->f32OmegaElec = vf_clamp(pst->f32OmegaElec, 0.0f, VF_OBS_MAX_OMEGA_ELEC);

    /* PLL 角度更新 */
    pst->f32ThetaObs = vf_wrap_2pi(pst->f32ThetaObs
                         + pst->f32OmegaElec * VF_CTRL_TS
                         + VF_OBS_PLL_KP * VF_CTRL_TS * fErrPll);

    /* 观测速度 */
    pst->f32SpeedObs = pst->f32OmegaElec * VF_OBS_ELEC_OMEGA_TO_RPM;
    pst->f32ErrAlpha = fErrAlpha;
    pst->f32ErrBeta  = fErrBeta;

    /* 锁定判断 */
    pst->u16Locked = ((fBemfSq > VF_OBS_LOCK_BEMF_SQ_THR) &&
                      (pst->f32SpeedObs > VF_OBS_LOCK_SPEED_RPM)) ? 1U : 0U;
}

/*============================================================================*/
/* VF 控制初始化                                                              */
/*============================================================================*/
void VF_Init(void)
{
    memset(&g_stVFCtrl, 0, sizeof(g_stVFCtrl));

    g_stVFCtrl.eStage             = VF_STAGE_STOP;
    g_stVFCtrl.f32TargetRpm       = 0.0f;
    g_stVFCtrl.f32AccelRpmPerSec  = VF_ACCEL_RPM_PER_SEC;
    g_stVFCtrl.f32CurrentRpm      = 0.0f;
    g_stVFCtrl.f32Theta           = 0.0f;
    g_stVFCtrl.f32VdRef           = 0.0f;
    g_stVFCtrl.f32VqRef           = 0.0f;
    g_stVFCtrl.f32AlignVdRef      = 0.0f;
    g_stVFCtrl.u32SoftStartCount  = 0;
    g_stVFCtrl.u32VfRunCount      = 0;
    g_stVFCtrl.u32IfBlendCount    = 0;
    g_stVFCtrl.u32IfRunCount      = 0;
    g_stVFCtrl.u32ObsBlendCount   = 0;
    g_stVFCtrl.u32SpeedRunCount   = 0;
    g_stVFCtrl.f32IfITarget       = 0.0f;
    g_stVFCtrl.f32IdTarget        = 0.0f;
    g_stVFCtrl.f32IqTarget        = 0.0f;
    g_stVFCtrl.f32VdPiOut         = 0.0f;
    g_stVFCtrl.f32VqPiOut         = 0.0f;
    g_stVFCtrl.f32ThetaOpen       = 0.0f;
    g_stVFCtrl.f32SpeedPiOutMax   = 0.25f;
    VF_LuenbergerInit();
}

/*============================================================================*/
/* 设置目标转速                                                               */
/*============================================================================*/
void VF_SetTargetRpm(float fTargetRpm)
{
    /* 7阶段(OBS_RUNNING)后，最低转速强制为450RPM */
    if (g_stVFCtrl.eStage >= VF_STAGE_OBS_RUNNING)
    {
        if (fTargetRpm < 450.0f)
            fTargetRpm = 450.0f;
    }
    g_stVFCtrl.f32TargetRpm = fTargetRpm;

    /* 根据目标转速动态调整速度 PI 输出限幅 */
    if (fTargetRpm >= 800.0f)
        g_stVFCtrl.f32SpeedPiOutMax = 0.95f;
    else if (fTargetRpm >= 450.0f)
        g_stVFCtrl.f32SpeedPiOutMax = 0.85f;
    else
        g_stVFCtrl.f32SpeedPiOutMax = 0.50f;
    g_stVFCtrl.stPiSpeed.fOutMax = g_stVFCtrl.f32SpeedPiOutMax;

    /* 如果当前停止且目标 > 0，自动进入启动流程 */
    if ((g_stVFCtrl.eStage == VF_STAGE_STOP) && (fTargetRpm > 0.0f))
    {
        g_stVFCtrl.eStage        = VF_STAGE_ALIGN;
        g_stVFCtrl.f32CurrentRpm = 0.0f;
        g_stVFCtrl.f32Theta      = 0.0f;
        g_stVFCtrl.u32AlignCount = 0;
        g_stVFCtrl.u32SoftStartCount = 0;
        g_stVFCtrl.u32VfRunCount = 0;
        g_stVFCtrl.u32RunFrames  = 0;
    }
}

/*============================================================================*/
/* 设置加速度                                                                 */
/*============================================================================*/
void VF_SetAccel(float fAccelRpmPerSec)
{
    if (fAccelRpmPerSec > 0.0f)
    {
        g_stVFCtrl.f32AccelRpmPerSec = fAccelRpmPerSec;
    }
}

/*============================================================================*/
/* 停止                                                                       */
/*============================================================================*/
void VF_Stop(void)
{
    g_stVFCtrl.eStage       = VF_STAGE_STOP;
    g_stVFCtrl.f32TargetRpm = 0.0f;
    g_stVFCtrl.f32CurrentRpm = 0.0f;
    g_stVFCtrl.f32Theta     = 0.0f;

    /* 输出 50% 占空比（安全状态） */
    TIM1->CCR1 = PWM_HALF_CYCLE_VF;
    TIM1->CCR2 = PWM_HALF_CYCLE_VF;
    TIM1->CCR3 = PWM_HALF_CYCLE_VF;
}

/*============================================================================*/
/* VF 主控制步进 — 10kHz 中断中调用                                           */
/*============================================================================*/
void VF_ControlStep(void)
{
    float fSpeedStep;     /* 每帧转速增量 (RPM) */
    float fOmegaElec;     /* 电角速度 (rad/s) */
    float fStepAngle;     /* 每帧角度增量 (rad) */
    float fVqPu;          /* q 轴电压指令 (pu) */

    /*======================================================================*/
    /* 阶段状态机                                                            */
    /*======================================================================*/
    switch (g_stVFCtrl.eStage)
    {
    /*----------------------------------------------------------------------*/
    /* STOP: 输出 50% 占空比，等待启动指令                                     */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_STOP:
        TIM1->CCR1 = PWM_HALF_CYCLE_VF;
        TIM1->CCR2 = PWM_HALF_CYCLE_VF;
        TIM1->CCR3 = PWM_HALF_CYCLE_VF;
        return;
        /* break; — unreachable */

    /*----------------------------------------------------------------------*/
    /* ALIGN: 转子预对齐                                                     */
    /*        在 d 轴方向施加固定电压矢量（θ=0, Vd=小值, Vq=0），              */
    /*        产生静止磁场将转子拉至已知电角度 0 位置。                        */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_ALIGN:
    {
        g_stVFCtrl.u32AlignCount++;

        if (g_stVFCtrl.u32AlignCount <= VF_ALIGN_FRAMES)
        {
            /*--- 对齐: 固定方向 d 轴电压 ---*/
            g_stVFCtrl.f32Theta      = 0.0f;
            g_stVFCtrl.f32VdRef      = VF_ALIGN_VOLTAGE_PU;
            g_stVFCtrl.f32VqRef      = 0.0f;
            g_stVFCtrl.f32AlignVdRef = VF_ALIGN_VOLTAGE_PU;
        }
        else
        {
            /*--- 对齐完成 → 进入软过渡 ---*/
            g_stVFCtrl.eStage           = VF_STAGE_RAMPING;
            g_stVFCtrl.u32SoftStartCount = 0;
            g_stVFCtrl.f32CurrentRpm    = VF_START_RPM;
            g_stVFCtrl.f32Theta         = 0.0f;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* RAMPING: 频率斜坡加速                                                  */
    /*          含 ALIGN→RAMPING 软过渡                                     */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_RAMPING:
    {
        /*--- 计算每帧转速增量 ---*/
        fSpeedStep = g_stVFCtrl.f32AccelRpmPerSec * VF_CTRL_TS;

        /*--- 频率斜坡 ---*/
        g_stVFCtrl.f32CurrentRpm += fSpeedStep;

        if (g_stVFCtrl.f32CurrentRpm >= g_stVFCtrl.f32TargetRpm)
        {
            g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;
            g_stVFCtrl.eStage = VF_STAGE_RUNNING;
        }

        /*--- 电角度积分 ---*/
        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.f32Theta + fStepAngle);

        /*--- V/f 曲线计算 Vq ---*/
        fVqPu = VF_GetVoltagePu(g_stVFCtrl.f32CurrentRpm);
        g_stVFCtrl.f32VqRef = fVqPu;

        /*==================================================================*/
        /* ALIGN → RAMPING 软过渡                                           */
        /* 在软过渡期间，Vd 从对齐值线性衰减到 0，Vq 从 0 线性增长到目标值    */
        /* 避免电压矢量方向突变导致电机抖动                                   */
        /*==================================================================*/
        if (g_stVFCtrl.u32SoftStartCount < VF_SOFTSTART_FRAMES)
        {
            float fProgress = (float)g_stVFCtrl.u32SoftStartCount
                            / (float)VF_SOFTSTART_FRAMES;
            /* Vd 从对齐值线性衰减到 0 */
            g_stVFCtrl.f32VdRef = g_stVFCtrl.f32AlignVdRef * (1.0f - fProgress);
            /* Vq 从 0 线性增长到目标值 */
            g_stVFCtrl.f32VqRef = fVqPu * fProgress;
            g_stVFCtrl.u32SoftStartCount++;
        }
        else
        {
            /* 软过渡结束，纯 Vq 输出 */
            g_stVFCtrl.f32VdRef = 0.0f;
            g_stVFCtrl.f32VqRef = fVqPu;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* RUNNING: VF 恒速运行，计数 2s 后自动切到 IF                           */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_RUNNING:
    {
        g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;

        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.f32Theta + fStepAngle);

        /* V/f 曲线电压 */
        fVqPu = VF_GetVoltagePu(g_stVFCtrl.f32CurrentRpm);
        g_stVFCtrl.f32VdRef = 0.0f;
        g_stVFCtrl.f32VqRef = fVqPu;

        /* VF 运行 2s 后 → IF_BLEND */
        g_stVFCtrl.u32VfRunCount++;
        if (g_stVFCtrl.u32VfRunCount >= VF_IF_SWITCH_DELAY)
        {
            /* ★ 预载 dq PI：
             *    VF 模式下 Vd=0, Vq=fVqPu。
             *    PI_d 输出从 0 开始，PI_q 输出从 fVqPu 开始。
             *    目标电流用切换瞬间的实测 dq 电流，保证输出连续。
             */
            vf_park(g_stVFCtrl.f32Ialpha, g_stVFCtrl.f32Ibeta,
                    g_stVFCtrl.f32Theta,
                    &g_stVFCtrl.f32Id, &g_stVFCtrl.f32Iq);

            g_stVFCtrl.f32IdTarget = 0.0f;
            g_stVFCtrl.f32IqTarget = g_stVFCtrl.f32Iq;
            g_stVFCtrl.f32IfITarget = g_stVFCtrl.f32Iq;

            /* d 轴 PI：Vd=0 开始，限幅小范围 */
            g_stVFCtrl.stPiD.fKp       = VF_IF_PI_KP;
            g_stVFCtrl.stPiD.fKi       = VF_IF_PI_KI;
            g_stVFCtrl.stPiD.fErrPrev  = 0.0f;
            g_stVFCtrl.stPiD.fOutPrev  = 0.0f;
            g_stVFCtrl.stPiD.fIntegral = 0.0f;
            g_stVFCtrl.stPiD.fOutMax   = VF_IF_PI_CORRECTION_MAX;
            g_stVFCtrl.stPiD.fOutMin   = -VF_IF_PI_CORRECTION_MAX;

            /* q 轴 PI：Vq=fVqPu 开始（维持当前转矩），限幅大范围 */
            g_stVFCtrl.stPiQ.fKp       = VF_IF_PI_KP;
            g_stVFCtrl.stPiQ.fKi       = VF_IF_PI_KI;
            g_stVFCtrl.stPiQ.fErrPrev  = 0.0f;
            g_stVFCtrl.stPiQ.fOutPrev  = fVqPu * 1.10f;
            g_stVFCtrl.stPiQ.fIntegral = fVqPu * 1.10f;
            g_stVFCtrl.stPiQ.fOutMax   = VF_IF_PI_OUT_MAX;
            g_stVFCtrl.stPiQ.fOutMin   = VF_IF_PI_OUT_MIN;

            g_stVFCtrl.f32IfBlendVqStart = fVqPu;
            g_stVFCtrl.u32IfBlendCount   = 0;
            g_stVFCtrl.eStage = VF_STAGE_IF_BLEND;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* IF_BLEND: 诊断阶段 — 保持 V/f 电压不变，检查切换本身是否导致电流掉    */
    /*           如果这里电流不掉，说明是 PI 的问题；如果还掉，是切换问题   */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_IF_BLEND:
    {
        g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;

        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.f32Theta + fStepAngle);

        /* ★ 过流保护（暂时屏蔽以诊断切换问题） */
        {
            float fMagSq = g_stVFCtrl.f32Ialpha * g_stVFCtrl.f32Ialpha
                         + g_stVFCtrl.f32Ibeta * g_stVFCtrl.f32Ibeta;
            g_stVFCtrl.f32CurrentMag = sqrtf(fMagSq);
            /* if (fMagSq > (VF_IF_OC_LIMIT_PU * VF_IF_OC_LIMIT_PU))
            {
                g_stVFCtrl.eStage = VF_STAGE_FAULT;
                ...
            } */
        }

        /* 保持 V/f 电压不变（同 VF_RUNNING），诊断切换本身 */
        fVqPu = VF_GetVoltagePu(g_stVFCtrl.f32CurrentRpm);
        g_stVFCtrl.f32VdRef = 0.0f;
        g_stVFCtrl.f32VqRef = fVqPu;

        g_stVFCtrl.u32IfBlendCount++;
        if (g_stVFCtrl.u32IfBlendCount >= VF_IF_BLEND_FRAMES)
        {
            /* blend 结束后启用 dq PI */
            /* 注意：不做 Park 重新测量 Iq_target，保持 3→4 过渡时设定的值不变 */
            vf_park(g_stVFCtrl.f32Ialpha, g_stVFCtrl.f32Ibeta,
                    g_stVFCtrl.f32Theta,
                    &g_stVFCtrl.f32Id, &g_stVFCtrl.f32Iq);

            /* 不修改 f32IqTarget / f32IfITarget — 保持 RUNNING→IF_BLEND 时的设定值 */

            g_stVFCtrl.stPiD.fKp       = VF_IF_PI_KP;
            g_stVFCtrl.stPiD.fKi       = VF_IF_PI_KI;
            g_stVFCtrl.stPiD.fErrPrev  = 0.0f;
            g_stVFCtrl.stPiD.fOutPrev  = 0.0f;
            g_stVFCtrl.stPiD.fIntegral = 0.0f;
            g_stVFCtrl.stPiD.fOutMax   = VF_IF_PI_CORRECTION_MAX;
            g_stVFCtrl.stPiD.fOutMin   = -VF_IF_PI_CORRECTION_MAX;

            g_stVFCtrl.stPiQ.fKp       = VF_IF_PI_KP;
            g_stVFCtrl.stPiQ.fKi       = VF_IF_PI_KI;
            g_stVFCtrl.stPiQ.fErrPrev  = 0.0f;
            g_stVFCtrl.stPiQ.fOutPrev  = fVqPu;
            g_stVFCtrl.stPiQ.fIntegral = fVqPu;
            g_stVFCtrl.stPiQ.fOutMax   = VF_IF_PI_OUT_MAX;
            g_stVFCtrl.stPiQ.fOutMin   = VF_IF_PI_OUT_MIN;
            g_stVFCtrl.eStage = VF_STAGE_IF_RUNNING;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* IF_RUNNING: 开环角度 + dq 电流环 PI                                  */
    /*             保持 2s 后自动加速到 VF_IF_TARGET_RPM                     */
    /*             角度仍为开环积分，观测器仅旁路诊断。                      */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_IF_RUNNING:
    {
        g_stVFCtrl.u32IfRunCount++;
        g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;

        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.f32Theta + fStepAngle);

        /* 过流保护（暂屏蔽） */
        {
            float fMagSq = g_stVFCtrl.f32Ialpha * g_stVFCtrl.f32Ialpha
                         + g_stVFCtrl.f32Ibeta * g_stVFCtrl.f32Ibeta;
            g_stVFCtrl.f32CurrentMag = sqrtf(fMagSq);
        }

        /* Park 变换得到 dq 电流反馈 */
        vf_park(g_stVFCtrl.f32Ialpha, g_stVFCtrl.f32Ibeta,
                g_stVFCtrl.f32Theta,
                &g_stVFCtrl.f32Id, &g_stVFCtrl.f32Iq);

        /* d 轴 PI：Id_ref = 0 */
        g_stVFCtrl.f32VdPiOut = vf_pi_run(
            &g_stVFCtrl.stPiD,
            g_stVFCtrl.f32IdTarget,
            g_stVFCtrl.f32Id);

        /* q 轴 PI */
        g_stVFCtrl.f32VqPiOut = vf_pi_run(
            &g_stVFCtrl.stPiQ,
            g_stVFCtrl.f32IqTarget,
            g_stVFCtrl.f32Iq);

        g_stVFCtrl.f32VdRef = g_stVFCtrl.f32VdPiOut;
        g_stVFCtrl.f32VqRef = g_stVFCtrl.f32VqPiOut;

        /*--- 开环角度备份 ---*/
        g_stVFCtrl.f32ThetaOpen = vf_wrap_2pi(g_stVFCtrl.f32ThetaOpen + fStepAngle);

        /*--- 保持5s后切观测角 ---*/
        if ((g_stVFCtrl.u32IfRunCount >= VF_OBS_SWITCH_DELAY) &&
            (g_stVFCtrl.stObs.u16Locked != 0U))
        {
            g_stVFCtrl.f32ThetaErrSave =
                vf_wrap_pi(g_stVFCtrl.stObs.f32ThetaObs - g_stVFCtrl.f32Theta);
            g_stVFCtrl.f32ThetaOpenRef = g_stVFCtrl.f32Theta;
            g_stVFCtrl.u32ObsBlendCount = 0;
            g_stVFCtrl.eStage = VF_STAGE_OBS_TRANSITION;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* OBS_TRANSITION: 控制角从 IF 开环角度渐进逼近观测角                    */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_OBS_TRANSITION:
    {
        float fCnt;
        float fRemain;

        g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32TargetRpm;

        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        /* 开环角度继续积分（备份，用于诊断） */
        g_stVFCtrl.f32ThetaOpen = vf_wrap_2pi(g_stVFCtrl.f32ThetaOpen + fStepAngle);

        g_stVFCtrl.u32ObsBlendCount++;
        fCnt = (float)g_stVFCtrl.u32ObsBlendCount;
        if (fCnt > (float)VF_OBS_TRANSITION_FRAMES)
        {
            fCnt = (float)VF_OBS_TRANSITION_FRAMES;
        }

        /* 渐进逼近: θ = θ_obs - 残余误差 × (剩余帧数/总帧数) */
        fRemain = g_stVFCtrl.f32ThetaErrSave
                * ((float)VF_OBS_TRANSITION_FRAMES - fCnt)
                / (float)VF_OBS_TRANSITION_FRAMES;
        g_stVFCtrl.f32Theta = vf_wrap_2pi(g_stVFCtrl.stObs.f32ThetaObs - fRemain);

        /* 过流保护（暂屏蔽） */
        {
            float fMagSq = g_stVFCtrl.f32Ialpha * g_stVFCtrl.f32Ialpha
                         + g_stVFCtrl.f32Ibeta * g_stVFCtrl.f32Ibeta;
            g_stVFCtrl.f32CurrentMag = sqrtf(fMagSq);
        }

        /* Park 变换得到 dq 电流反馈 */
        vf_park(g_stVFCtrl.f32Ialpha, g_stVFCtrl.f32Ibeta,
                g_stVFCtrl.f32Theta,
                &g_stVFCtrl.f32Id, &g_stVFCtrl.f32Iq);

        /* d 轴 PI */
        g_stVFCtrl.f32VdPiOut = vf_pi_run(
            &g_stVFCtrl.stPiD,
            g_stVFCtrl.f32IdTarget,
            g_stVFCtrl.f32Id);

        /* q 轴 PI */
        g_stVFCtrl.f32VqPiOut = vf_pi_run(
            &g_stVFCtrl.stPiQ,
            g_stVFCtrl.f32IqTarget,
            g_stVFCtrl.f32Iq);

        g_stVFCtrl.f32VdRef = g_stVFCtrl.f32VdPiOut;
        g_stVFCtrl.f32VqRef = g_stVFCtrl.f32VqPiOut;

        if (g_stVFCtrl.u32ObsBlendCount >= VF_OBS_TRANSITION_FRAMES)
        {
            /* 过渡完成，完全使用观测器角度 */
            g_stVFCtrl.f32Theta = g_stVFCtrl.stObs.f32ThetaObs;

            /*--- 保存 Iq 前馈基准值，速度 PI 只输出修正量（Ki=0 也能维持基准） ---*/
            {
                float fObsSpd = g_stVFCtrl.stObs.f32SpeedObs;
                if (fObsSpd < 0.0f) fObsSpd = 0.0f;

                /* 用当前 Iq_target 作为前馈基准值，确保至少 VF_IF_IQ_TARGET_PU */
                float fPreload = g_stVFCtrl.f32IqTarget;
                if (fPreload < VF_IF_IQ_TARGET_PU)
                    fPreload = VF_IF_IQ_TARGET_PU;
                if (fPreload > 0.30f)
                    fPreload = 0.30f;

                g_stVFCtrl.f32IqBase   = fPreload;   /* ← 保存为前馈基准 */

                /* 速度 PI 从 0 开始，只输出修正量 */
                g_stVFCtrl.stPiSpeed.fKp       = VF_SPEED_PI_KP;
                g_stVFCtrl.stPiSpeed.fKi       = VF_SPEED_PI_KI;
                g_stVFCtrl.stPiSpeed.fOutPrev  = 0.0f;
                g_stVFCtrl.stPiSpeed.fIntegral = 0.0f;
                g_stVFCtrl.stPiSpeed.fOutMax   = g_stVFCtrl.f32SpeedPiOutMax;
                g_stVFCtrl.stPiSpeed.fOutMin   = VF_SPEED_PI_OUT_MIN;

                /* 同时预载 q 电流 PI（从 fPreload 开始），保持 Vq 连续 */
                g_stVFCtrl.stPiQ.fErrPrev = fPreload - g_stVFCtrl.f32Iq;
                g_stVFCtrl.stPiQ.fOutPrev = g_stVFCtrl.f32VqPiOut;
                g_stVFCtrl.stPiQ.fIntegral = g_stVFCtrl.f32VqPiOut;
                g_stVFCtrl.stPiQ.fProportional = VF_IF_PI_KP * g_stVFCtrl.stPiQ.fErrPrev;
            }

            /* 速度 ramp 从当前观测速度开始，最终目标由 f32TargetRpm（ADC 设定）决定 */
            g_stVFCtrl.f32SpeedTarget = g_stVFCtrl.stObs.f32SpeedObs;
            if (g_stVFCtrl.f32SpeedTarget < 50.0f)
                g_stVFCtrl.f32SpeedTarget = 50.0f;

            /* 速度 PI 从 0 开始，初始误差 = SpeedTarget - ObsSpeed ≈ 0 */
            {
                float fObsSpd2 = g_stVFCtrl.stObs.f32SpeedObs;
                if (fObsSpd2 < 0.0f) fObsSpd2 = 0.0f;
                g_stVFCtrl.stPiSpeed.fErrPrev  = g_stVFCtrl.f32SpeedTarget - fObsSpd2;
                g_stVFCtrl.stPiSpeed.fProportional = VF_SPEED_PI_KP * g_stVFCtrl.stPiSpeed.fErrPrev;
            }

            g_stVFCtrl.u32SpeedRunCount = 0;
            g_stVFCtrl.eStage = VF_STAGE_OBS_RUNNING;
        }
        break;
    }

    /*----------------------------------------------------------------------*/
    /* OBS_RUNNING: 观测器角度 + 速度环 + dq 电流环                         */
    /*             速度 ramp 到 f32TargetRpm（ADC 旋钮设定），              */
    /*             速度 PI 输出作为 Iq 目标                                  */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_OBS_RUNNING:
    {
        g_stVFCtrl.u32SpeedRunCount++;

        /* 速度 ramp：跟踪 f32TargetRpm（ADC 旋钮设定），支持加减速 */
        {
            float fStep = VF_SPEED_ACCEL_RPM_PER_SEC * VF_CTRL_TS;
            if (g_stVFCtrl.f32SpeedTarget < g_stVFCtrl.f32TargetRpm)
            {
                g_stVFCtrl.f32SpeedTarget += fStep;
                if (g_stVFCtrl.f32SpeedTarget > g_stVFCtrl.f32TargetRpm)
                    g_stVFCtrl.f32SpeedTarget = g_stVFCtrl.f32TargetRpm;
            }
            else if (g_stVFCtrl.f32SpeedTarget > g_stVFCtrl.f32TargetRpm)
            {
                g_stVFCtrl.f32SpeedTarget -= fStep;
                if (g_stVFCtrl.f32SpeedTarget < g_stVFCtrl.f32TargetRpm)
                    g_stVFCtrl.f32SpeedTarget = g_stVFCtrl.f32TargetRpm;
            }
        }

        g_stVFCtrl.f32CurrentRpm = g_stVFCtrl.f32SpeedTarget;

        fOmegaElec = g_stVFCtrl.f32CurrentRpm
                   * VF_2PI * (float)MOTOR_POLE_PAIRS / 60.0f;
        fStepAngle = fOmegaElec * VF_CTRL_TS;
        g_stVFCtrl.f32ThetaOpen = vf_wrap_2pi(g_stVFCtrl.f32ThetaOpen + fStepAngle);
        g_stVFCtrl.f32Theta = g_stVFCtrl.stObs.f32ThetaObs;

        {
            float fMagSq = g_stVFCtrl.f32Ialpha * g_stVFCtrl.f32Ialpha
                         + g_stVFCtrl.f32Ibeta * g_stVFCtrl.f32Ibeta;
            g_stVFCtrl.f32CurrentMag = sqrtf(fMagSq);
        }

        vf_park(g_stVFCtrl.f32Ialpha, g_stVFCtrl.f32Ibeta,
                g_stVFCtrl.f32Theta,
                &g_stVFCtrl.f32Id, &g_stVFCtrl.f32Iq);

        /* 速度 PI（降采样运行，避免 10kHz 过冲导致 Bang-Bang）
         * Iq 目标 = 前馈基准 + PI 修正量
         * 即使 Ki=0，基准值也不丢失 */
        if ((g_stVFCtrl.u32SpeedRunCount % VF_SPEED_DECIMATION) == 0U)
        {
            float fPiOut = vf_pi_run(
                &g_stVFCtrl.stPiSpeed,
                g_stVFCtrl.f32SpeedTarget,
                g_stVFCtrl.stObs.f32SpeedObs);
            g_stVFCtrl.f32IqTarget = g_stVFCtrl.f32IqBase + fPiOut;
            /* 限幅到安全范围 */
            if (g_stVFCtrl.f32IqTarget < 0.0f)
                g_stVFCtrl.f32IqTarget = 0.0f;
            if (g_stVFCtrl.f32IqTarget > g_stVFCtrl.f32SpeedPiOutMax)
                g_stVFCtrl.f32IqTarget = g_stVFCtrl.f32SpeedPiOutMax;
            g_stVFCtrl.f32IfITarget = g_stVFCtrl.f32IqTarget;
        }

        /* d 轴 PI：Id_ref = 0 */
        g_stVFCtrl.f32VdPiOut = vf_pi_run(
            &g_stVFCtrl.stPiD,
            g_stVFCtrl.f32IdTarget,
            g_stVFCtrl.f32Id);

        /* q 轴 PI：Iq_ref = 速度 PI 输出 */
        g_stVFCtrl.f32VqPiOut = vf_pi_run(
            &g_stVFCtrl.stPiQ,
            g_stVFCtrl.f32IqTarget,
            g_stVFCtrl.f32Iq);

        g_stVFCtrl.f32VdRef = g_stVFCtrl.f32VdPiOut;
        g_stVFCtrl.f32VqRef = g_stVFCtrl.f32VqPiOut;
        break;
    }

    /*----------------------------------------------------------------------*/
    /* FAULT: 故障保护                                                        */
    /*----------------------------------------------------------------------*/
    case VF_STAGE_FAULT:
        TIM1->CCR1 = PWM_HALF_CYCLE_VF;
        TIM1->CCR2 = PWM_HALF_CYCLE_VF;
        TIM1->CCR3 = PWM_HALF_CYCLE_VF;
        return;
        /* break; — unreachable */
    }

    /*======================================================================*/
    /* 电流限制 — 如果电流幅值超过阈值，按比例降低 Vq                        */
    /*======================================================================*/
    if (g_stVFCtrl.f32CurrentMag > VF_CURRENT_LIMIT_PU)
    {
        float fScale = VF_CURRENT_LIMIT_PU / g_stVFCtrl.f32CurrentMag;
        g_stVFCtrl.f32VqRef *= fScale;
    }

    /*======================================================================*/
    /* 反 Park 变换: dq → αβ                                               */
    /*======================================================================*/
    VF_InvPark(g_stVFCtrl.f32VdRef, g_stVFCtrl.f32VqRef,
               g_stVFCtrl.f32Theta,
               &g_stVFCtrl.f32Valpha, &g_stVFCtrl.f32Vbeta);

    /*======================================================================*/
    /* SVPWM: αβ → 三相占空比                                                */
    /*======================================================================*/
    VF_Svpwm(g_stVFCtrl.f32Valpha, g_stVFCtrl.f32Vbeta,
             &g_stVFCtrl.u16Ta, &g_stVFCtrl.u16Tb, &g_stVFCtrl.u16Tc);

    /*======================================================================*/
    /* 更新 TIM1 比较寄存器                                                  */
    /*======================================================================*/
    TIM1->CCR1 = g_stVFCtrl.u16Ta;
    TIM1->CCR2 = g_stVFCtrl.u16Tb;
    TIM1->CCR3 = g_stVFCtrl.u16Tc;

    /*======================================================================*/
    /* 龙伯格观测器（旁路诊断，不影响控制）                                  */
    /*======================================================================*/
    VF_LuenbergerRun();

    /*======================================================================*/
    /* 运行计数                                                              */
    /*======================================================================*/
    g_stVFCtrl.u32RunFrames++;
}
