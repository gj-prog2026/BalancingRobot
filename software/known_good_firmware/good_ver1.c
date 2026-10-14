/*----------------------------------
used t6, CONFIG = 1, SMPLRT_DIV = 0
------------------------------------*/

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
 
TIM_HandleTypeDef htim3;   // motor PWM
TIM_HandleTypeDef htim4;   // 100Hz control tick
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef  hi2c1;
 
volatile uint8_t control_tick = 0;
volatile uint8_t motors_enabled = 0;
volatile uint32_t dbg_ovr = 0;    // ticks that arrived before the last was handled
static uint32_t dbg_fail = 0;     // failed IMU reads
static uint32_t dbg_dtmax = 0;    // longest gap between control updates (ms)
volatile uint8_t ramp_test = 0;
 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_Init_All(void);
static void MX_TIM3_PWM_Init(void);
static void MX_TIM4_ControlTick_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Encoder_Init(void);
 
static void Motor_SetLeft(int16_t pwm);
static void Motor_SetRight(int16_t pwm);
static void STBY_Set(uint8_t on);
static void Motors_Enable(uint8_t on);
 
static uint8_t I2C_ReadReg(uint8_t reg, uint8_t *out);
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t val);
static uint8_t MPU6050_Init(void);
static uint8_t MPU6050_ReadRaw(int16_t *ax, int16_t *az, int16_t *gy);
static uint8_t MPU6050_ReadAll(int16_t a[3], int16_t g[3]);
static void Axis_Debug_Loop(void);
static void Gyro_Calibrate(void);
static void Update_Pitch(float dt);
 
typedef struct {
    float pitch_angle;
    float gyro_bias;
} imu_state_t;
 
static imu_state_t imu = {0};
static float dbg_acc = 0.0f, dbg_gyr = 0.0f; 
 
/* Measured while holding the bot UPRIGHT (balancing orientation), not
   lying flat — those are two different reference orientations for this
   sensor. Flat-table trim was 179.0; upright trim is -20.0. Re-measure
   if the sensor is remounted or the PCB orientation changes. */
#define PITCH_TRIM_DEG     -13.5f
#define SAFETY_CUTOFF_DEG  40.0f
 
static float Pitch_Relative(void)
{
    return imu.pitch_angle - PITCH_TRIM_DEG;
}
 
/* ---- PID ---- */
typedef struct { float kp, ki, kd, integral, prev_error; } pid_ctrl_t;
 
/* Starting-point gains, revised after first live test (tuningpid.txt):
   the original Ki=0.5 was causing integral windup — sustained moderate
   tilt let the integral term build up substantial extra correction,
   which then overshot once the error crossed back toward zero. Ki cut
   sharply; Kp raised slightly for a firmer immediate response so the
   system relies less on the integral term to begin with. */
static pid_ctrl_t angle_pid = { .kp = 3.0f, .ki = 0.0f, .kd = 0.0f };
 
/* Velocity/outer loop stubbed — fixed at 0 until encoders are wired in.
   angle_setpoint stays at 0 (i.e., exactly level per PITCH_TRIM_DEG),
   meaning the bot always tries to balance in place, no deliberate
   forward/backward drive yet. */
static float angle_setpoint = 0.0f;
 
static float PID_Update(pid_ctrl_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, 200.0f), -200.0f); // anti-windup
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
 
static void Safety_Check(void)
{
    if (fabsf(Pitch_Relative()) > SAFETY_CUTOFF_DEG) {
        Motors_Enable(0);
    }
}
 
#define MPU6050_ADDR       (0x68 << 1)
#define REG_PWR_MGMT_1     0x6B
#define REG_PWR_MGMT_2     0x6C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_CONFIG_2 0x1D
#define SLEEP_BIT          0x40
 
void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        if (control_tick) dbg_ovr++;
        control_tick = 1;
    }
}
 
/* ---- Tuning-session limits ---- */
#define KP_MAX             100.0f
#define KI_MAX              20.0f
#define KD_MAX              10.0f
#define SETPOINT_MAX_DEG    20.0f   // live trim range for 't'
#define ENABLE_WINDOW_DEG   10.0f   // refuse 'E' unless bot is this close to the setpoint

#define ENC_CPR   897.0f
#define WHEEL_R   0.0215f
#define TWO_PI_F  6.2831853f
#define DEG2RAD   0.0174533f
#define LOGN 300
static int16_t lg[LOGN][5];
static uint32_t lg_n = 0;
 
/* ---- Serial command handling ----
   Everything here is non-blocking: gain entry is a tiny state machine fed
   one byte at a time, so the 100Hz loop never stalls waiting on the keyboard
   (the old version could sit in a 3s wait with motors frozen at the last PWM). */
static char     cmd_letter = 0;     // 0 = idle, else p/i/k/t while collecting digits
static char     cmd_buf[16];
static uint8_t  cmd_len = 0;
static uint32_t cmd_start = 0;
 
static void Say(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void Dump_Log(void)
{
    if (motors_enabled) { Say("disable first\r\n"); return; }
    uint32_t n = lg_n < LOGN ? lg_n : LOGN;
    uint32_t start = lg_n - n;
    char b[48];
    Say("LOG rel*100,acc*100,gyro*10,enc_d,pwm\r\n");
    for (uint32_t k = 0; k < n; k++) {
        int16_t *r = lg[(start + k) % LOGN];
        int len = snprintf(b, sizeof b, "%d,%d,%d,%d,%d\r\n", r[0], r[1], r[2], r[3], r[4]);
        HAL_UART_Transmit(&huart2, (uint8_t *)b, len, 100);
        HAL_Delay(20);
    }
    Say("END\r\n");
}

static void Print_Status(void)
{
    char msg[96];
    int len = snprintf(msg, sizeof(msg),
        "rel=%.2f sp=%.2f Kp=%.2f Ki=%.2f Kd=%.2f en=%d\r\n",
        Pitch_Relative(), angle_setpoint,
        angle_pid.kp, angle_pid.ki, angle_pid.kd, motors_enabled);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100);
}

static float g_scale = 1.0f;
static float v_scale = 1.0f;
 
static void Apply_Value(char letter, const char *text)
{
    char *end;
    float val = strtof(text, &end);
 
    /* atof() returns 0 on garbage, which would silently zero a gain
       (e.g. a bare "p" + Enter -> Kp = 0 -> no control). Reject instead. */
    if (end == text || *end != '\0') {
        Say("bad number - ignored\r\n");
        return;
    }
 
    switch (letter) {
    case 'p':
        if (val < 0.0f || val > KP_MAX) { Say("Kp out of range - ignored\r\n"); return; }
        angle_pid.kp = val;
        break;
    case 'i':
        if (val < 0.0f || val > KI_MAX) { Say("Ki out of range - ignored\r\n"); return; }
        angle_pid.ki = val;
        angle_pid.integral = 0;      // otherwise ki*integral steps instantly
        break;
    case 'k':
        if (val < 0.0f || val > KD_MAX) { Say("Kd out of range - ignored\r\n"); return; }
        angle_pid.kd = val;
        break;
    case 't':
        if (fabsf(val) > SETPOINT_MAX_DEG) { Say("setpoint out of range - ignored\r\n"); return; }
        angle_setpoint = val;
        break;
    case 'g':
       if (val < 0.2f || val > 1.5f) { Say("g out of range - ignored\r\n"); return; }
       g_scale = val;
       break;
    case 'v':
       if (val < -1.5f || val > 1.5f) { Say("v out of range - ignored\r\n"); return; }
       v_scale = val;
       break;
    default:
        return;
    }
    Print_Status();
}
 
static void Cmd_Byte(uint8_t c)
{
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';   // case-insensitive
 
    /* Mid-way through a value entry such as "p20.0\n" */
    if (cmd_letter) {
        if (c == '\r' || c == '\n') {
            cmd_buf[cmd_len] = '\0';
            char letter = cmd_letter;
            cmd_letter = 0;
            Apply_Value(letter, cmd_buf);
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = (char)c;
        } else {
            cmd_letter = 0;
            Say("entry too long - ignored\r\n");
        }
        return;
    }
 
    switch (c) {
    case 'e':
        if (fabsf(Pitch_Relative() - angle_setpoint) > ENABLE_WINDOW_DEG) {
            Say("refused: hold the bot upright first\r\n");
        } else {
            angle_pid.integral = 0;
            angle_pid.prev_error = angle_setpoint - Pitch_Relative(); // no derivative kick
            Motors_Enable(1);
            Say("motors ENABLED\r\n");
        }
        break;
    case 'd':
        Motors_Enable(0);
        Say("motors DISABLED\r\n");
        break;
    case 's':
        Print_Status();
        break;
    case 'l': 
        Dump_Log();
        break;
    case 'r': 
        ramp_test = 1; 
        break;
    case 'p': case 'i': case 'k': case 't': case 'g': case 'v':
        cmd_letter = (char)c;
        cmd_len = 0;
        cmd_start = HAL_GetTick();
        break;
    default:
        break;   // ignore stray bytes / newlines
    }
}


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_All();
    MX_TIM3_PWM_Init();
    MX_TIM4_ControlTick_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Encoder_Init();

    STBY_Set(0);
    Motor_SetLeft(0);
    Motor_SetRight(0);

    if (!MPU6050_Init()) { Say("IMU init failed\r\n"); Error_Handler(); }
    Say("calibrating gyro - keep still\r\n");
    Gyro_Calibrate();

    int16_t ax, az, gy;
    if (MPU6050_ReadRaw(&ax, &az, &gy))
        imu.pitch_angle = atan2f((float)-az, (float)-ax) * 57.2958f;

    HAL_TIM_Base_Start_IT(&htim4);
    Say("LQR ready: e = enable, d = disable, t<deg> = trim\r\n");

    int16_t  enc_prev = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int32_t  enc_total = 0;
    uint8_t  was_enabled = 0;
    uint32_t last_print = 0;

    while (1)
    {
        uint8_t c;
        if (HAL_UART_Receive(&huart2, &c, 1, 0) == HAL_OK) Cmd_Byte(c);

        if (!control_tick) continue;
        control_tick = 0;
        static uint32_t last_t = 0;
        uint32_t t_now = HAL_GetTick();
        if (last_t && (t_now - last_t) > dbg_dtmax) dbg_dtmax = t_now - last_t;
        last_t = t_now;
        const float dt = 0.01f;

        Update_Pitch(dt);

        int16_t enc_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        int16_t d = (int16_t)(enc_now - enc_prev);   /* wrap-safe */
        enc_prev = enc_now;
        if (motors_enabled && !was_enabled) { enc_total = 0; lg_n = 0; }   /* x = 0 at enable */
        was_enabled = motors_enabled;
        enc_total += d;

        float rel  = Pitch_Relative() - angle_setpoint;      /* deg   */
        float gyro = dbg_gyr;                                /* deg/s */
        float phi  = enc_total * (TWO_PI_F / ENC_CPR);       /* rad   */
        float w    = d * (TWO_PI_F / ENC_CPR) / dt;          /* rad/s */
        float x    = WHEEL_R * (phi - rel  * DEG2RAD);
        float xd   = WHEEL_R * (w   - gyro * DEG2RAD);

        float out = g_scale * (70.39f*rel + 9.96f*gyro + v_scale*(-175.3f*x - 1975.4f*xd));
        int16_t pwm = (int16_t)fmaxf(fminf(out, 999.0f), -999.0f);
        if (!motors_enabled) pwm = 0;

        if (ramp_test && motors_enabled) {
            static uint16_t rt = 0;
            int tri = (rt < 150) ? 3 * rt : 3 * (300 - rt);   /* 0 -> 450 -> 0 */
            pwm = (int16_t)(-tri);                             /* negative = forward */
            if (++rt >= 300) { rt = 0; ramp_test = 0; }
        }

        if (motors_enabled) {
            int16_t *r = lg[lg_n % LOGN];
            r[0] = (int16_t)(rel * 100.0f);
            r[1] = (int16_t)((dbg_acc - angle_setpoint) * 100.0f);
            r[2] = (int16_t)(gyro * 10.0f);
            r[3] = d;
            r[4] = pwm;
            lg_n++;
        }
        Motor_SetLeft(pwm);
        Motor_SetRight(pwm);
        Safety_Check();

        if (HAL_GetTick() - last_print >= 100) {
            last_print = HAL_GetTick();
            char buf[96];
            int n = snprintf(buf, sizeof buf, "r=%d x=%d v=%d p=%d f=%lu o=%lu d=%lu\r\n",
                             (int)(rel*10.0f), (int)(x*100.0f), (int)(xd*100.0f), pwm,
                             (unsigned long)dbg_fail, (unsigned long)dbg_ovr, (unsigned long)dbg_dtmax);
            dbg_dtmax = 0;
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
        }
    }
}
 
static void GPIO_Init_All(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
 
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); // frees PB3/PB4 from JTAG
 
    GPIO_InitTypeDef g = {0};
 
    g.Pin = GPIO_PIN_13; // LED
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
 
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5; // AIN1, AIN2
    HAL_GPIO_Init(GPIOA, &g);
 
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; // STBY, BIN1, BIN2
    HAL_GPIO_Init(GPIOB, &g);
 
    g.Pin = GPIO_PIN_2; // USART2 TX
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
 
    g.Pin = GPIO_PIN_3; // USART2 RX
    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &g);
 
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7; // TIM3 PWM out
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
 
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7; // I2C1 SCL/SDA (open-drain)
    g.Mode = GPIO_MODE_AF_OD;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;   // left encoder A/B
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
}

static void MX_TIM1_Encoder_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM_Encoder_InitTypeDef e = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    e.EncoderMode = TIM_ENCODERMODE_TI12;
    e.IC1Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = 6;
    e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter = 6;
    if (HAL_TIM_Encoder_Init(&htim1, &e) != HAL_OK) Error_Handler();
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}
 
static void MX_TIM3_PWM_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
 
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);
 
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
 
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
 
static void MX_TIM4_ControlTick_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 7199;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 99;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}
 
static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
 
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}
 
static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
 
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 50000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}
 
static void Motor_SetLeft(int16_t pwm)
{
    if (pwm >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        pwm = -pwm;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}
 
static void Motor_SetRight(int16_t pwm)
{
    pwm = -pwm; // confirmed inversion fix
 
    if (pwm >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        pwm = -pwm;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}
 
static void STBY_Set(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
 
static void Motors_Enable(uint8_t on)
{
    motors_enabled = on;
    STBY_Set(on);
    if (!on) {
        Motor_SetLeft(0);
        Motor_SetRight(0);
    }
}
 
static uint8_t I2C_ReadReg(uint8_t reg, uint8_t *out)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, out, 1, 100) != HAL_OK) return 0;
    return 1;
}
 
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t cmd[2] = { reg, val };
    return HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100) == HAL_OK;
}
 
/* Verified retry-based wake sequence from Stage 1 bring-up (chip is an
   ICM-20689, not a genuine MPU6050 — needs reset before wake, and the
   wake write must be verified/retried, a fixed delay was unreliable). */
static uint8_t MPU6050_Init(void)
{
    I2C_WriteReg(REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
 
    uint8_t pwr1 = 0xFF;
    for (int attempt = 0; attempt < 20; attempt++) {
        I2C_WriteReg(REG_PWR_MGMT_1, 0x00);
        HAL_Delay(20);
        if (!I2C_ReadReg(REG_PWR_MGMT_1, &pwr1)) continue;
        if (!(pwr1 & SLEEP_BIT)) break;
    }
    if (pwr1 & SLEEP_BIT) return 0;
 
    I2C_WriteReg(REG_PWR_MGMT_2, 0x00);
    I2C_WriteReg(REG_GYRO_CONFIG, 0x18);
    I2C_WriteReg(REG_ACCEL_CONFIG, 0x00);
    I2C_WriteReg(REG_ACCEL_CONFIG_2, 0x04);
    I2C_WriteReg(REG_SMPLRT_DIV, 0);
    I2C_WriteReg(REG_CONFIG, 1);
    HAL_Delay(50);
 
    return 1;
}
 
static uint8_t MPU6050_ReadRaw(int16_t *ax, int16_t *az, int16_t *gy)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100) != HAL_OK) return 0;
 
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    return 1;
}
 
static void Gyro_Calibrate(void)
{
    int32_t sum = 0;
    int16_t ax, az, gy;
    for (int i = 0; i < 500; i++) {
        if (MPU6050_ReadRaw(&ax, &az, &gy)) {
            sum += gy;
        }
        HAL_Delay(2);
    }
    imu.gyro_bias = (float)sum / 500.0f;
}
 
static void Update_Pitch(float dt)
{
    int16_t ax, az, gy;
    if (!MPU6050_ReadRaw(&ax, &az, &gy)) { dbg_fail++; return; }
 
    float accel_angle = atan2f((float)-az, (float)-ax) * 57.2958f;
    float gyro_rate = ((float)gy - imu.gyro_bias) / 16.4f;
    dbg_acc = accel_angle - PITCH_TRIM_DEG;   // same scale as rel
    dbg_gyr = gyro_rate;
 
    imu.pitch_angle = 0.998f * (imu.pitch_angle + gyro_rate * dt)
                     + 0.002f * accel_angle;
}

static uint8_t MPU6050_ReadAll(int16_t a[3], int16_t g[3])
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100) != HAL_OK) return 0;
 
    a[0] = (int16_t)((buf[0]  << 8) | buf[1]);    /* accel X */
    a[1] = (int16_t)((buf[2]  << 8) | buf[3]);    /* accel Y */
    a[2] = (int16_t)((buf[4]  << 8) | buf[5]);    /* accel Z */
    /* buf[6], buf[7] = temperature */
    g[0] = (int16_t)((buf[8]  << 8) | buf[9]);    /* gyro X  */
    g[1] = (int16_t)((buf[10] << 8) | buf[11]);   /* gyro Y  */
    g[2] = (int16_t)((buf[12] << 8) | buf[13]);   /* gyro Z  */
    return 1;
}
 
/*static void Axis_Debug_Loop(void)   // never returns 
{
    int16_t a[3], g[3];
    float   bias[3] = {0.0f, 0.0f, 0.0f};
    char    msg[96];
 
    STBY_Set(0);                    // motors off while you move the bot by hand 
 
    // gyro bias: bot must be still for this ~1 s 
    for (int i = 0; i < 200; i++) {
        if (MPU6050_ReadAll(a, g)) {
            for (int k = 0; k < 3; k++) bias[k] += (float)g[k];
        }
        HAL_Delay(5);
    }
    for (int k = 0; k < 3; k++) bias[k] /= 200.0f;
 
    while (1) {
        float    asum[3]  = {0.0f, 0.0f, 0.0f};
        float    gpeak[3] = {0.0f, 0.0f, 0.0f};
        int      n = 0;
        uint32_t t0 = HAL_GetTick();
 
        while (HAL_GetTick() - t0 < 250) {
            if (MPU6050_ReadAll(a, g)) {
                for (int k = 0; k < 3; k++) {
                    asum[k] += (float)a[k];
                    float r = ((float)g[k] - bias[k]) / 16.4f;
                    if (fabsf(r) > fabsf(gpeak[k])) gpeak[k] = r;
                }
                n++;
            }
            HAL_Delay(5);
        }
 
        int len;
        if (n == 0) {
            len = snprintf(msg, sizeof(msg), "I2C read failed\r\n");
        } else {
            len = snprintf(msg, sizeof(msg),
                "a[g] x=%+.2f y=%+.2f z=%+.2f | gpk[dps] x=%+.0f y=%+.0f z=%+.0f\r\n",
                asum[0] / n / 16384.0f, asum[1] / n / 16384.0f, asum[2] / n / 16384.0f,
                gpeak[0], gpeak[1], gpeak[2]);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100);
    }
}*/
 
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
 
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
 
static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}