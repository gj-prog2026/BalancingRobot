/* =====================================================================
   STAGE 4 — Angle PID. The bot actively tries to balance for the first
   time. Velocity/outer loop is STUBBED (fixed setpoint) on purpose —
   get the inner loop working and tuned before adding the outer loop,
   so a balancing failure isn't tangled up with an untuned outer loop.
 
   UART commands:
     E = enable motors (arms the PID output)
     D = disable motors (immediate stop, safe default)
     S = print current pitch + gains
   ===================================================================== */
 
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
 
TIM_HandleTypeDef htim3;   // motor PWM
TIM_HandleTypeDef htim4;   // 100Hz control tick
UART_HandleTypeDef huart2;
I2C_HandleTypeDef  hi2c1;
 
volatile uint8_t control_tick = 0;
volatile uint8_t motors_enabled = 0;
 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_Init_All(void);
static void MX_TIM3_PWM_Init(void);
static void MX_TIM4_ControlTick_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
 
static void Motor_SetLeft(int16_t pwm);
static void Motor_SetRight(int16_t pwm);
static void STBY_Set(uint8_t on);
static void Motors_Enable(uint8_t on);
 
static uint8_t I2C_ReadReg(uint8_t reg, uint8_t *out);
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t val);
static uint8_t MPU6050_Init(void);
static uint8_t MPU6050_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx);
static void Gyro_Calibrate(void);
static void Update_Pitch(float dt);
 
typedef struct {
    float pitch_angle;
    float gyro_bias;
    float gyro_rate_dps;
} imu_state_t;
 
static imu_state_t imu = {0};
 
/* TEMP DEBUG: raw axis readings, for empirically determining how the
   MPU6050's axes map onto the chassis (dead-bug mount orientation).
   Safe to remove once the axis mapping is confirmed. */
static int16_t g_raw_ax = 0, g_raw_ay = 0, g_raw_az = 0;
static int16_t g_last_pwm = 0; // TEMP DEBUG: last commanded PID output/PWM
 
/* Measured while holding the bot UPRIGHT (balancing orientation), not
   lying flat — those are two different reference orientations for this
   sensor. Flat-table trim was 179.0; upright trim is -20.0. Re-measure
   if the sensor is remounted or the PCB orientation changes. */
#define PITCH_TRIM_DEG     -20.0f
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
static pid_ctrl_t angle_pid = { .kp = 20.0f, .ki = 0.1f, .kd = 0.9f };
 
/* Velocity/outer loop stubbed — fixed at 0 until encoders are wired in.
   angle_setpoint stays at 0 (i.e., exactly level per PITCH_TRIM_DEG),
   meaning the bot always tries to balance in place, no deliberate
   forward/backward drive yet. */
static float angle_setpoint = -60.38f;//changed this value based on rel_pitch reading min and max
 
static float PID_Update(pid_ctrl_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, 200.0f), -200.0f); // anti-windup
    float derivative = -imu.gyro_rate_dps;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
 
static void Safety_Check(void)
{
    float error = Pitch_Relative() - angle_setpoint;
    if (fabsf(error) > SAFETY_CUTOFF_DEG) {
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
        control_tick = 1;
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
 
    STBY_Set(0);
    Motor_SetLeft(0);
    Motor_SetRight(0);
 
    char msg[64];
    int len;

    HAL_TIM_Base_Start_IT(&htim4);
 
    uint32_t tick_count = 0;
    uint32_t last_heartbeat = HAL_GetTick();

    int16_t pwm = 999;

    while(1){
        pwm = -pwm;  // flip direction each cycle
        Motor_SetLeft(pwm);
        Motor_SetRight(pwm);

        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 500) {
            int32_t left_count  = Encoder_GetLeftCount();   // whatever your encoder read function is
            int32_t right_count = Encoder_GetRightCount();
            len = snprintf(msg, sizeof(msg), "pwm=%d L=%ld R=%ld\r\n", pwm, left_count, right_count);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
            HAL_Delay(20);  // ~50Hz logging, fast enough to see the lag
        }
    }
    /*char msg[128]; // widened for TEMP DEBUG raw axis fields
    int len;
 
    if (!MPU6050_Init()) {
        while (1) { // distinct fast-blink = sensor init failed
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
        }
    }
 
    Gyro_Calibrate(); // bot MUST be flat and still through this
 
    HAL_TIM_Base_Start_IT(&htim4);
 
    uint32_t tick_count = 0;
    uint32_t last_heartbeat = HAL_GetTick();
 
    while (1) {
        if (control_tick) {
            control_tick = 0;
            tick_count++;
 
            const float dt = 0.01f;
            Update_Pitch(dt);
            Safety_Check();
 
            if (motors_enabled) {
                int16_t pwm;

                float output = PID_Update(&angle_pid, angle_setpoint, Pitch_Relative(), dt);
                pwm = (int16_t)fmaxf(fminf(output, 999.0f), -999.0f);

                    /* Deadband compensation: a small correctly-computed PWM
                       value can still be too weak to overcome the GA12-N20
                       gearbox's static friction, so the wheels don't actually
                       move until error (and thus PWM) grows much larger than
                       it should need to — this shows up as a felt "delay"
                       before the motors respond to a slight tilt. Boost any
                       nonzero output up to a minimum floor that reliably
                       breaks static friction, found empirically by commanding
                       fixed PWM values from a dead stop until the wheels
                       reliably start turning.
                       TODO: verify/adjust MOTOR_DEADBAND for your actual
                       hardware — 120 is a starting guess, not measured. */
            /*    #define MOTOR_DEADBAND 155
                if (pwm > 0 && pwm < MOTOR_DEADBAND) pwm = MOTOR_DEADBAND;
                if (pwm < 0 && pwm > -MOTOR_DEADBAND) pwm = -MOTOR_DEADBAND;
                

                g_last_pwm = pwm; // TEMP DEBUG
                Motor_SetLeft(pwm);
                Motor_SetRight(pwm); // Motor_SetRight already inverts internally
            } else {
                g_last_pwm = 0; // TEMP DEBUG
                Motor_SetLeft(0);
                Motor_SetRight(0);
            }
 
            if (tick_count >= 10) { // 10Hz print
                tick_count = 0;
                len = snprintf(msg, sizeof(msg),
                    "rel_pitch=%.2f pwm=%d raw(ax,ay,az)=%d,%d,%d Kp=%.2f Ki=%.2f Kd=%.2f motors=%d\r\n",
                    Pitch_Relative(), g_last_pwm, g_raw_ax, g_raw_ay, g_raw_az,
                    angle_pid.kp, angle_pid.ki, angle_pid.kd, motors_enabled);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
            }
        }
 
        /* Repeating heartbeat blink — independent alive-signal, lesson
           from Stage 3's silent-PuTTY issue. */
        /*if (HAL_GetTick() - last_heartbeat >= 1000) {
            last_heartbeat = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
 
        uint8_t rx_byte;
        if (HAL_UART_Receive(&huart2, &rx_byte, 1, 0) == HAL_OK) {
            if (rx_byte == 'e' || rx_byte == 'E') {
                Motors_Enable(1);
                angle_pid.integral = 0; angle_pid.prev_error = 0; // clear stale windup
                len = snprintf(msg, sizeof(msg), "motors ENABLED\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
            } else if (rx_byte == 'd' || rx_byte == 'D') {
                Motors_Enable(0);
                len = snprintf(msg, sizeof(msg), "motors DISABLED\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
            } else if (rx_byte == 's' || rx_byte == 'S') {
                len = snprintf(msg, sizeof(msg),
                    "rel_pitch=%.2f pwm=%d raw(ax,ay,az)=%d,%d,%d Kp=%.2f Ki=%.2f Kd=%.2f motors=%d\r\n",
                    Pitch_Relative(), g_last_pwm, g_raw_ax, g_raw_ay, g_raw_az,
                    angle_pid.kp, angle_pid.ki, angle_pid.kd, motors_enabled);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
            } else if (rx_byte == 'p' || rx_byte == 'i' || rx_byte == 'k') {
                /* Live gain tuning — no reflash needed. Send e.g. "p20.0"
                   then Enter, or "i0.1" then Enter, or "k0.4" for Kd.
                   Buffers digits until newline, then parses as a float. */
            /*    char gain_letter = rx_byte;
                char numbuf[16];
                int numlen = 0;
                uint32_t start = HAL_GetTick();
                while (HAL_GetTick() - start < 3000 && numlen < (int)sizeof(numbuf) - 1) {
                    uint8_t nb;
                    if (HAL_UART_Receive(&huart2, &nb, 1, 0) == HAL_OK) {
                        if (nb == '\r' || nb == '\n') break;
                        numbuf[numlen++] = (char)nb;
                    }
                }
                numbuf[numlen] = '\0';
                float val = atof(numbuf);
                if (gain_letter == 'p') angle_pid.kp = val;
                else if (gain_letter == 'i') angle_pid.ki = val;
                else if (gain_letter == 'k') angle_pid.kd = val;
                len = snprintf(msg, sizeof(msg), "set %c=%.3f -> Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                    gain_letter, val, angle_pid.kp, angle_pid.ki, angle_pid.kd);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
            }
        }
    }*/
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
    huart2.Init.BaudRate = 9600;
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
    I2C_WriteReg(REG_ACCEL_CONFIG_2, 0x08);
    /* SMPLRT_DIV=19 with CONFIG=4 (DLPF on, 1kHz internal rate) gives
       an actual sensor output rate of 1000/(1+19) = 50Hz — HALF the
       100Hz rate this loop reads at. That means every second read was
       pulling a stale, unchanged sample, effectively halving real
       gyro/accel resolution and adding jitter to the derivative term.
       DIV=9 gives 1000/(1+9) = 100Hz, matching the control loop exactly. */
    I2C_WriteReg(REG_SMPLRT_DIV, 9);
    I2C_WriteReg(REG_CONFIG, 4);
    HAL_Delay(50);
 
    return 1;
}
 
static uint8_t MPU6050_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100) != HAL_OK) return 0;
 
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    return 1;
}
 
static void Gyro_Calibrate(void)
{
    int32_t sum = 0;
    int16_t ax, ay, az, gx;
    for (int i = 0; i < 500; i++) {
        if (MPU6050_ReadRaw(&ax, &ay, &az, &gx)) {
            sum += gx;
        }
        HAL_Delay(2);
    }
    imu.gyro_bias = (float)sum / 500.0f;
}
 
static void Update_Pitch(float dt)
{
    int16_t ax, ay, az, gx;
    if (!MPU6050_ReadRaw(&ax, &ay, &az, &gx)) return;
    if (ax <= -32760 || ay <= -32760 || az <= -32760) return; 
 
    g_raw_ax = ax; g_raw_ay = ay; g_raw_az = az; // TEMP DEBUG
 
    float accel_angle = atan2f((float)ax, (float)az) * 57.2958f;
    /* GYRO_CONFIG=0x18 selects the ±2000°/s range, which outputs
       16.4 LSB per °/s — NOT 131 LSB/(°/s), which is only correct for
       the much narrower ±250°/s range. Using the wrong constant here
       made gyro_rate read ~8x too small, effectively blinding the
       complementary filter to fast rotation (exactly the case that
       matters most during a quick fall). */
    float gyro_rate = ((float)gx - imu.gyro_bias) / 16.4f;
 
    imu.pitch_angle = 0.98f * (imu.pitch_angle + gyro_rate * dt)
                     + 0.02f * accel_angle;
    imu.gyro_rate_dps = gyro_rate; 
}
 
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