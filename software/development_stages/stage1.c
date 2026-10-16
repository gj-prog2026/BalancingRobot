#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
 
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef  hi2c1;
 
volatile uint8_t control_tick = 0;
 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void LED_Init(void);
static void MX_TIM4_ControlTick_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
 
static uint8_t I2C_ReadReg(uint8_t reg, uint8_t *out);
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t val);
static uint8_t MPU6050_Init(void);
static uint8_t MPU6050_ReadRaw(int16_t *ay, int16_t *az, int16_t *gx);
static void Gyro_Calibrate(void);
static void Update_Pitch(float dt);
 
typedef struct {
    float pitch_angle;
    float gyro_bias;
} imu_state_t;
 
static imu_state_t imu = {0};
 
#define MPU6050_ADDR       (0x68 << 1)
#define REG_PWR_MGMT_1     0x6B
#define REG_PWR_MGMT_2     0x6C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_WHO_AM_I       0x75
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
    LED_Init();
    MX_TIM4_ControlTick_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
 
    char msg[64];
    int len;
 
    if (!MPU6050_Init()) {
        /* Wake never confirmed within retry budget — flash LED fast forever
           as a distinct "sensor init failed" signal, rather than silently
           continuing with a sleeping sensor. */
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
        }
    }
 
    len = snprintf(msg, sizeof(msg), "MPU6050_Init OK, calibrating gyro...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
 
    Gyro_Calibrate(); // bot MUST be flat and still
 
    len = snprintf(msg, sizeof(msg), "calibration done, streaming pitch...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
 
    HAL_TIM_Base_Start_IT(&htim4);
 
    uint32_t tick_count = 0;
 
    while (1) {
        if (control_tick) {
            control_tick = 0;
            tick_count++;
 
            Update_Pitch(0.01f);
 
            if (tick_count >= 10) { // 10Hz print rate
                tick_count = 0;
                len = snprintf(msg, sizeof(msg), "pitch=%.2f\r\n", imu.pitch_angle);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }
 
        uint8_t rx_byte;
        if (HAL_UART_Receive(&huart2, &rx_byte, 1, 0) == HAL_OK) {
            HAL_UART_Transmit(&huart2, &rx_byte, 1, 100);
        }
    }
}
 
static void LED_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
 
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_2;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
 
    g.Pin = GPIO_PIN_3;
    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &g);
 
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
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
 
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_OD;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
 
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
 
/* Manual write-then-read (not HAL_I2C_Mem_Read/Write) — sidesteps the
   STM32F1 repeated-START HAL quirk found during bring-up. */
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
 
/* Sensor identified as ICM-20689 (WHO_AM_I=0x98). Needs RESET before
   wake (not just a direct wake write), and the wake write must be
   VERIFIED and RETRIED — a fixed post-reset delay was found unreliable
   during bring-up; the sleep bit sometimes didn't clear on the first
   write. Returns 1 on confirmed success, 0 if wake never confirmed
   within the retry budget. */
static uint8_t MPU6050_Init(void)
{
    I2C_WriteReg(REG_PWR_MGMT_1, 0x80); // reset
    HAL_Delay(100);
 
    uint8_t pwr1 = 0xFF;
    for (int attempt = 0; attempt < 20; attempt++) {
        I2C_WriteReg(REG_PWR_MGMT_1, 0x00); // clear sleep bit
        HAL_Delay(20);
        if (!I2C_ReadReg(REG_PWR_MGMT_1, &pwr1)) continue;
        if (!(pwr1 & SLEEP_BIT)) break; // confirmed awake
    }
    if (pwr1 & SLEEP_BIT) return 0; // never confirmed within budget
 
    I2C_WriteReg(REG_PWR_MGMT_2, 0x00);   // enable all accel+gyro axes
    I2C_WriteReg(REG_GYRO_CONFIG, 0x18);  // full-scale range
    I2C_WriteReg(REG_ACCEL_CONFIG, 0x00); // ±2g range
    I2C_WriteReg(REG_ACCEL_CONFIG_2, 0x08);
    I2C_WriteReg(REG_SMPLRT_DIV, 19);
    I2C_WriteReg(REG_CONFIG, 4);
    HAL_Delay(50);
 
    return 1;
}
 
static uint8_t MPU6050_ReadRaw(int16_t *ay, int16_t *az, int16_t *gx)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100) != HAL_OK) return 0;
 
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    return 1;
}
 
static void Gyro_Calibrate(void)
{
    int32_t sum = 0;
    int16_t ay, az, gx;
    for (int i = 0; i < 500; i++) {
        if (MPU6050_ReadRaw(&ay, &az, &gx)) {
            sum += gx;
        }
        HAL_Delay(2);
    }
    imu.gyro_bias = (float)sum / 500.0f;
}
 
static void Update_Pitch(float dt)
{
    int16_t ay, az, gx;
    if (!MPU6050_ReadRaw(&ay, &az, &gx)) return;
 
    float accel_angle = atan2f((float)ay, (float)az) * 57.2958f;
    float gyro_rate = ((float)gx - imu.gyro_bias) / 131.0f;
 
    imu.pitch_angle = 0.98f * (imu.pitch_angle + gyro_rate * dt)
                     + 0.02f * accel_angle;
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