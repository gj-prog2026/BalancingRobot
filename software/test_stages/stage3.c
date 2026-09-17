#include "stm32f1xx_hal.h"
#include <stdio.h>
 
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_Init_All(void);
static void MX_TIM3_PWM_Init(void);
static void MX_USART2_UART_Init(void);
 
static void Motor_SetLeft(int16_t pwm);
static void Motor_SetRight(int16_t pwm);
static void STBY_Set(uint8_t on);
 
#define TEST_DUTY 150 // gentle bring-up speed, matches your original motor_test.c
 
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_All();
    MX_TIM3_PWM_Init();
    MX_USART2_UART_Init();
 
    /* STBY starts LOW (disabled) — motors physically cannot spin until
       explicitly enabled, same safety default we settled on earlier. */
    STBY_Set(0);
    Motor_SetLeft(0);
    Motor_SetRight(0);
 
    char msg[64];
    int len;
 
    uint8_t stby_state = 0;
    uint32_t last_heartbeat = HAL_GetTick();
 
    while (1) {
        /* Repeating heartbeat — NOT one-shot at boot — so the "ready"
           message survives the HC-05 connect-after-power race we hit
           earlier with the IMU stages. Also gives a visual LED pulse
           independent of PuTTY entirely. */
        if (HAL_GetTick() - last_heartbeat >= 1000) {
            last_heartbeat = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            len = snprintf(msg, sizeof(msg),
                "Stage 3 ready. f=fwd r=rev s=stop t=toggle STBY\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
        }
 
        uint8_t byte;
        if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK) {
            switch (byte) {
                case 'f':
                    Motor_SetLeft(TEST_DUTY);
                    Motor_SetRight(TEST_DUTY);
                    len = snprintf(msg, sizeof(msg), "FORWARD, duty=%d\r\n", TEST_DUTY);
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
                    break;
 
                case 'r':
                    Motor_SetLeft(-TEST_DUTY);
                    Motor_SetRight(-TEST_DUTY);
                    len = snprintf(msg, sizeof(msg), "REVERSE, duty=%d\r\n", TEST_DUTY);
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
                    break;
 
                case 's':
                    Motor_SetLeft(0);
                    Motor_SetRight(0);
                    len = snprintf(msg, sizeof(msg), "STOP\r\n");
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
                    break;
 
                case 't':
                    stby_state = !stby_state;
                    STBY_Set(stby_state);
                    len = snprintf(msg, sizeof(msg), "STBY now %s (PWM unchanged)\r\n",
                        stby_state ? "ENABLED" : "DISABLED");
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
                    break;
 
                default:
                    break;
            }
        }
    }
}
 
static void GPIO_Init_All(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
 
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); // frees PB3/PB4 from JTAG for STBY/BIN1
 
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
 
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7; // PWM out
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
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
    pwm = -pwm; // known inversion fix from earlier bring-up
 
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
 