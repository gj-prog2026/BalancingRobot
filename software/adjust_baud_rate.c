/*
 * hc05_config.c - ONE-OFF utility: set an HC-05 to 115200 baud from the STM32.
 *
 * Uses the same USART2 wiring as the balancing bot (PA2 = TX -> HC-05 RXD,
 * PA3 = RX <- HC-05 TXD). No USB-UART adapter needed.
 *
 * THIS IS A STANDALONE REPLACEMENT FOR main.c. Do not keep it in the project
 * alongside your real main.c (two main() functions = failed build = stale flash).
 * Swap the contents in, flash, run once, then put your real main.c back.
 *
 * Procedure
 *   1. Flash this program, then remove ALL power from the bot.
 *   2. Hold the HC-05's button (or tie its KEY/EN pin high) and apply power to
 *      the board and the module together. Release once the module LED is
 *      blinking SLOWLY (about once per 2 s) - that's AT mode, fixed at 38400 baud.
 *   3. This program talks to it at 38400, sets 115200 / 1 stop bit / no parity,
 *      then reads the setting back to verify.
 *   4. LED result (Blue Pill LED is active-low; swap LED_ON/LED_OFF if yours isn't):
 *        solid ON    = success
 *        1 blink     = no reply to "AT"  (module not in AT mode, or TX/RX wiring)
 *        2 blinks    = module rejected the baud-change command
 *        3 blinks    = command accepted but read-back doesn't show 115200
 *   5. Power off, flash your real firmware, power up WITHOUT holding the button.
 *      The module now runs at 115200 in normal mode; the new rate applies from
 *      the next power-up out of AT mode.
 */
 
#include "stm32f1xx_hal.h"
#include <string.h>
 
UART_HandleTypeDef huart2;
 
static void SystemClock_Config(void);
static void GPIO_Init_All(void);
static void MX_USART2_UART_Init(uint32_t baud);
static void Error_Handler(void);
 
#define LED_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
 
#define AT_MODE_BAUD    38400u                      /* HC-05 AT mode is always 38400 */
#define SET_CMD         "AT+UART=115200,0,0\r\n"    /* baud, stop bits (0 = 1), parity (0 = none) */
#define EXPECT_IN_READ  "115200"                    /* "+UART:115200,0,0" */
 
/* Send one AT command and collect the reply until OK / FAIL / ERROR or timeout.
   HC-05 requires the command to end in CR+LF. */
static void At_Cmd(const char *cmd, char *resp, uint16_t resp_size, uint32_t timeout_ms)
{
    uint8_t  b;
    uint16_t len = 0;
 
    /* throw away anything stale sitting in the receiver */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    while (HAL_UART_Receive(&huart2, &b, 1, 0) == HAL_OK) { }
    resp[0] = '\0';
 
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
 
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (HAL_UART_Receive(&huart2, &b, 1, 2) == HAL_OK) {
            if (len < resp_size - 1) {
                resp[len++] = (char)b;
                resp[len] = '\0';
            }
            if (strstr(resp, "OK\r\n") || strstr(resp, "FAIL") || strstr(resp, "ERROR")) {
                break;
            }
        }
    }
}
 
static void Blink_Code(uint8_t n)   /* never returns */
{
    while (1) {
        for (uint8_t i = 0; i < n; i++) {
            LED_ON();  HAL_Delay(200);
            LED_OFF(); HAL_Delay(300);
        }
        HAL_Delay(1500);
    }
}
 
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_All();
    MX_USART2_UART_Init(AT_MODE_BAUD);
 
    char    resp[64];
    uint8_t in_at_mode = 0;
 
    HAL_Delay(1500);                /* let the HC-05 finish booting */
 
    /* Poll with plain "AT" for up to ~10 s so you have time to get the module into AT mode */
    for (int attempt = 0; attempt < 20 && !in_at_mode; attempt++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* flicker = still trying */
        At_Cmd("AT\r\n", resp, sizeof(resp), 300);
        if (strstr(resp, "OK")) {
            in_at_mode = 1;
        } else {
            HAL_Delay(200);
        }
    }
    LED_OFF();
    if (!in_at_mode) Blink_Code(1);
 
    At_Cmd(SET_CMD, resp, sizeof(resp), 1000);
    if (!strstr(resp, "OK")) Blink_Code(2);
 
    At_Cmd("AT+UART?\r\n", resp, sizeof(resp), 1000);
    if (!strstr(resp, EXPECT_IN_READ)) Blink_Code(3);
 
    LED_ON();                       /* success: solid on */
    while (1) { }
}
 
/* ---------------- Init (same clock tree and pins as the balancing bot) ---------------- */
 
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
    LED_OFF();
 
    /* Keep the motor driver inert while this runs: STBY and all direction pins low */
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5;             // AIN1, AIN2
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; // STBY, BIN1, BIN2
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
 
    g.Pin = GPIO_PIN_2; // USART2 TX
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
 
    g.Pin = GPIO_PIN_3; // USART2 RX
    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &g);
}
 
static void MX_USART2_UART_Init(uint32_t baud)
{
    __HAL_RCC_USART2_CLK_ENABLE();
 
    huart2.Instance = USART2;
    huart2.Init.BaudRate = baud;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
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