#include "stm32f1xx_hal.h"

TIM_HandleTypeDef htim4;
volatile uint8_t control_tick = 0;


static void SystemClock_Config(void);
static void Error_Handler(void);
static void LED_Init(void);
static void MX_TIM4_ControlTick_Init(void);

void TIM4_IRQHandler(void){
    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)){
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        control_tick = 1;
    }
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    MX_TIM4_ControlTick_Init();

    HAL_TIM_Base_Start_IT(&htim4);

    uint32_t tick_count = 0;

    while(1){
        if(control_tick){
            control_tick = 0;
            tick_count++;

            //toggles every 50th tick
            if(tick_count >= 50){
                tick_count = 0;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }
    }
}

static void LED_Init(void){
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

/*  TIM4 sits on APB1. APB1 prescaler is DIV2 (36MHz)
    when APB1 prescaler != 1, timer clock automatically
    doubles -> TIM4 sees 72MHz*/
    
static void MX_TIM4_ControlTick_Init(void){
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
    while (1) {
        // stuck here means clock or TIM4 init failed
    }
}