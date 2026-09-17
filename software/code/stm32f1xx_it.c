#include "stm32f1xx_hal.h"
 
/* Core Cortex-M3 exception handlers */
 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
    while (1)
    {
    }
}
 
void MemManage_Handler(void)
{
    while (1)
    {
    }
}
 
void BusFault_Handler(void)
{
    while (1)
    {
    }
}
 
void UsageFault_Handler(void)
{
    while (1)
    {
    }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
/* This is the one that actually fixes the hang: HAL_Init() enables the
   SysTick interrupt, and without this handler it falls back to the
   startup file's default weak handler, which is an infinite loop. */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

