#include <stdint.h>
#include "stm32f4xx.h"
#include "time.h"

static volatile timeUs_t ticks;

void TicksInit(void)
{
   SysTick_Config(SystemCoreClock / 1000000);
}
void SysTick_Handler(void)
{
    ticks++;
}

void DelayUs(uint32_t us)
{
    uint32_t t = ticks;
    while (ticks - t < us)
        ;
}
void DelayMs(uint32_t ms)
{
    uint32_t t = ticks;
    while (ticks - t < ms*1000 )
        ;
}

timeUs_t micros(void)
{
    return ticks;
}