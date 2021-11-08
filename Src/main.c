#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "time/time.h"
#include "setup.h"
#include "IO/LED.h"
#include "Sensors/BNO080/bno080.h"
#include "Sensors/BNO080/hw.h"
volatile int i = 0;

void setup(void)
{
    initSystem();
    TicksInit();
}
volatile struct GyroIntegratedRVFloat_t gameRV;

int main(void)
{

    setup();

    while (1)
    {
       
    }
}