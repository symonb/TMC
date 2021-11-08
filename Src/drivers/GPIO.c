#include <stdbool.h>
#include "stm32f4xx.h"

void GPIOTogglePin(GPIO_TypeDef* bus, uint16_t pin){
    ((GPIO_TypeDef*)bus)->ODR ^=pin;
}

void GPIOSetPin(GPIO_TypeDef* bus, uint16_t pin, bool value){
    if(value)
        ((GPIO_TypeDef*)bus)->BSRRL = 1<<pin;
    else
        ((GPIO_TypeDef*)bus)->BSRRH = (1<<pin)<<16;
}