#include "stm32f4xx.h"
#include "config.h"
void initCLOCK(void)
{
    //we want to run at 168MHz using HSI
    /* HAL values
    PLLM = 8
    PLLN = 168
    PLLP = 2
    PLLq = 4
    
    Real values:
    plln = 252
    */
    RCC->CR &= ~(RCC_CR_HSEON);
    RCC->CR |= RCC_CR_HSION; //turn HSI on
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
    };                       //wait for HSI ready
    RCC->CFGR = (uint32_t)0; //reset CFGR register - source clock is set as HSI by default

    //RCC->CFGR |= (RCC_CFGR_PPRE1_DIV4|RCC_CFGR_PPRE2_DIV2);
    RCC->PLLCFGR = (uint32_t)0x24003010; //reset PLLCFGR register
    RCC->CIR = (uint32_t)0;              //disable all interupts
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); //reset PLLM
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2;  //plm 1000 = 8
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN); //reset PLLN

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7; //plln = 252

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP); //reset PLLP
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_1;
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ); //reset PLLQ
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2;

    RCC->CR |= RCC_CR_PLLON; //turn PLL on
    while (!(RCC_CR_PLLRDY & RCC->CR))
    {
    }; //wait for PLL

    FLASH->ACR &= ~0x00000007;
    FLASH->ACR |= 0x3;

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC_CFGR_SWS_PLL & RCC->CFGR))
    {
    };
    volatile uint32_t test = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
    test = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16;
    SystemCoreClockUpdate();
}

void initRCC(void)
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN);

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}
static void initGPIO(void)
{   
    GPIOC->MODER |= GPIO_MODER_MODER8_0;  //LEDPin
    GPIOD->MODER |= GPIO_MODER_MODER11_0; //LEDPin
    //USART 3
    GPIOD->MODER &=~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOD->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOD->AFR[1] &= ~(0x000000FF);
    GPIOD->AFR[1] |= 0x00000077;
    /*USART 1*/
    GPIOA->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER9_1);
    GPIOA->AFR[1] &= ~(0x00000FF0);
    GPIOA->AFR[1] |= (0x00000777);
    /*FTDI converter pins*/
    GPIOB->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
    GPIOB->ODR |= (GPIO_ODR_ODR_4 | GPIO_ODR_ODR_7);
    GPIOB->ODR |= (GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6);
    GPIOB->ODR &= ~(GPIO_ODR_ODR_4 | GPIO_ODR_ODR_7);

    /*FSM305 IMU*/
    GPIOA->MODER &=~(GPIO_MODER_MODER15|GPIO_MODER_MODER8|GPIO_MODER_MODER12);
    GPIOA->MODER |= GPIO_MODER_MODER15_0; //PA15 CSN
    GPIOA->MODER |= GPIO_MODER_MODER8_0; //output    BOOTN
    GPIOA->MODER |= GPIO_MODER_MODER12_0;//waken

    GPIOC->MODER &= ~(GPIO_MODER_MODER9|GPIO_MODER_MODER7);   //PC9 interrupt - input
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0; //pull up mode
    GPIOC->MODER |= GPIO_MODER_MODER7_0;  //output NRST

    GPIOC->MODER &=~(GPIO_MODER_MODER10|GPIO_MODER_MODER11|GPIO_MODER_MODER12);
    GPIOC->MODER |= (GPIO_MODER_MODER10_1|GPIO_MODER_MODER11_1|GPIO_MODER_MODER12_1);    //C10-12 AF
    GPIOC->AFR[1] |= (0x6<<8|0x6<<12|0x6<<16);//AF6 for spi3
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10|GPIO_OSPEEDER_OSPEEDR11|GPIO_OSPEEDER_OSPEEDR12);    //high speed

}
static uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

static void initUSART1(void)
{
    volatile uint32_t baud_rate = SystemCoreClock;
    volatile uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4; //AHB prescaler bits 7:4
    baud_rate = baud_rate >> APBAHBPrescTable[tmp];
    tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> 10;       //apb1 prescaler bits 12:10
    baud_rate = baud_rate >> APBAHBPrescTable[tmp]; //fancy table stolen from std_periph
    baud_rate /= USART1_BAUD;
    USART1->BRR = (uint16_t)baud_rate;
    USART1->CR1 = USART_CR1_RE | USART_CR1_TE; //enable uart rx and tx
    USART1->CR1 |= USART_CR1_IDLEIE;           //enable idle interrupt

    //enable DMA channel4 stream 7 for tx -> datasheet dma assignment
    DMA2_Stream7->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_DIR_0; //channel4, memory incrementation, mem ->periph
    DMA2_Stream7->PAR = (uint32_t) & (USART1->DR);                        //source is always usart1 data register
    DMA2_Stream7->CR |= DMA_SxCR_TCIE;                                    //enable transfer complete interrupt

    //enable DMA channel4 stream 5 for rx
    DMA2_Stream5->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC; //channel4, memory incrementation, circular buffer
    DMA2_Stream5->PAR = (uint32_t) & (USART1->DR);
    DMA2_Stream5->CR |= DMA_SxCR_TCIE;

    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR; //enable dma tx_rx
    USART1->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void initUSART3(void)
{
    volatile uint32_t baud_rate = SystemCoreClock;
    volatile uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4; //AHB prescaler bits 7:4
    baud_rate = baud_rate >> APBAHBPrescTable[tmp];
    tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;       //apb1 prescaler bits 12:10
    baud_rate = baud_rate >> APBAHBPrescTable[tmp]; //fancy table stolen from std_periph
    baud_rate /= USART3_BAUD;
    USART3->BRR = (uint16_t)baud_rate;         //Baud rate - hardcoded to 115200
    USART3->CR1 = USART_CR1_RE | USART_CR1_TE; //enable uart rx and tx
    USART3->CR1 |= USART_CR1_IDLEIE;           //enable idle interrupt

    //enable DMA channel4 stream 3 for tx -> datasheet dma assignment
    DMA1_Stream3->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_DIR_0; //channel4, memory incrementation, mem ->periph
    DMA1_Stream3->PAR = (uint32_t) & (USART3->DR);                        //source is always usart3 data register
    DMA1_Stream3->CR |= DMA_SxCR_TCIE;                                    //enable transfer complete interrupt

    //enable DMA channel4 stream 1 for rx
    DMA1_Stream1->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC; //channel4, memory incrementation, circular buffer
    DMA1_Stream1->PAR = (uint32_t) & (USART3->DR);
    DMA1_Stream1->CR |= DMA_SxCR_TCIE;

    USART3->CR3 = USART_CR3_DMAT | USART_CR3_DMAR; //enable dma transmit and receibe
    USART3->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(39);
    NVIC_SetPriority(39, 0);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}
void initINT(void)
{
/*--------PC9 for FSM305--------*/
SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PC;  //PC9 for FSM305
EXTI->IMR |=EXTI_IMR_MR9;                      //enable mask
EXTI->FTSR |=EXTI_FTSR_TR9;
EXTI->RTSR &=~EXTI_RTSR_TR9;                    //falling edge
NVIC_SetPriority(EXTI9_5_IRQn, 1);
//NVIC_EnableIRQ(EXTI9_5_IRQn);
/*-------------------------*/
}
void initSPI3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    SPI3->CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1
	
    SPI3->CR1 |= (1<<2);  // Master Mode
        
    SPI3->CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
        
    SPI3->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
        
    SPI3->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
        
    SPI3->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
        
    SPI3->CR1 &= ~(1<<11);  // DFF=0, 8 bit data
        
    SPI3->CR2 = 0;
}
void initSystem()
{
    initCLOCK();
    initRCC();
    initUSART1();
    initUSART3();
    
    initSPI3();
    initGPIO();
    initINT();
    
}