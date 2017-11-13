#include "stm32f429xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_dac.h"
#include "math.h"

#define F_CPU       100000000UL
#define AHB1        F_CPU
#define APB1        F_CPU/4
#define APB1_TIM    APB1*2
#define APB2        F_CPU/2
#define APB2_TIM    APB2*2
#define SysTicks F_CPU/1000000

#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3

#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7

#define PIN8    8
#define PIN9    9
#define PIN10   10
#define PIN11   11

#define PIN12   12
#define PIN13   13
#define PIN14   14
#define PIN15   15

#define ADC_BUF_NUM 10
volatile uint16_t ADC_Buff[ADC_BUF_NUM];

#define NUM_OF_POINTS 20 
#define Pi 3.1416

uint16_t sin_buff[NUM_OF_POINTS], cos_buff[NUM_OF_POINTS];

void fill_sincos(void)
{
    for(uint16_t i=0;i<NUM_OF_POINTS;i++)
    {
        sin_buff[i]=fabs((sin(2*Pi*i/NUM_OF_POINTS)+1)*2047); //make sin 
        cos_buff[i]=fabs((cos(2*Pi*i/NUM_OF_POINTS)+1)*2047); //make cos 
    }
}

void USART2_IRQHandler(void)
{
    if(USART2->SR & USART_SR_RXNE)
    {
        USART6->DR = USART2->DR;
    }
}

void USART6_IRQHandler(void)
{
    if(USART6->SR & USART_SR_RXNE)
    {
        USART2->DR = USART6->DR;
    }
}

//SysTick Interrupt
void SysTick_Handler(void)
{
    static uint32_t del = 0;
    static uint8_t s = 0;
    del++;
    
    if (del == 1000000) // 1s
    {
        del = 0;
        if (s==0)
        {
            GPIOB->BSRR |= GPIO_BSRR_BR14;
            s=1;
        }else
        {
            GPIOB->BSRR |= GPIO_BSRR_BS14;
            s=0;            
        }       
    }
}

int main(void)
{
    //__IO uint32_t StartUpCounter = 0, HSEStatus = 0, HSIStatus = 0;
    
    /*RCC->CR |= RCC_CR_HSION;

    do
    {
        HSIStatus = RCC->CR & RCC_CR_HSIRDY;
        StartUpCounter++;
    }    
    while((HSIStatus == 0) && (StartUpCounter != HSI_TIMEOUT_VALUE));
    */
    if( (RCC->CR & RCC_CR_HSIRDY) != RESET)
    {
        /* Включаем буфер предвыборки FLASH */
        FLASH->ACR |= FLASH_ACR_PRFTEN;

        /* Конфигурируем Flash на 2 цикла ожидания */
    	/* Это нужно потому, что Flash не может работать на высокой частоте */        
    	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    	FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2WS;   
        
        /* HCLK = SYSCLK || AHB prescaler*/
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //AHB clk = 100MHz
        
    	/* PCLK1 = HCLK || APB Low speed prescaler (APB1)*/
    	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV4;

        /* PCLK2 = HCLK || APB high-speed prescaler (APB2)*/
    	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV2;
        
        /* Set PLL input sourse*/
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
        
        /*Set PLL M prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
        RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
        
        /*Set PLL N prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
        RCC->PLLCFGR |= (100 << RCC_PLLCFGR_PLLN_Pos);
        
        /*Set PLL P prescaler */
        //RCC->PLLCFGR |= RCC_PLLCFGR_PLLP;
        
        RCC->CR |= RCC_CR_PLLON;
        
        while ((RCC->CR & RCC_CR_PLLRDY) == 0)
        {}
            
        /*Set SYSCLK as PLL */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_PLL;
            
        while ((RCC->CFGR & RCC_CFGR_SWS) !=  RCC_CFGR_SWS_PLL)
        {}

    }

    fill_sincos();
    
    //Enable clock on port B, C, D
    RCC-> AHB1ENR |= RCC_AHB1ENR_GPIOAEN | 
                     RCC_AHB1ENR_GPIOBEN | 
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_GPIODEN |
                     RCC_AHB1ENR_GPIOEEN;
    
    //Set port D pin 0,7, 14 as out
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN0*2 |
                    GPIO_MODE_OUTPUT_PP << PIN7*2 |
                    GPIO_MODE_OUTPUT_PP << PIN14*2;

    //Set GPIOD PIN5 and 6 as usart2 RX TX
    GPIOD->MODER |= GPIO_MODE_AF_PP << PIN5*2 | 
                    GPIO_MODE_AF_PP << PIN6*2;
    GPIOD->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN5*2 | 
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN6*2;
    GPIOD->PUPDR |= GPIO_PULLUP << PIN5*2 |
                    GPIO_PULLUP << PIN6*2;
    GPIOD->AFR[0] |= GPIO_AF7_USART2 << PIN5*4 | 
                     GPIO_AF7_USART2 << PIN6*4;

    //Set GPIOC PIN6 and 7 as usart6 RX TX
    GPIOC->MODER |= GPIO_MODE_AF_PP << PIN6*2 |
                    GPIO_MODE_AF_PP << PIN7*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN6*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN7*2;
    GPIOC->PUPDR |= GPIO_PULLUP << PIN6*2 |
                    GPIO_PULLUP << PIN7*2;
    GPIOC->AFR[0] |= GPIO_AF8_USART6 << PIN6*4 |
                     GPIO_AF8_USART6 << PIN7*4;
                     
    //Set GPIOA PIN3 GPIOC PIN10 as analog input
    GPIOA->MODER |= GPIO_MODE_ANALOG << PIN3*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN3*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN3*2;

    GPIOC->MODER |= GPIO_MODE_ANALOG << PIN0*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2;
    GPIOC->PUPDR |= GPIO_NOPULL << PIN0*2;
    
    //Set GPIOA PIN4 as analog out
    GPIOA->MODER |= GPIO_MODE_ANALOG << PIN4*2 |
                    GPIO_MODE_ANALOG << PIN5*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN4*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN5*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN4*2 |
                    GPIO_NOPULL << PIN5*2;

    //Set GPIOE PIN 9,11,13,14 as TIM1 PWM out
    GPIOE->MODER |= GPIO_MODE_AF_PP << PIN9*2  |
                    GPIO_MODE_AF_PP << PIN11*2 |
                    GPIO_MODE_AF_PP << PIN13*2 |
                    GPIO_MODE_AF_PP << PIN14*2;
    GPIOE->OSPEEDR |= GPIO_SPEED_FREQ_MEDIUM << PIN9*2  |
                      GPIO_SPEED_FREQ_MEDIUM << PIN11*2 |
                      GPIO_SPEED_FREQ_MEDIUM << PIN13*2 |
                      GPIO_SPEED_FREQ_MEDIUM << PIN14*2;
    GPIOE->PUPDR |= GPIO_NOPULL << PIN9*2  |
                    GPIO_NOPULL << PIN11*2 |
                    GPIO_NOPULL << PIN13*2 |
                    GPIO_NOPULL << PIN14*2;
    GPIOE->AFR[1] |= GPIO_AF1_TIM1 << (PIN9*4  - 32) |
                     GPIO_AF1_TIM1 << (PIN11*4 - 32) |
                     GPIO_AF1_TIM1 << (PIN13*4 - 32) |
                     GPIO_AF1_TIM1 << (PIN14*4 - 32);

    GPIOD->MODER |= GPIO_MODE_AF_PP << PIN12*2 |
                    GPIO_MODE_AF_PP << PIN13*2 |
                    GPIO_MODE_AF_PP << PIN14*2 |
                    GPIO_MODE_AF_PP << PIN15*2;
    GPIOD->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN12*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN13*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN14*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN15*2;
    GPIOD->PUPDR |= GPIO_NOPULL << PIN12*2 |
                    GPIO_NOPULL << PIN13*2 |
                    GPIO_NOPULL << PIN14*2 |
                    GPIO_NOPULL << PIN15*2;
    GPIOD->AFR[1] |= GPIO_AF2_TIM4 << (PIN12*4 - 32) |
                     GPIO_AF2_TIM4 << (PIN13*4 - 32) |
                     GPIO_AF2_TIM4 << (PIN14*4 - 32) |
                     GPIO_AF2_TIM4 << (PIN15*4 - 32);

    //MX_USART2_UART_Init();
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = F_CPU/(115200*4);
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;    
    
    //MX_USART6_UART_Init();
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    USART6->BRR = F_CPU/(115200*2);
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;    

    //Timer 3s Init
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//50 Mhz
    TIM3->PSC = 49;
    TIM3->ARR = 10;
    TIM3->CR2 = TIM_CR2_MMS_1;
    TIM3->CR1 = TIM_CR1_CEN;
    
    //ADC1 Init
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //включаем тактирование
    ADC1->CR2 = ADC_CR2_ADON | 
                ADC_CR2_DMA |
                ADC_CR2_DDS |
                ADC_CR2_EXTSEL_3 | 
                ADC_CR2_EXTEN_0;// подаем питание на АЦП
    ADC1->CR1 = ADC_CR1_SCAN;// разрешаем прерывания по окончанию преобразования
    ADC1->SMPR2 = ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2;// скорость для 1 преобразвания
    ADC1->SMPR1 = ADC_SMPR1_SMP10_1 | ADC_SMPR1_SMP10_2; // скорость для 2 преобразвания
    ADC1->SQR1 =  1 << ADC_SQR1_L_Pos; // 2 преобразования
    ADC1->SQR3 = (3 << ADC_SQR3_SQ1_Pos) | (10 << ADC_SQR3_SQ2_Pos); // выбор 3 и 10 каналов
    
    //DMA Init
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->CR = DMA_SxCR_CIRC |
                        DMA_SxCR_MINC | 
                        DMA_SxCR_MSIZE_0 | 
                        DMA_SxCR_PSIZE_0;
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t) ADC_Buff;
    DMA2_Stream0->NDTR = ADC_BUF_NUM;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
                   
    //DAC Init               
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR = DAC_CR_EN1 |
              DAC_CR_DMAEN1 |
              DAC_CR_TEN1 |
              DAC_CR_EN2 |
              DAC_CR_DMAEN2 |
              DAC_CR_TEN2;
    
    //TIM6 init for DAC SIN OUT
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //50Mhz
    TIM6->PSC = 4; 
    TIM6->ARR = 10;
    TIM6->CR1 = TIM_CR1_CEN;
    TIM6->CR2 = TIM_CR2_MMS_1;
    
    //DMA for DAC
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream5->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_CIRC |
                       DMA_SxCR_MINC |
                       DMA_SxCR_PSIZE_0 |
                       DMA_SxCR_MSIZE_0 |
                       DMA_SxCR_CHSEL_0 |
                       DMA_SxCR_CHSEL_1 |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream5->PAR = (uint32_t) &DAC->DHR12R1;
    DMA1_Stream5->M0AR = (uint32_t)sin_buff;
    DMA1_Stream5->NDTR = NUM_OF_POINTS;
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    DMA1_Stream6->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_CIRC |
                       DMA_SxCR_MINC |
                       DMA_SxCR_PSIZE_0 |
                       DMA_SxCR_MSIZE_0 |
                       DMA_SxCR_CHSEL_0 |
                       DMA_SxCR_CHSEL_1 |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream6->PAR = (uint32_t) &DAC->DHR12R2;
    DMA1_Stream6->M0AR = (uint32_t)cos_buff;
    DMA1_Stream6->NDTR = NUM_OF_POINTS;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    
    //TIM1 PWM mode
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//APB2_TIM clk=100MHz
    TIM1->PSC = 99;
    TIM1->ARR = 20000;
    TIM1->CCER = TIM_CCER_CC1E |
                 TIM_CCER_CC2E |
                 TIM_CCER_CC3E |
                 TIM_CCER_CC4E;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 |
                  TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2M_1 |
                  TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 |
                  TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC4M_1 |
                  TIM_CCMR2_OC4M_2;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->CCR1 = 20000;
    TIM1->CCR2 = 2000;
    TIM1->CCR3 = 2200;
    TIM1->CCR4 = 10000;
    TIM1->CR1 = TIM_CR1_CEN;
    
    //TIM4 for input capture
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;//APB1_TIM clk = 50MHz
    TIM4->CCMR1 = TIM_CCMR1_CC1S_0 |
                  TIM_CCMR1_CC2S_0;
    TIM4->CCMR2 = TIM_CCMR2_CC3S_0 |
                  TIM_CCMR2_CC4S_0;
    TIM4->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E |
                 TIM_CCER_CC2P |
                 TIM_CCER_CC2E |
                 TIM_CCER_CC3P |
                 TIM_CCER_CC3E |
                 TIM_CCER_CC4P |
                 TIM_CCER_CC4E;
    TIM4->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM4->PSC = 49;
    TIM4->CR1 = TIM_CR1_CEN;
    
       
 /*   
    RCC-> APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 =  SPI_CR1_BR_2| SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_DFF;
    
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_CPHA;
    
    uint32_t s = convert(3);
  */  
    
    SysTick_Config(SysTicks);
    
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART6_IRQn);

    GPIOB->BSRR |= GPIO_BSRR_BS0;
    
    while (1)
    {
      if(!(GPIOC->IDR & GPIO_PIN_13)) 
      {
          GPIOB->BSRR |= GPIO_BSRR_BR7;
      }      
      else
      {
          GPIOB->BSRR |= GPIO_BSRR_BS7;
      }     
    }   
    return 0;
}