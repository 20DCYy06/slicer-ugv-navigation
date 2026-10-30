/* Native test register doubles only. NEVER on MCU include paths. */
#include <stdint.h>
typedef struct { uint32_t CR1,PSC,ARR,CCR1,CCR2,CCR3,CCR4,CCMR1,CCMR2,CCER,BDTR,EGR,SMCR,CNT; } TIM_TypeDef;
typedef struct { uint32_t SR,DR,CR1,CR2,CR3,BRR; } USART_TypeDef;
typedef struct { uint32_t IDR,MODER,OTYPER,OSPEEDR,PUPDR,AFR[2],BSRRH; } GPIO_TypeDef;
typedef struct { uint32_t CR,CFGR,AHB1ENR,APB1ENR,APB2ENR,CSR; } RCC_TypeDef;
typedef struct { uint32_t CR2,SR,DR,SMPR2,SQR3,CCR; } ADC_TypeDef;
typedef struct { uint32_t KR,PR,RLR,SR; } IWDG_TypeDef;
typedef struct { uint32_t CTRL; } SysTick_TypeDef;
typedef struct { uint32_t VTOR,CPACR,CFSR,HFSR; } SCB_TypeDef;
typedef struct { uint32_t ICER[8],ICPR[8]; } NVIC_TypeDef;
static TIM_TypeDef t1,t2,t3,t4,t5,t9,t10,t11;
static USART_TypeDef u1,u2,u3;
static GPIO_TypeDef ga,gb,gd,ge;
static RCC_TypeDef rc;
static ADC_TypeDef ad,ac;
static IWDG_TypeDef iw;
static SysTick_TypeDef sy;
static SCB_TypeDef sc;
static NVIC_TypeDef nv;
#define TIM1 (&t1)
#define TIM2 (&t2)
#define TIM3 (&t3)
#define TIM4 (&t4)
#define TIM5 (&t5)
#define TIM9 (&t9)
#define TIM10 (&t10)
#define TIM11 (&t11)
#define USART1 (&u1)
#define USART2 (&u2)
#define USART3 (&u3)
#define GPIOA (&ga)
#define GPIOB (&gb)
#define GPIOD (&gd)
#define GPIOE (&ge)
#define RCC (&rc)
#define ADC1 (&ad)
#define ADC (&ac)
#define IWDG (&iw)
#define SysTick (&sy)
#define SCB (&sc)
#define NVIC (&nv)
#define USART_SR_ORE (1u<<3)
#define USART_SR_FE (1u<<1)
#define USART_SR_NE (1u<<2)
#define USART_SR_PE 1u
#define USART_SR_RXNE (1u<<5)
#define USART_SR_TXE (1u<<7)
#define USART_CR1_UE (1u<<13)
#define USART_CR1_TE (1u<<3)
#define USART_CR1_RE (1u<<2)
#define USART_CR1_RXNEIE (1u<<5)
#define ADC_CR2_SWSTART (1u<<30)
#define ADC_SR_EOC (1u<<1)
#define ADC_CR2_ADON 1u
#define TIM_BDTR_MOE (1u<<15)
#define TIM_CR1_ARPE (1u<<7)
#define TIM_CR1_CEN 1u
#define RCC_CR_HSION 1u
#define RCC_CR_HSIRDY 2u
#define RCC_CFGR_SWS 12u
#define RCC_CR_PLLON (1u<<24)
#define RCC_CSR_LSION 1u
#define RCC_CSR_LSIRDY 2u
#define USART1_IRQn 37
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define __enable_irq() ((void)0)
#define __disable_irq() ((void)0)
#define __get_PRIMASK() 0u
#define __set_PRIMASK(n) ((void)(n))
#define NVIC_EnableIRQ(n) ((void)(n))
#define SysTick_Config(n) ((void)(n))
