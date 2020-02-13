/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "stm32l1xx_ll_tim.h"


#define Fs   (uint16_t)740

#define Cs  (uint16_t)1109
#define D2  (uint16_t)1175
#define E2  (uint16_t)1318
#define F2s   (uint16_t)1480
#define G2   (uint16_t)1568
#define A2   (uint16_t)1760
#define P   (uint16_t)1

#define A					(int)880
#define A1				(int)1760
#define B					(int)989
#define C					(int)1030
#define D					(int)1175
#define E					(int)1318
#define F					(int)1397
#define G					(int)1568
#define Abs				(int)15999999
#define MUTE			(int)16000000
	
#define ME					(int)329
#define MC					(int)261
#define MG					(int)391
#define	MA					(int)440
#define	MF					(int)349
#define	MD					(int)293
#define	MF1					(int)369
#define	MD1					(int)311
#define	MMC					(int)277
#define	MME					(int)311
#define	MMB					(int)246
	
#define BPM								130 // TEMPO

/*for 10ms update event*/
#define TIMx_PSC			2 
#define TIMxx_PSC			32000 
#define TIMx_ARR			10

/*Macro function for ARR calculation*/
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))

/*TEMPO CALCULATION*/
#define NOTE_TIME							(float)((60.0/BPM)/2.0) //1 note = eight note
#define MUTE_TIME							(float)0.05
#define MUTE_ARR							(int)(((32000000 * MUTE_TIME) / ((32000)))-1)
#define EIGHT_NOTE_ARR				(int)(((32000000 * NOTE_TIME) / ((32000)))-1)
#define LAST_EIGHT_NOTE_ARR		(int)((((32000000 * NOTE_TIME) / ((32000)))-1)-MUTE_ARR)

void SystemClock_Config(void);
void TIM_BASE_Config(uint16_t);
void TIM_OC_GPIO_Config(void);
void TIM_OC_Config(uint16_t);
void TIM_BASE_DurationConfig(void);
void GPIO_Config(void);

int tetris[] = {E,E,MUTE,B,MUTE,C,MUTE,D,D,MUTE,C,MUTE,B,MUTE,A,A,MUTE,A,MUTE,C,MUTE,E,E,MUTE,D,MUTE,C,MUTE,B,B,B,MUTE,C,MUTE,D,D,MUTE,E,E,MUTE,C,C,MUTE,A,A,MUTE,A,A,MUTE,Abs,Abs,Abs,MUTE,D,D,MUTE,F
									,MUTE,A1,A1,MUTE,G,MUTE,F,MUTE,E,E,E,MUTE,C,MUTE,E,E,MUTE,D,MUTE,C,MUTE,B,B,MUTE,B,MUTE,C,MUTE,D,D,MUTE,E,E,MUTE,C,C,MUTE,A,A,MUTE,A,A,MUTE,Abs,MUTE};
	
int Pirate[] = {D,D,MUTE,D,MUTE,D,D,MUTE,D,MUTE,D,D,MUTE,D,MUTE,D,MUTE,D,D,MUTE,D,MUTE,D,MUTE,D,D,MUTE,D,
												MUTE,D,D,MUTE,D,MUTE,D,D,MUTE,D,D,MUTE,A,MUTE,Cs,MUTE,D,D,MUTE,D,D,MUTE,D,MUTE,E,MUTE,Fs,Fs,MUTE,Fs,Fs,MUTE,Fs,MUTE,G,MUTE,E,E,MUTE,E,
												E,MUTE,D,MUTE,Cs,MUTE,Cs,MUTE,D,D,MUTE,A,MUTE,Cs,MUTE,D,D,MUTE,D,D,MUTE,D,MUTE,E,MUTE,Fs,Fs,MUTE,Fs,Fs,MUTE,Fs,MUTE,G,MUTE,E,E,MUTE,E,
												E,MUTE,D,MUTE,Cs,MUTE,D,D,MUTE,MUTE,A2,MUTE,Cs,MUTE,D,D,MUTE,D,D,MUTE,D,MUTE,Fs,MUTE,G,G,MUTE,G,G,MUTE,G,MUTE,A,MUTE,B,B,MUTE,B,B,MUTE,
												A,MUTE,G,MUTE,A,MUTE,D,D,MUTE,MUTE,D,MUTE,E,MUTE,Fs,Fs,MUTE,Fs,Fs,MUTE,G,G,MUTE,A,MUTE,D,D,MUTE,MUTE,D,MUTE,Fs,MUTE,E,E,MUTE,E,E,MUTE,
												Fs,MUTE,D,MUTE,E,E,MUTE,MUTE,A,MUTE,Cs,MUTE,D2,D2,MUTE,D2,D2,MUTE,D2,MUTE,E2,MUTE,F2s,F2s,MUTE,F2s,F2s,MUTE,F2s,MUTE,G2,MUTE,E2,E2,MUTE,
												E2,E2,MUTE,D2,MUTE,Cs,MUTE,Cs,MUTE,D2,D2,MUTE,A,MUTE,MUTE,Cs,MUTE,D2,D2,MUTE,D2,D2,MUTE,D2,MUTE,E2,MUTE,F2s,F2s,MUTE,F2s,F2s,MUTE,F2s,MUTE,G2,
												MUTE,E2,E2,MUTE,E2,E2,MUTE,D2,MUTE,Cs,MUTE,D2,D2,MUTE,A,MUTE,Cs,MUTE,D2,D2,MUTE,D2,D2,MUTE,D2,MUTE,F2s,MUTE,G2,G2,MUTE,G2,G2,MUTE,G2,MUTE,
												A,MUTE,B,B,MUTE,B,B,MUTE,A,MUTE,G2,MUTE,A2,MUTE,D2,D2,MUTE,MUTE,D2,MUTE,E2,MUTE,F2s,F2s,MUTE,F2s,F2s,MUTE,G2,G2,MUTE,A2,MUTE,D2,D2,MUTE,
												MUTE,D2,MUTE,F2s,MUTE,E2,E2,MUTE,E2,E2,MUTE,D2,MUTE,Cs,MUTE,D2,D2,MUTE,D2,D2,MUTE,E2,E2,MUTE,F2s,F2s,MUTE,F2s,MUTE,F2s,MUTE,G2,G2,MUTE,A2,
												A2,A2,A2,MUTE,F2s,MUTE,D2,MUTE,A,A,A,A,MUTE,B,B,B,B,MUTE,G2,MUTE,D2,MUTE,A,A,A,A,MUTE,D,D,MUTE,D,D,MUTE,Cs,Cs,Cs};
	
int mario[] = 	{ME ,ME ,MUTE,ME ,MUTE,ME ,MUTE,MC ,MUTE,ME ,MUTE,MG ,MUTE,MC ,MUTE,MME ,MUTE,MMB ,MUTE,
									ME ,MUTE,MG ,MUTE,MA ,MUTE,MF ,MUTE,MG ,MUTE,ME ,MUTE,MC ,MUTE,MD ,MUTE,MMB ,MUTE,MC ,MUTE,
									MME ,MUTE,MMB ,MUTE,ME ,MUTE,MG ,MUTE,MA ,MUTE,MF ,MUTE,MG ,MUTE,ME ,MUTE,MC ,MUTE,
									MD ,MUTE,MMB ,MUTE,MG ,MUTE,MF1 ,MUTE,MF ,MUTE,MD ,MUTE,ME,MUTE,MC ,MUTE,MC ,MUTE,MD ,MUTE,
									MG ,MUTE,MF1 ,MUTE,MF ,MUTE,MD ,MUTE,ME ,MUTE,MMC ,MUTE,MMC ,MUTE,MMC ,MUTE,MG ,MUTE,MF1 ,MUTE,MF ,MUTE,MD ,MUTE,
									ME ,MUTE,MC ,MUTE,MC ,MUTE,MD ,MUTE,MD1 ,MUTE,MD ,MUTE,MC ,MUTE,MC ,MUTE,MC ,MUTE,MC ,MUTE,
									MC ,MUTE,MD ,MUTE,ME ,MUTE,MC ,MUTE,MC ,MUTE,MC ,MUTE,MC ,MUTE,MC ,MUTE,MD ,MUTE,ME,MUTE,
									MC ,MUTE,MC ,MUTE,MC ,MUTE,MC ,MUTE,MD ,MUTE,ME ,MUTE,MC ,MUTE};

uint16_t j;

uint32_t seg[10] = {LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
									 LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15,
									 LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_2| LL_GPIO_PIN_15,
									 LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14| LL_GPIO_PIN_15,
									 LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11| LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
									 };
uint32_t digit[4] = {LL_GPIO_PIN_0,LL_GPIO_PIN_1,LL_GPIO_PIN_2,LL_GPIO_PIN_3};

uint16_t adc_data = 0;

int main(void)
{
  SystemClock_Config();
	TIM_OC_GPIO_Config();
	GPIO_Config();
	TIM_OC_Config(ARR_CALCULATE(MUTE));
	TIM_BASE_DurationConfig();
	
	
	
	int ij[4] = {0,0,0,0};
	int music = 0;
	
	while(1)
	{
		ADC1->CR2 |= (1<<30);
		while((ADC1->SR & (1<<1)) == 0);
		adc_data = ADC1->DR;

		LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_2 |LL_GPIO_PIN_10 |LL_GPIO_PIN_11 |LL_GPIO_PIN_12 |LL_GPIO_PIN_13 |LL_GPIO_PIN_14 |LL_GPIO_PIN_15);
					LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3);
				for(int j=0;j<=3;j++){
					
					LL_GPIO_SetOutputPin(GPIOC,digit[j]);
					LL_GPIO_SetOutputPin(GPIOB,seg[1]);
				}
		
		
		/// SPEAKER PART DONT TOUCH !
			for(int i=0;i<sizeof(mario)/4;)
			{
				if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET )
				{
					LL_TIM_SetAutoReload(TIM4,ARR_CALCULATE(mario[i]));
					LL_TIM_OC_SetCompareCH1(TIM4,LL_TIM_GetAutoReload(TIM4) / 2);
					if(mario[i]==MUTE){LL_TIM_SetAutoReload(TIM2,MUTE_ARR);}
					else if(mario[i+1]==MUTE){LL_TIM_SetAutoReload(TIM2,LAST_EIGHT_NOTE_ARR);}
					else{LL_TIM_SetAutoReload(TIM2,EIGHT_NOTE_ARR);}
					++i;
					LL_TIM_ClearFlag_UPDATE(TIM2);
					LL_TIM_SetCounter(TIM2, 0);
				}
			}
		/// END SPEAKER
		}
		

		
}


void GPIO_Config(void)
{
	LL_GPIO_InitTypeDef timic_gpio;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	timic_gpio.Mode = LL_GPIO_MODE_OUTPUT;
	timic_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	timic_gpio.Pin = LL_GPIO_PIN_2 |LL_GPIO_PIN_10 |LL_GPIO_PIN_11 |LL_GPIO_PIN_12 |LL_GPIO_PIN_13 |LL_GPIO_PIN_14 |LL_GPIO_PIN_15 ;
	timic_gpio.Pull = LL_GPIO_PULL_NO;
	timic_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &timic_gpio);
	
	timic_gpio.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC, &timic_gpio);
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	ADC1->CR1 |= (1<<11);
	ADC1->CR1 &= ~((7<<13)|(1<<24)|(1<<25));
	ADC1->CR2 &= ~(1<<11);
	ADC1->SMPR3 |= (2<<12);
	ADC1->SQR5 |= (5<<0);
	ADC1->CR2 |= (1<<0);
	
}

void TIM_BASE_DurationConfig(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 19 - 1;
	timbase_initstructure.Prescaler =  32000 - 1;
	LL_TIM_Init(TIM2, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM2); 
	LL_TIM_ClearFlag_UPDATE(TIM2); //Force clear update flag
}

void TIM_BASE_Config(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	LL_TIM_Init(TIM4, &timbase_initstructure);
	
}


void TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void TIM_OC_Config(uint16_t note)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_BASE_Config(note);
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 2;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	//NVIC_SetPriority(TIM4_IRQn, 1);
	//NVIC_EnableIRQ(TIM4_IRQn);
	//LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}

void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
	}
}
void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}