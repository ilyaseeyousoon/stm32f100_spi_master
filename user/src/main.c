#include <stm32f10x.h>
#include <stm32f10x_conf.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_spi.h"

#include "main.h"
 uint32_t  i,m=0;
 uint16_t  ams=0;
 uint16_t  adc1=0;
 uint16_t dt[100];
uint32_t delay_count=0;
uint16_t adc=0;
uint8_t Receive_buf[256];
uint8_t Receive_W=0,Receive_R=0,Receive_C=0;
uint32_t tetra=0;
 

void set_7led(int x);
void SysTick_Handler(void)   
{
if (delay_count>0)
{
delay_count--;	
}
}

void delay_ms(uint32_t delay_temp)
{
delay_count = delay_temp;
	while(delay_count){}
}

void Delay(volatile uint32_t nCount) 
	{
    for (; nCount != 0; nCount--);
}
void send_to_uart(uint8_t data) 
	{

while(!(USART1->SR & USART_SR_TC)); 

USART1->DR=data; 

}
	void  RCC_Configuration(void)
	{
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
		RCC_PLLCmd(DISABLE);
		 /*RCC system reset(for debug purpose) */
    RCC_DeInit();
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
   
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK*/
        RCC_PCLK1Config(RCC_HCLK_Div1);

        //ADC CLK
        RCC_ADCCLKConfig(RCC_PCLK2_Div2);

        /* PLLCLK = 8MHz * 3 = 24 MHz */
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_6);



        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08) {}
    

    /*Then need to enable peripheral clocks ----------------------------------------------*/
		
		
		
		
		
		
		
		
		
		
		
		//////////////////
		

//		  char PLL_MUL=12; // ??????????? ????????? PLL.
//   RCC->CFGR2 &=~(RCC_CFGR2_PREDIV1); // ??????????? ???????? HSE.
//   RCC->CFGR2|= RCC_CFGR2_PREDIV1_DIV4; // ?????? ??????? HSE ?? 4.
//   RCC->CFGR &=~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); // ???????????.
//   RCC->CFGR |= RCC_CFGR_PLLSRC_PREDIV1; // ??????????? PLL ?? HSE/PREDIV1.
//   RCC->CFGR|=((PLL_MUL - 4)<<18); // ???????? ??????? ?? PLL_MUL.
//   RCC->CR |= RCC_CR_PLLON; // ????????? PLL.
//   while ((RCC->CR & RCC_CR_PLLRDY)==0) {} // ???????? ?????????? PLL.
//   RCC->CFGR &=~RCC_CFGR_SW; // ???????? ???? SW0, SW1.
//   RCC->CFGR |= RCC_CFGR_SW_PLL; // ???????????? ? ?????? PLL.
//   while ((RCC->CFGR&RCC_CFGR_SWS)!=0x08) {} // ???????? ???????????? ?? PLL.
		
		
		
		
		
		
////////////////////////
		
		
		
		
//		    RCC->CFGR &=~RCC_CFGR_SW;
//	
//	   RCC->CR &=~ RCC_CR_PLLON;
//     char PLL_MUL=10; 
//   RCC->CFGR2 &=~(RCC_CFGR2_PREDIV1); 
//   RCC->CFGR &=~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); 
//		RCC->CFGR2|= RCC_CFGR2_PREDIV1_DIV2;
//   RCC->CFGR |= RCC_CFGR_PLLSRC_PREDIV1;
//   RCC->CFGR|=((PLL_MUL  )<<12);
//  RCC->CR |= RCC_CR_PLLON;
//   while ((RCC->CR & RCC_CR_PLLRDY)==0) {} 
//   RCC->CFGR &=~RCC_CFGR_SW; 
//   RCC->CFGR |= RCC_CFGR_SW_PLL;
//   while ((RCC->CFGR&RCC_CFGR_SWS)!=0x08) {} 
	}



void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
	
	
		Receive_buf[Receive_W]= USART_ReceiveData(USART1);
		Receive_W++;
		Receive_C++;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
	
}

void init_button()
{
    RCC->APB2ENR |= RCC_APB2Periph_GPIOA;
    GPIOA->CRL |= 0x04;
    GPIOA->CRL &= ~0xb;
}

void adc_init() 
	{
    ////////////////
//????? SPI1 (Master)
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC  , ENABLE); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_8);//led
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC , &GPIO_InitStructure);
 
		
		GPIO_InitStructure.GPIO_Pin = (  GPIO_Pin_7 );// забыл 7ю ногу настроить теперь все раьботает как часы
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
  GPIO_InitStructure.GPIO_Pin = (  GPIO_Pin_6 );
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ( GPIO_Pin_5  );
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_4  );
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

      
  /////////////////////////////////////    
	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB2ENR_AFIOEN, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);		
////////////////////////////////////////////				
GPIO_InitTypeDef    GPIO_InitStruct;

  //????????????? ?????? PA9 - USART1_Tx
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; //????????? ?????? PA9
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; //???????? ????? ????????????
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; //????? ?????????????? ???????, ????? Push-Pull
  GPIO_Init(GPIOA, &GPIO_InitStruct); //???????? ????????? ????????? ? ????????? GPIO?

  //????????????? ?????? PA10 - USART1_Rx
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; //????????? ?????? PA10
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING; 
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Input floating
  GPIO_Init(GPIOA, &GPIO_InitStruct); //???????? ????????? ????????? ? ????????? GPIO?
USART_InitTypeDef    USART_InitStruct;
  //????????????? USART1
  USART_InitStruct.USART_BaudRate = 9600; //???????? ?????? 9600 ???
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; //????? ????? 8 ???
  USART_InitStruct.USART_StopBits = USART_StopBits_1; //1 ????-???
  USART_InitStruct.USART_Parity = USART_Parity_No ; //??? ???????? ????????
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //??? ??????????? ????????
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //??????? ?????????? ? ???????? USART1
  USART_Init(USART1, &USART_InitStruct); //???????? ????????? ????????? ? ????????? USART1


////////////////////////////////////////////

  USART1->CR1 |= USART_CR1_TE; 
	USART1->CR1 |= USART_CR1_RE; 
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);
	
	
	//USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); 
	
	
	USART_Cmd(USART1,ENABLE);
	
	SPI_InitTypeDef SPI_InitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
SPI_Init(SPI1, &SPI_InitStructure);
 SPI_Cmd(SPI1, ENABLE);
	

}
	


int main(void)
{
	 RCC_Configuration();
	adc_init();

	init_button();
	SysTick_Config(SystemCoreClock/200000);//10 mks
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
GPIO_SetBits(GPIOA,GPIO_Pin_6);
//SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
	delay_ms(100000);	delay_ms(100000);	delay_ms(100000);	delay_ms(100000);	delay_ms(100000);	delay_ms(100000);	delay_ms(100000);
uint16_t data[15] ;
	data[0]=0x01;
	data[1]=0x02;
	data[2]=0x03;
	data[3]=0x04;
	data[4]=0x05;
	data[5]=0x06;
	data[6]=0x07;
	data[7]=0x08;
	data[8]=0x09;
	data[9]=0x10;
	data[10]=0x11;
	data[11]=0x12;
	data[12]=0x13;
	data[13]=0x14;
	data[14]=0x15;

	uint16_t u=0;
 while(1)
    {
			if (u<15)
			{
for(i=0;i<1000000;i++){}
		delay_ms(10000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		delay_ms(15);
		 SPI_I2S_SendData(SPI1, data[u]);  //1 bait	
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)	
		;
		if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET)
		{ 
		dt[m] =  SPI_I2S_ReceiveData(SPI1) 
			;
				delay_ms(10);
					GPIO_SetBits(GPIOA,GPIO_Pin_4);
		m=m+1;
    u=u+1;
		}
	    }
			else
			{
			delay_ms(10000);delay_ms(10000);delay_ms(10000);delay_ms(10000);delay_ms(10000);delay_ms(10000);delay_ms(10000);delay_ms(10000);
				u=0;
			
			}
		    }	
	        }
	
      