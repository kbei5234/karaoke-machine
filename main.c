#include "stm32l476xx.h"
//#include "SysClock.h"
#include "UART.h"
//#include "SysTick.h"
#include "lcd.h"
#include "music.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//#include <time.h>

//Define joystick buttons as their IDR pins
#define center 0x1
#define up 0x2
#define down 0x4

uint8_t buffer[BufferSize];
//uint8_t buffer1[BufferSize];

void movingString(uint8_t* str, uint8_t len);

extern uint64_t ms;

void hang();
void DAC_System_Clock_Init(void);
void UART_System_Clock_Init(void);
void DAC_Init(void);
void TIM4_Init(void);
void TIM4_IRQHandler(void);
uint16_t sine(uint16_t angle_in_degrees);
void SysTick_Initialize(int ticks);
void delay(int ms);
void playsong(struct song song);
void Joystick_Initialization(void);

volatile int TimeDelay;		//Global variable for delay

double v=0;					//Global variables for DAC
double angle=0;
int volume=5;

int main(void){
	int n;
	int r;
	
	//srand(time(0));

	UART_System_Clock_Init(); // Switch System Clock = 80 MHz
	UART2_Init();
	
	n = sprintf((char *)buffer, "   _____      __          __ \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  / ___/___  / /__  _____/ /_\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  \\__ \\/ _ \\/ / _ \\/ ___/ __/\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, " ___/ /  __/ /  __/ /__/ /_  \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "/____/\\___/_/\\___/\\___/\\__/  \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "   /   |                     \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  / /| |                     \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, " / ___ |                     \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "/_/__|_|                     \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  / ___/____  ____  ____ _   \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  \\__ \\/ __ \\/ __ \\/ __ `/   \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, " ___/ / /_/ / / / / /_/ /    \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "/____/\\____/_/ /_/\\__, /     \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "                 /____/      \r\n");
	USART_Write(USART2, buffer, n);
	
	n = sprintf((char *)buffer, "\n\n  Call Me Maybe\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  +-----------+\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  by Carly Rae Jepsen\r\n");
	USART_Write(USART2, buffer, n);
	
	n = sprintf((char *)buffer, "\n\n  What Makes You Beautiful\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  +----------------------+\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  by One Direction\r\n");
	USART_Write(USART2, buffer, n);
	
	n = sprintf((char *)buffer, "\n\n  Teenage Dream\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  +-----------+\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  by Katy Perry\r\n");
	USART_Write(USART2, buffer, n);
	
	n = sprintf((char *)buffer, "\n\n  EXPERT MODE\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  +---------+\r\n");
	USART_Write(USART2, buffer, n);

	n = sprintf((char *)buffer, "\n            ,   ,0     ,\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "            |)  |)   ,'|\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  ____     0'   '   | 0'\r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, "  |  |             0'   \r\n");
	USART_Write(USART2, buffer, n);
	n = sprintf((char *)buffer, " 0' 0\r\n");
	USART_Write(USART2, buffer, n);


	int mode=0;

	//Initializations:
	//System_Clock_Init();
	DAC_Init();
	TIM4_Init();
	LCD_Initialization();
	SysTick_Initialize(999);
	Joystick_Initialization();
	
	while(1){
		if(GPIOA->IDR & center){
			switch(mode){
				case 0:
					n = sprintf((char *)buffer, "\n-------------\r\n");
					USART_Write(USART2, buffer, n);
					//free(n);
					n = sprintf((char *)buffer, "Call Me Maybe\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "-------------\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Hey, I just met you\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "And this is crazy\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "But here's my number\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "So call me maybe\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "It's hard to look right\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "At you baby\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "But here's my number\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "So call me maybe\r\n");
					USART_Write(USART2, buffer, n);
//					r = rand() % 100;
//					n = sprintf((char *)buffer, "\nMusic Taste: %d\r\n", r);
//					USART_Write(USART2, buffer, n);
				DAC_System_Clock_Init(); // initialize DAC clock
				playsong(maybe);
				UART_System_Clock_Init(); // initialize UART clock
					break;
				case 1:
					n = sprintf((char *)buffer, "\n-------------\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Teenage Dream\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "-------------\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "You make me\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Feel like I'm living a\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Teenage dream\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "The way you turn me on\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "I can't sleep\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Let's run away and\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Don't ever look back\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Don't ever look back\r\n");
					USART_Write(USART2, buffer, n);
					DAC_System_Clock_Init();
					playsong(dream);
					UART_System_Clock_Init();
					break;
				case 2:
					n = sprintf((char *)buffer, "\n------------------------\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "What Makes You Beautiful\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "------------------------\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Baby, you light up my world\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Like nobody else\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "The way that you flip your\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "hair gets me overwhelmed\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "But when you smile at the\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "ground, it ain't hard to tell\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "You don't know, oh, oh\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "You don't know you're\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "beautiful\r\n");
					USART_Write(USART2, buffer, n);
					DAC_System_Clock_Init();
					playsong(beautiful);
					UART_System_Clock_Init();
					break;
				case 3:
					n = sprintf((char *)buffer, "\n------\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "Expert\r\n");
					USART_Write(USART2, buffer, n);
					n = sprintf((char *)buffer, "------\r\n");
					USART_Write(USART2, buffer, n);
					DAC_System_Clock_Init();
					playsong(lovestory);
					UART_System_Clock_Init();
					break;
			}

			while(GPIOA->IDR & center);

		}
		if(GPIOA->IDR & down){
			mode--;
			while(GPIOA->IDR & down);
		}
		if(GPIOA->IDR & up){
			mode++;
			while(GPIOA->IDR & up);
		}
		
		if(mode>3) mode=0;
		if(mode<0) mode=3;
		
		switch(mode){
			case 0:
				LCD_DisplayString((uint8_t*)"MAYBE ");
				break;
			case 1:
				LCD_DisplayString((uint8_t*)"DREAM ");
				break;
			case 2:
				LCD_DisplayString((uint8_t*)"BEAUTY");
				break;
			case 3:
				LCD_DisplayString((uint8_t*)"EXPERT");
				break;
		}
	}
}


void DAC_System_Clock_Init(void){ // initialize to 16MHz
	// Enable High Speed Internal Clock (HSI = 16 MHz)
  RCC->CR |= ((uint32_t)RCC_CR_HSION);
	
  // wait until HSI is ready
  while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 );
	
  // Select HSI as system clock source 
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;  //01: HSI16 oscillator used as system clock

  // Wait till HSI is used as system clock source 
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) == 0 );
}

void UART_System_Clock_Init(void) { // initialize to 80MHz
	uint32_t HSITrim;

	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
  // must be correctly programmed according to the frequency of the CPU clock
  // (HCLK) and the supply voltage of the device.		
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
		
	// Enable the Internal High Speed oscillator (HSI
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	// Adjusts the Internal High Speed oscillator (HSI) calibration value
	// RC oscillator frequencies are factory calibrated by ST for 1 % accuracy at 25oC
	// After reset, the factory calibration value is loaded in HSICAL[7:0] of RCC_ICSCR	
	HSITrim = 16; // user-programmable trimming value that is added to HSICAL[7:0] in ICSCR.
	RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;
	RCC->ICSCR |= HSITrim << 24;
	
	RCC->CR    &= ~RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);
	
	// Select clock source to PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // 00 = No clock, 01 = MSI, 10 = HSI, 11 = HSE
	
	// Make PLL as 80 MHz
	// f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 20/2 = 160 MHz
	// f(PLL_R) = f(VCO clock) / PLLR = 160MHz/2 = 80MHz
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 20U << 8;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 1U << 4; // 000: PLLM = 1, 001: PLLM = 2, 010: PLLM = 3, 011: PLLM = 4, 100: PLLM = 5, 101: PLLM = 6, 110: PLLM = 7, 111: PLLM = 8

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;  // 00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable Main PLL PLLCLK output 

	RCC->CR   |= RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY) == 0);
	
	// Select PLL selected as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // 00: MSI, 01:HSI, 10: HSE, 11: PLL
	
	// Wait until System Clock has been selected
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	
	// The maximum frequency of the AHB, the APB1 and the APB2 domains is 80 MHz.
	RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1; SYSCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB high-speed prescaler (APB1) = 1, HCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB high-speed prescaler (APB2) = 1, HCLK not divided
	
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; 
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;	
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN; // Enable Main PLL PLLSAI3CLK output enable
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN; // Enable Main PLL PLL48M1CLK output enable
	
	RCC->CR &= ~RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == RCC_CR_PLLSAI1ON );
	
	// Configure and enable PLLSAI1 clock to generate 11.294MHz 
	// 8 MHz * 24 / 17 = 11.294MHz
	// f(VCOSAI1 clock) = f(PLL clock input) *  (PLLSAI1N / PLLM)
	// PLLSAI1CLK: f(PLLSAI1_P) = f(VCOSAI1 clock) / PLLSAI1P
	// PLLUSB2CLK: f(PLLSAI1_Q) = f(VCOSAI1 clock) / PLLSAI1Q
	// PLLADC1CLK: f(PLLSAI1_R) = f(VCOSAI1 clock) / PLLSAI1R
	RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1N;
	RCC->PLLSAI1CFGR |= 24U<<8;
	
	// SAI1PLL division factor for PLLSAI1CLK
	// 0: PLLSAI1P = 7, 1: PLLSAI1P = 17
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1P;
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;
	
	// SAI1PLL division factor for PLL48M2CLK (48 MHz clock)
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1Q;
	// RCC->PLLSAI1CFGR |= U<<21;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;
	
	// PLLSAI1 division factor for PLLADC1CLK (ADC clock)
	// 00: PLLSAI1R = 2, 01: PLLSAI1R = 4, 10: PLLSAI1R = 6, 11: PLLSAI1R = 8
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1R; 
	// RCC->PLLSAI1CFGR |= U<<25;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
	
	RCC->CR |= RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == 0);
	
	// SAI1 clock source selection
	// 00: PLLSAI1 "P" clock (PLLSAI1CLK) selected as SAI1 clock
	// 01: PLLSAI2 "P" clock (PLLSAI2CLK) selected as SAI1 clock
	// 10: PLL "P" clock (PLLSAI3CLK) selected as SAI1 clock
	// 11: External input SAI1_EXTCLK selected as SAI1 clock	
	RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;

	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
}

void DAC_Init(void){
	//Enable clock to GPIO Port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	//Set PA5 as analog
	GPIOA->MODER |= GPIO_MODER_MODER2;
	
	//Configure TIM4 as Master Trigger
	TIM4_Init();
	
	//Enable DAC clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;
	
	//Enable DAC output buffer
	DAC->MCR &= ~DAC_MCR_MODE2;
	//DAC->MCR |= DAC_MCR_MODE2_0;
	
	//Select TIM4 TRGO as trigger for output
	DAC->CR &= ~DAC_CR_TSEL2;
	DAC->CR |= DAC_CR_TSEL2_2;
	DAC->CR |= DAC_CR_TSEL2_0;
	
	DAC->CR |= DAC_CR_TEN2;
	
	//Enable DAC2
	DAC->CR |= DAC_CR_EN2;
	
}

void TIM4_Init(void){
	//Enable TIM4 Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	
	//Edge-aligned mode
	TIM4->CR1 &= ~TIM_CR1_CMS;
	
	//Counting direction: Up counting
	TIM4->CR1 &= ~TIM_CR1_DIR;
	
	//Master mode selection
	TIM4->CR2 &= ~TIM_CR2_MMS;	// Master mode selection
	TIM4->CR2 |= TIM_CR2_MMS_2;	// 100 = OC1REF as TRG0
	
	//Trigger interrupt enable
	TIM4->DIER |= TIM_DIER_TIE;
	
	//Update interrupt enable
	TIM4->DIER |= TIM_DIER_UIE;
	
	//OC1M: Output Compare 1 mode
	TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;		// 0110 = PWM mode 1
	
	//The counter clock frequency (CK_CNT) = fCK_PSC / (PSC[15:0] +1)
	//Timer driving frequency = 16 MHz/(1 + PSC) = 16 MHz/(1+9) = 1.6 MHz
	TIM4->PSC = 9;		//max 65535
	
	//Trigger frequency = 16 MHz/ (1 + ARR) = 16 MHz/100 = 16 KHz
	TIM4->ARR = 99;		// max 65535
	
	TIM4->CCR1 = 50;		// Duty ratio 50%
	
	TIM4->CCER |= TIM_CCER_CC1E;	//OC1 signal is output on the corresponding output pin
	
	//Enable timer
	TIM4->CR1 |= TIM_CR1_CEN;
	
	//Set priority to 1
	NVIC_SetPriority(TIM4_IRQn, 1);

	//Enable TIM4 interrupt in NVIC
	NVIC_EnableIRQ(TIM4_IRQn);

}

void TIM4_IRQHandler(void){
	if((TIM4->SR & TIM_SR_CC1IF) != 0){
	//Calculate sine of v and multiply by volume
		double result = sine(v)*volume;
		//Sent result to DAC
		DAC->DHR12R2 = (int)result;
		//Increment v by desired angle	
		v+=angle;
		//If v reaches 360 degress, reset to 0		
		if(v >= 360) v = 0;
		//Clear flag		
		TIM4->SR &= ~TIM_SR_CC1IF;
	}
	//Clear flag
	TIM4->SR &= ~TIM_SR_UIF;
}

void SysTick_Handler(void){
	if(TimeDelay>0)TimeDelay--;
}

void SysTick_Initialize(int ticks){
	//Turn on MSI clock
	RCC->CR |= RCC_CR_MSIRGSEL;
	RCC->CR &= ~RCC_CR_MSIRANGE;
	RCC->CR |= RCC_CR_MSIRANGE_7;
	
	RCC->CR |= RCC_CR_MSION;
	while(!(RCC->CR & RCC_CR_MSIRDY));

	//Set status and control register
	SysTick->CTRL = 0;
	//Set reload value register
	SysTick->LOAD |= ticks-1;
	//Set interrupt priority
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);
	//Reset counter value
	SysTick->VAL=0;
	//Select processor clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Pos;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Pos;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Pos;
	
}

void delay(int ms){
	//Wait for number of ms
	TimeDelay= ms;
	while(TimeDelay!=0);
}


void playsong(struct song song){
	int counter;
	for(counter=0; counter<song.notes; counter++){
		volume=song.play[counter].volume;
		angle = song.play[counter].angle;
		delay(song.play[counter].duration);
		
		volume=0;
		angle=0;
		delay(10);
	}
}

void Joystick_Initialization(void){
	// Enable clock for GPIOA, then configure appropriate pins PA0, PA1, and PA2 for joystick
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(0x3F); // cc3
	GPIOA->PUPDR &= ~(0x3F); // cc3
	GPIOA->PUPDR |= 0x2A; // 882
	
}
