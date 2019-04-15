#include "stm32f334x8.h"


void
main(void)
{
	// Turn on GPIOA
	RCC->AHBENR |= 1 << 17;

	// Turn on GPIOF
	RCC->AHBENR |= 1 << 22;

	// Set the MCO pull-up high to see we're alive.
	GPIOA->PUPDR = 1 << 16;

	// Turn on the HSE.
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY)) {}

	// Enable the MCO pin (PA8) AF0
	GPIOA->AFR[1] &= ~0xF;

	// Turn on Alternate Functions
	GPIOA->MODER |= 0x2 << 16;

	// Divide the output by 2.
	RCC->CFGR |= RCC_CFGR_MCOPRE_0;

	// Turn on MCO for the HSI.
	RCC->CFGR |= RCC_CFGR_MCO_HSE;

	// Stay Here, Forever.
	while (1) {
	}
}
