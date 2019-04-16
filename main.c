
#include "stm32f334x8.h"

#include "bxcan.h"

bxcan_state_t caniface;

void
main(void)
{
	// Turn on some important hardware.
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOFEN | RCC_AHBENR_CRCEN;

	// Set the MCO pull-up high to see we're alive.
	GPIOA->PUPDR = 1 << 16;

	// Turn on the HSE.
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY)) {}

	// Turn on the SYSCFG clock.
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Setup the CAN interface.
	{
		// Reset the CAN peripheral.
		RCC->APB1RSTR |= RCC_APB1RSTR_CANRST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;

		// Enable CAN clock.
		RCC->APB1ENR |= RCC_APB1ENR_CANEN;

		// Allow interrupts - XXX Check these priorities!
		NVIC_SetPriority(CAN_TX_IRQn, 5);
		NVIC_SetPriority(CAN_RX0_IRQn, 5);
		NVIC_SetPriority(CAN_RX1_IRQn, 5);
		NVIC_SetPriority(CAN_SCE_IRQn, 5);

		NVIC_EnableIRQ(CAN_TX_IRQn);
		NVIC_EnableIRQ(CAN_RX0_IRQn);
		NVIC_EnableIRQ(CAN_RX1_IRQn);
		NVIC_EnableIRQ(CAN_SCE_IRQn);

		bxcan_init(&caniface, CAN);
		bxcan_reconfigure(&caniface);
	}

	// Make sure we're not routing anything to the MCO.
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;

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
