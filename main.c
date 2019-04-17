
#include "stm32f334x8.h"

#include "hwutil.h"
#include "bxcan.h"

bxcan_state_t caniface;

#define _FCK 8000000U

#define _SET_REG(reg, field, val) reg = (reg & ~(field ## _Msk)) | (val << (field ## _Pos))

void
main(void)
{

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

		// Turn off the PLL.
		RCC->CR &= ~RCC_CR_PLLON;

		// Attach PLL to (HSE/2) * 9 and HCLK = SYSCLK / 1
		_SET_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, 1);
		_SET_REG(RCC->CFGR, RCC_CFGR_PLLMUL, 7);
		_SET_REG(RCC->CFGR, RCC_CFGR_PLLSRC, 1);

		RCC->CR |= RCC_CR_PLLON;
		do {} while(!(RCC->CR & RCC_CR_PLLRDY));

		_SET_REG(RCC->CFGR, RCC_CFGR_SW, 1);

	}

	// Setup USART3
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
                // TX on PC10
		_gpio_afr(GPIOC, 10, 7);
		_gpio_moder(GPIOC, 10, 0x2);

		// RX on PC11
		_gpio_afr(GPIOC, 11, 7);
		_gpio_moder(GPIOC, 11, 0x2);

		// Enable the USART3 clock.
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		// Set the outgoing bit rate.
		USART3->BRR = _FCK / 38400;

		// Turn on the transmitter.
		// Turn on the USART.
		USART3->CR1 |= USART_CR1_UE | USART_CR1_TE;

		const char *_msg = "hey you guys! ";

		while (1) {
			for (int i = 0; i < 14; i++) {
				do {} while (!(USART3->ISR & USART_ISR_TXE));
				USART3->TDR = _msg[i];
			}
		}
	}

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

	// Turn on MCO for the HSI.
	RCC->CFGR |= RCC_CFGR_MCO_PLL;

	// Stay Here, Forever.
	while (1) {
	}
}
