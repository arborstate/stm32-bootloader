
#include "stm32f334x8.h"

#include "system.h"
#include "hwutil.h"
#include "bxcan.h"

bxcan_state_t caniface;

volatile uint32_t tick = 0;

void
SysTick_Handler(void)
{
	tick += 1;

	return;
}

void
main(void)
{
	// Setup USART3
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
                // TX on PC10
		_gpio_afr(GPIOC, 10, 7);
		_gpio_moder(GPIOC, 10, 0x2);

		// RX on PC1
		_gpio_afr(GPIOC, 11, 7);
		_gpio_moder(GPIOC, 11, 0x2);

		// Enable the USART3 clock.
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		// Set the outgoing bit rate.
		USART3->BRR = clock_info.pclk1 / 38400;

		// Turn on the transmitter.
		// Turn on the USART.
		USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	}

	// Turn on systick.
	{
		SysTick->CTRL = 0;
		SysTick->VAL = 0;
		// Reload every 1ms.
		SysTick->LOAD = (clock_info.hclk / 1000) - 1;
		// Tie AHB/HCLK into SysTick, and turn on interrupts.
		SysTick->CTRL = 1 << SysTick_CTRL_ENABLE_Pos | 1 << SysTick_CTRL_TICKINT_Pos | 1 << SysTick_CTRL_CLKSOURCE_Pos;
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

	xmodem_receive(USART3);

	// Stay Here, Forever.
	while (1) {
	}
}
