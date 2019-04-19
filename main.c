
#include "stm32f334x8.h"

#include "hwutil.h"
#include "bxcan.h"

bxcan_state_t caniface;


#define _SET_REG(reg, field, val) reg = (reg & ~(field ## _Msk)) | (val << (field ## _Pos))

void
SysTick_Handler(void)
{
	do {} while (!(USART3->ISR & USART_ISR_TXE));
	USART3->TDR = '>';

	return;
}


struct {
	uint32_t hclk;
	uint32_t pclk1;
	uint32_t pclk2;
} clock_info;

void
main(void)
{
	// Start assuming the HSI is on.
	{
		clock_info.hclk = 8000000U;
		clock_info.pclk1 = 8000000U;
		clock_info.pclk2 = 8000000U;
	}

	// Enable the HSE.
	{
		// Turn on the HSE.
		RCC->CR |= RCC_CR_HSEON;
		while (!(RCC->CR & RCC_CR_HSERDY)) {}

		// Turn on the SYSCFG clock.
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

		// Turn off the PLL.
		RCC->CR &= ~RCC_CR_PLLON;

		// Attach PLL to HSE * 9 = 72 MHz.
		_SET_REG(RCC->CFGR, RCC_CFGR_PLLSRC, 1); // PLL fed HSE
		_SET_REG(RCC->CFGR2, RCC_CFGR2_PREDIV, 0); // With no scaling.
		_SET_REG(RCC->CFGR, RCC_CFGR_PLLMUL, 7); // Multiplied by 9.

		// HCLK = SYSCLK
		_SET_REG(RCC->CFGR, RCC_CFGR_HPRE, 0);
		clock_info.hclk = 72000000U;

		// Configure The Peripheral PCLK/fClk
		_SET_REG(RCC->CFGR, RCC_CFGR_PPRE1, 0); // APB1 = HCLK
		clock_info.pclk1 = clock_info.hclk;
		_SET_REG(RCC->CFGR, RCC_CFGR_PPRE2, 0); // APB2 = HCLK
		clock_info.pclk2 = clock_info.hclk;

		// Turn on the PLL.
		RCC->CR |= RCC_CR_PLLON;
		do {} while(!(RCC->CR & RCC_CR_PLLRDY));

		// Adjust the flash wait states and pre-fetch buffer
		// before we boost up the SYSCLK frequency.
		_SET_REG(FLASH->ACR, FLASH_ACR_HLFCYA, 0);
		_SET_REG(FLASH->ACR, FLASH_ACR_PRFTBE, 1);
		_SET_REG(FLASH->ACR, FLASH_ACR_LATENCY, 2);

		// Wait for the prefetch buffer to be online.
		do {} while (!(FLASH->ACR & FLASH_ACR_PRFTBS));

		// Source SYSCLK from the PLL.
		_SET_REG(RCC->CFGR, RCC_CFGR_SW, 2);
	}

#ifdef _OUTPUT_MCO
	// Turn on the MCO so we can measure the clock.
	{
		_gpio_moder(GPIOA, 8, 2);
		// Set the MCO pull-up high to see we're alive.
		GPIOA->PUPDR = 1 << 16;
		_SET_REG(RCC->CFGR, RCC_CFGR_MCO, 0);

		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		// Enable the MCO pin (PA8) AF0
		_gpio_afr(GPIOA, 8, 0);

		// Don't divide the PLL.
		_SET_REG(RCC->CFGR, RCC_CFGR_PLLNODIV, 1);

		// Divide the MCO output by 2.
		_SET_REG(RCC->CFGR, RCC_CFGR_MCOPRE, 1);

		// Route the SYSCLK to the MCO.
		_SET_REG(RCC->CFGR, RCC_CFGR_MCO, 4);
	}
#endif

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
		USART3->CR1 |= USART_CR1_UE | USART_CR1_TE;

		const char *_msg = "hey you guys!\r\n";

		for (int i = 0; i < 15; i++) {
			do {} while (!(USART3->ISR & USART_ISR_TXE));
			USART3->TDR = _msg[i];
		}
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

	// Stay Here, Forever.
	while (1) {
	}
}
