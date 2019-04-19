#include "system.h"
#include "stm32f3xx.h"

#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x0
#endif

// Keep this to be compatible with the rest of CMSIS-and-friends.
uint32_t SystemCoreClock;
clock_info_t clock_info;

void system_init(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	// Turn on CP10 and CP11
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif
	// Disable all interrupts.
	RCC->CIR = 0U;

	// Start assuming the HSI is on.
	{
		clock_info.sysclk = 8000000U;
		clock_info.hclk = 8000000U;
		SystemCoreClock = clock_info.hclk;
		clock_info.pclk1 = 8000000U;
		clock_info.pclk2 = 8000000U;
		clock_info.pllclk = 0U;
	}

	// Turn the HSI is on.
	_SET_REG(RCC->CR, RCC_CR_HSION, 1);

	// Wait until it's ready.
	do {} while (!(RCC->CR & RCC_CR_HSIRDY));

	// Configure the SysClk to run off of the HSI.
	_SET_REG(RCC->CFGR, RCC_CFGR_SW, 0);
	do {} while ((RCC->CR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

	// Turn off the PLL.
	_SET_REG(RCC->CR, RCC_CR_PLLON, 0);
	// Turn off the clock security system.
	_SET_REG(RCC->CR, RCC_CR_CSSON, 0);
	// Turn off the HSE.
	_SET_REG(RCC->CR, RCC_CR_HSEON, 0);
	// Turn off the HSE bypass.
	_SET_REG(RCC->CR, RCC_CR_HSEBYP, 0);

	// Turn off any peripheral clock, PLL, and MCO config.
	RCC->CFGR = 0U;
	RCC->CFGR2 = 0U;
	RCC->CFGR3 = 0U;

#ifdef _FAST_AND_THE_FURIOUS
	// Enable the 8 MHz HSE with the PLL at 72 MHz.
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
		clock_info.pllclk = 72000000U;

		// We're going to use the PLL as the SYSCLK.
		clock_info.sysclk = clock_info.pllclk;

		// HCLK (AHB) = SYSCLK
		_SET_REG(RCC->CFGR, RCC_CFGR_HPRE, 0);
		clock_info.hclk = clock_info.sysclk;
		SystemCoreClock = clock_info.hclk;

		// PCLK1 (APB1) - Slow Peripheral Clock
		// Note: The datasheet caps the speed of the PCLK1 at 36 MHz.
		_SET_REG(RCC->CFGR, RCC_CFGR_PPRE1, 4); // PCLK1 = HCLK / 2
		clock_info.pclk1 = clock_info.hclk / 2;

		// PCLK2 (APB2) - Fast Peripheral Clock
		// Note: This clock can run at 72 MHz.
		_SET_REG(RCC->CFGR, RCC_CFGR_PPRE2, 0); // PCLK2 = HCLK
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
		do {} while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	}
#endif

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

#ifdef VECT_TAB_SRAM
	// Vectors From RAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;
#else
	// Vectors From Flash
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
#endif
}
