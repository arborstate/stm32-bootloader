
#include "stm32f334x8.h"

#include "system.h"
#include "hwutil.h"
#include "bxcan.h"
#include "xmodem.h"

const uint32_t *app_addr = (uint32_t *)0x08000800;

bxcan_state_t caniface;

volatile uint32_t tick = 0;

void
SysTick_Handler(void)
{
	tick += 1;

	return;
}

void
usart_send(USART_TypeDef *usart, char c)
{
	do {} while (!(usart->ISR & USART_ISR_TXE));
	usart->TDR = c;
}

int
_xmodem_flash(USART_TypeDef *usart)
{
	struct xmodem_state state;
	char resp;
	uint32_t timeout;

#define _RESET_TIMEOUT() timeout = tick + (3 * 1000)
	int count = 0;

	resp = xmodem_cancel(&state);

	// Force a timeout.
	timeout = 0;
	volatile uint16_t *addr;
	addr = (uint16_t *)app_addr;

	while (1) {
		if (usart->ISR & USART_ISR_RXNE) {
			_RESET_TIMEOUT();
			resp = xmodem_ingest(&state, usart->RDR);

			switch (state.state) {
			case XMODEM_STATE_CANCEL:
				// Start writing after the first page.
				addr = (uint16_t *)app_addr;
				break;

			case XMODEM_STATE_SOH:
				break;

			case XMODEM_STATE_EOP: {
				if (FLASH->CR & FLASH_CR_LOCK) {
					FLASH->KEYR = 0x45670123;
					FLASH->KEYR = 0xCDEF89AB;
				}


				// Erase each page at the 2k boundary.
				if (((uint32_t)addr % 2048) == 0) {
					// Wait while the flash module does its thing.
					do {} while (FLASH->SR & FLASH_SR_BSY);

					// Turn on flash page erase.
					FLASH->CR |= FLASH_CR_PER;

					FLASH->AR = (uint32_t)addr;

					// Kick off the erase.
					FLASH->CR |= FLASH_CR_STRT;

					// Wait while the flash module does its thing.
					do {} while (FLASH->SR & FLASH_SR_BSY);
					// ... per the datasheet, give the flash
					// controller a second chance to be sure.
					do {} while (FLASH->SR & FLASH_SR_BSY);

					// Check for EOP in FLASH SR, and clear bit.
					do {} while (!(FLASH->SR & FLASH_SR_EOP));
					FLASH->SR &= ~FLASH_SR_EOP;

					// Turn off flash page erase.
					FLASH->CR &= ~FLASH_CR_PER;
				}

				// XXX - Our xmodem packet is always 128 bytes.
				for (int i = 0; i < state.packetpos; i += 2) {
					uint16_t v = state.packet[i] | (state.packet[i + 1] << 8);

					// Wait while the flash module does its thing.
					do {} while (FLASH->SR & FLASH_SR_BSY);

					// Turn on flash reprogramming.
					FLASH->CR |= FLASH_CR_PG;

					// Clear any previous errors.
					FLASH->SR |= FLASH_SR_PGERR;

					*addr = v;

					// Wait for the write to finish.
					do {} while (FLASH->SR & FLASH_SR_BSY);

					// XXX - Disable this for now, since EOP
					// isn't being asserted anymore?  What?
#if 0
					// Check for EOP in FLASH SR, and clear bit.
					do {} while (!(FLASH->SR & FLASH_SR_EOP));
					FLASH->SR &= ~FLASH_SR_EOP;
#endif

					// Turn off flash reprogramming.
					FLASH->CR &= ~FLASH_CR_PG;

					if (FLASH->SR & FLASH_SR_PGERR) {
						resp = xmodem_cancel(&state);
						break;
					}

					if (*addr != v) {
						// We failed to write the value.  Bail.
						resp = xmodem_cancel(&state);
						break;
					}

					addr++;
				}

				// Wait for the write to finish.
				do {} while (FLASH->SR & FLASH_SR_BSY);

				break;
			}
			case XMODEM_STATE_EOT:
			case XMODEM_STATE_EOB:
				// Lock the flash back up.
				FLASH->CR |= FLASH_CR_LOCK;
				addr = (uint16_t *)app_addr;

				done = 1;
				break;
			default:
				break;
			}

			if (resp != 0) {
				usart_send(usart, resp);
			}
		}

		if (tick >= timeout) {
			_RESET_TIMEOUT();
			if (resp != 0) {
				usart_send(usart, resp);
			}
			count += 1;

			if (count >= 3) {
				resp = xmodem_cancel(&state);
				addr = (uint16_t *)app_addr;

				count = 0;
			}
		}

	}
#undef _RESET_TIMEOUT
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

	// test_reflash();

	_xmodem_flash(USART3);

	// Stay Here, Forever.
	while (1) {
	}
}


void
test_reflash(void)
{
	// We should be entering this with the flash locked.
	if (!(FLASH->CR & FLASH_CR_LOCK)) {
		// We're not locked. Something is wrong.
		return;
	}

	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	if (FLASH->CR & FLASH_CR_LOCK) {
		// If we think we're still locked, something went wrong.
		return;
	}

	// Page Erase
	{
		FLASH->CR |= FLASH_CR_PER;

		// Let's play with the last page.
		FLASH->AR = 0x0800F800;

		// Kick off the erase.
		FLASH->CR |= FLASH_CR_STRT;

		// Wait while the flash module does its thing.
		do {} while (FLASH->SR & FLASH_SR_BSY);

		// Check for EOP in FLASH SR, and clear bit.
		do {} while (!(FLASH->SR & FLASH_SR_EOP));

		FLASH->SR &= ~FLASH_SR_EOP;
	}

	// Page Write
	{
		FLASH->CR |= FLASH_CR_PG;
		const char *msg = "HEY YOU GUYS!!";
		uint16_t v;
		uint16_t *addr = (uint16_t *)0x0800F800;

		for (int i = 0; i < ((strlen(msg) + 1) / 2); i++) {
			v = msg[i * 2] | (msg[(i * 2) + 1] << 8);
			*addr = v;
			// Wait for the write to finish.
			do {} while (FLASH->SR & FLASH_SR_BSY);

			if (*addr != v) {
				// We failed to write the value.  Bail.
				goto lockit;
			}

			addr++;
		}
	}

lockit:
	// Lock flash memory by writing LOCK to FLASH_CR
	FLASH->CR |= FLASH_CR_LOCK;
}
