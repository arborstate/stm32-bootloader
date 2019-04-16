#include "stm32f334x8.h"

#include "bxcan.h"

void
CAN_TX_IRQHandler(void)
{
}

void
CAN_RX0_IRQHandler(void)
{
}

void
CAN_SCE_IRQHandler(void)
{
}

int
bxcan_init(bxcan_state_t *state, CAN_TypeDef *hw)
{
	memset(state, 0, sizeof(bxcan_state_t));
	state->hw = hw;
}

int
bxcan_reconfigure(bxcan_state_t *state)
{
	// Make sure we're not asleep.
	state->hw->MCR &= ~CAN_MCR_SLEEP;

	// Go into init mode.
	state->hw->MCR |= CAN_MCR_INRQ;
	do {} while (!(state->hw->MCR & CAN_MSR_INAK));

	// Turn on silent loopback and 125k (per http://bittiming.can-wiki.info/)
	state->hw->BTR = CAN_BTR_LBKM | CAN_BTR_SILM | 0x1C003;

	// Get out of init mode.
	state->hw->MCR &= ~CAN_MCR_INRQ;
	do {} while (state->hw->MCR & CAN_MSR_INAK);

	return 0;
}
