#include "stm32f334x8.h"

#include "can.h"

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
can_init(can_state_t *state, CAN_TypeDef *hw)
{
	memset(state, 0, sizeof(can_state_t));
	state->hw = hw;
}

int can_reconfigure(can_state_t *state)
{
	// Make sure we're not asleep.
	state->hw->MCR &= ~CAN_MCR_SLEEP;

	// Go into init mode.
	state->hw->MCR |= CAN_MCR_INRQ;
	do {} while (!(state->hw->MCR & CAN_MSR_INAK));

	// Get out of init mode.
	state->hw->MCR &= ~CAN_MCR_INRQ;
	do {} while (state->hw->MCR & CAN_MSR_INAK);

	return 0;
}
