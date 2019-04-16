#ifndef __BXCAN_H__
#define __BXCAN_H__

#include <string.h>

struct __bxcan_state {
	CAN_TypeDef *hw;
};

typedef struct __bxcan_state bxcan_state_t;

int bxcan_init(bxcan_state_t *state, CAN_TypeDef *hw);
int bxcan_reconfigure(bxcan_state_t *state);

#endif /* __BXCAN_H__ */
