#ifndef __CAN_H__
#define __CAN_H__

#include <string.h>

struct __can_state {
	CAN_TypeDef *hw;
};

typedef struct __can_state can_state_t;

int can_init(can_state_t *state, CAN_TypeDef *hw);
int can_reconfigure(can_state_t *state);

#endif /* __CAN_H__ */
