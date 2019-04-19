#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdint.h>

#define _SET_REG(reg, field, val) reg = (reg & ~(field ## _Msk)) | (val << (field ## _Pos))

struct __clock_info {
	uint32_t pllclk;
	uint32_t sysclk;
	uint32_t hclk;
	uint32_t pclk1;
	uint32_t pclk2;
};

typedef struct __clock_info clock_info_t;

extern clock_info_t clock_info;
extern uint32_t SystemCoreClock;

#endif /* __SYSTEM_H__ */
