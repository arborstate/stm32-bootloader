#ifndef __HWUTIL_H__
#define __HWUTIL_H__

#define _gpio_afr(gpio, pin, func)					\
	gpio->AFR[pin >> 3] =						\
		(gpio->AFR[pin >> 3] & ~(0xF << (4 * (pin % 8)))) |	\
		((func & 0xF) << (4 * (pin % 8)))

#define _gpio_moder(gpio, pin, mode) \
	gpio->MODER &= ~(0x3 << (pin * 2)); \
	gpio->MODER |= (mode << (pin * 2))

#endif /* __HWUTIL_H__ */
