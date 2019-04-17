#ifndef __GPIO_H__
#define __GPIO_H__

#define _gpio_afr(gpio, pin, func)					\
	gpio->AFR[pin >> 3] =						\
		(gpio->AFR[pin >> 3] & ~(0xF << (4 * (pin % 8)))) |	\
		((func & 0xF) << (4 * (pin % 8)))

#define _gpio_moder(gpio, pin, mode) \
	GPIOC->MODER &= ~(0x3 << (pin * 2)); \
	GPIOC->MODER |= (mode << (pin * 2))

#endif /* __GPIO_H__ */
