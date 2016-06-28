/*
 * gpio_ext.c
 *
 */

#include "gpio_ext.h"
#include "com_task.h"
#include "errno.h"


static inline int port_type_to_int(PORT_Type *port)
{
	switch ((int) port) {
	case PORTA_BASE:
		return 0;
	case PORTB_BASE:
		return 1;
	case PORTC_BASE:
		return 2;
	case PORTD_BASE:
		return 3;
	case PORTE_BASE:
		return 4;
	default:
		return -EINVAL;
	}
}

/* returns GPIO index in gpio_list table and -EINVAL
 * if there is no entry for this gpio.
 */
int is_gpio_valid(uint8_t pin)
{
	uint16_t i;
	int temp;
	if (pin == 0xFF)
		return -EINVAL;

	for (i = 0; i < sizeof(gpio_list)/sizeof(struct gpio_id); i++){
		temp = port_type_to_int(gpio_list[i].port) * 32;
		temp += gpio_list[i].pin;
		if ( temp == pin )
			return i;
	}

	return -EINVAL;
}

int set_gpio_status(uint8_t status, uint8_t pin)
{
	gpio_pin_config_t gpio_config;
	int index;

	gpio_config.pinDirection = (status & APALIS_TK1_K20_GPIO_STA_OE) ? kGPIO_DigitalOutput : kGPIO_DigitalInput;
	gpio_config.outputLogic = (status & APALIS_TK1_K20_GPIO_STA_VAL);

	index = is_gpio_valid(pin);

	if (index >= 0)
		GPIO_PinInit(gpio_list[index].gpio, gpio_list[index].pin, &gpio_config);
	else
		return index;

	return 0;
}


uint8_t get_gpio_status(uint8_t pin)
{
	uint8_t status;
	int index;
	GPIO_Type *base;
	uint32_t gpio_pin;

	index = is_gpio_valid(pin);
	if (index == -EINVAL)
		return 0xFF;
	base = gpio_list[index].gpio;
	gpio_pin = gpio_list[index].pin;

	if (((base->PDDR) >> gpio_pin) & 0x01U) {
		status = APALIS_TK1_K20_GPIO_STA_OE;
		status += (((base->PDOR) >> gpio_pin) & 0x01U) ? APALIS_TK1_K20_GPIO_STA_VAL : 0x00;
	} else {
		status = 0x00;
		status += (((base->PDIR) >> gpio_pin) & 0x01U) ? APALIS_TK1_K20_GPIO_STA_VAL : 0x00;
	}

	return status;
}

int gpio_registers(uint8_t *rx_buf, uint8_t *tx_buf){
	if (rx_buf[0] == APALIS_TK1_K20_READ_INST) {
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_GPIOREG:
			return -ENOENT;
			break;
		case APALIS_TK1_K20_GPIO_NO:
			if (gen_regs.gpio_no != 0xFF){
				tx_buf[0] = gen_regs.gpio_no;
				return 1;
			} else
				return -ENOENT;
			break;
		case APALIS_TK1_K20_GPIO_STA:
			if (gen_regs.gpio_no != 0xFF){
				tx_buf[0] = get_gpio_status(gen_regs.gpio_no);
				return 1;
			} else
				return -ENOENT;
			break;
		default:
			return -ENOENT;
		}
	} else if (rx_buf[0] == APALIS_TK1_K20_WRITE_INST) {
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_GPIOREG:
			return -ENOENT;
			break;
		case APALIS_TK1_K20_GPIO_NO:
			if (is_gpio_valid(rx_buf[2]) >= 0){
				gen_regs.gpio_no = rx_buf[2];
				return 0;
			} else {
				gen_regs.gpio_no = 0xFF;
				return -ENOENT;
			}
			break;
		case APALIS_TK1_K20_GPIO_STA:
			return set_gpio_status(rx_buf[2], gen_regs.gpio_no);
			break;
		default:
			return -ENOENT;
		}
	}
	return -ENOENT;
}




