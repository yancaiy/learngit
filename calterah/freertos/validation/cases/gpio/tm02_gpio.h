#ifndef TM02_GPIO_H
#define TM02_GPIO_H


#define GPIO_CID_1			1

void case_gpio_id12(uint32_t gpio_no, uint32_t direction, uint32_t val);
void case_gpio_id345(uint32_t gpio_no, gpio_int_active_t type);
void case_gpio_debounce(void);


#endif
