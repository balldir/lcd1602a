# LCD1602A lib 

A simple low level library for communication with lcd1602a.

Example builds with arm-none-eabi and designed for maple like stm32f103 dev board.

## Porting guide

First two things shoudl be defined : **DEF_PORT_TYPE** and **DEF_GPIO_TYPE**.
In exapmle I've created file config.h

```
#define DEF_GPIO_TYPE uint16_t
#define DEF_PORT_TYPE GPIO_TypeDef*
```

This defines should be included into inc/lcd1602a_stm32.h . This was done by compiler arguments in Makefile.

```
-include Inc/config.h
```
Library should be inited. For stm32 it's done like this
```
static void gpio_write(GPIO_TypeDef* port, uint16_t pin, unsigned int state)
{
	HAL_GPIO_WritePin(port, pin, (GPIO_PinState)state);
}

static unsigned int gpio_read(GPIO_TypeDef* port, uint16_t pin)
{
	return HAL_GPIO_ReadPin(port, pin);
}

static void wait_ns(unsigned int ns)
{
	HAL_Delay(ns/1000000 + 1);
}

lcd1602a_ctx_t lcd = {
        .interface = LCD1602A_INTERFACE_8B,
		.gpio_write_pin = gpio_write,
		.gpio_read_pin = gpio_read,
		.wait_ns = wait_ns,
		.e_pin = {
			.port = GPIOB,
			.pin = GPIO_PIN_13,
		},
		.rs_pin = {
			.port = GPIOB,
			.pin = GPIO_PIN_15,
		},
		.rw_pin = {
			.port = GPIOB,
			.pin = GPIO_PIN_14,
		},
		.data_pins = {
				{ GPIOB,GPIO_PIN_3},
				{ GPIOB,GPIO_PIN_4},
				{ GPIOB,GPIO_PIN_10},
				{ GPIOB,GPIO_PIN_6},
				{ GPIOB,GPIO_PIN_7},
				{ GPIOB,GPIO_PIN_8},
				{ GPIOB,GPIO_PIN_9},
				{ GPIOB,GPIO_PIN_12},
		},
};
```

And the last step is to call init function
```
lcd1602a_init(&lcd);
```

It's done. Now it's possible to call something like
```
lcd1602a_print_data(&lcd, "Test", sizeof("Test"));
```