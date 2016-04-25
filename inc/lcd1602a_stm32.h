/*
 * lcd1602a_stm32.h
 *
 *  Created on: 24 квіт. 2016 р.
 *      Author: balldir
 */

#ifndef INC_LCD1602A_STM32_H_
#define INC_LCD1602A_STM32_H_

#define LCD1602A_CLEAR_DISPLAY_WAIT_US  1520
#define LCD1602A_RETURN_HOME_WAIT_US    1520
#define LCD1602A_ENTRY_MODE_SET_WAIT_US 37
#define LCD1602A_DISPLAY_ONOFF_WAIT_US  37
#define LCD1602A_DISPLAY_SHIFT_WAIT_US  37
#define LCD1602A_FUNC_SET_WAIT_US       37
#define LCD1602A_SET_CGRAM_WAIT_US      37
#define LCD1602A_SET_DDRAM_WAIT_US      37
#define LCD1602A_READ_BUSY_WAIT_US      2
#define LCD1602A_WRITERAM_WAIT_US       37
#define LCD1602A_READRAM_WAIT_US        37

typedef enum {
	LCD1602A_RS_LOW = 0,
	LCD1602A_RS_HIGH = 1,
} lcd1602a_rs_state_t;

typedef enum {
	LCD1602A_RW_WRITE = 0,
	LCD1602A_RW_READ  = 1,
} lcd1602a_rw_state_t;

typedef enum {
	LCD1602A_ERROR = -1,
	LCD1602A_OK = 0,
} lcd1602a_error_t;

typedef enum {
	LCD1602A_NO_WAIT = 0,
	LCD1602A_WAIT = 1,
} lcd1602a_wait_t;

typedef enum {
	LCD1602A_DECREMENT_ADDR = 0,
	LCD1602A_INCREMENT_ADDR = 0x2,
} lcd1602a_id_t;

typedef enum {
	LCD1602A_NO_SHIFT = 0,
	LCD1602A_SHIFT = 1,
} lcd1602a_shift_t;

typedef enum {
	LCD1602A_DISPLAY_OFF = 0x0,
	LCD1602A_DISPLAY_ON = 0x4,
} lcd1602a_display_onoff_t;

typedef enum {
	LCD1602A_CURSOR_OFF = 0x0,
	LCD1602A_CURSOR_ON =  0x2,
} lcd1602a_cursor_onoff_t;

typedef enum {
	LCD1602A_CURSOR_BLINK_OFF    = 0x0,
	LCD1602A_CURSOR_BLINK_ON     = 0x1,
} lcd1602a_cursor_blink_onoff_t;

typedef enum {
	LCD1602A_SHIFT_CURSOR_LEFT   = 0x0,
	LCD1602A_SHIFT_CURSOR_RIGHT  = 0x4,
	LCD1602A_SHIFT_DISPLAY_LEFT  = 0x8,
	LCD1602A_SHIFT_DISPLAY_RIGHT = 0xC,
} lcd1602a_cursor_display_shift_t;

typedef enum {
	LCD1602A_FONT_5X11 = 0x4,
	LCD1602A_FONT_5X8  = 0x0,
} lcd1602a_font_t;

typedef enum {
	LCD1602A_LINES_1  = 0x0,
	LCD1602A_LINES_2  = 0x8,
} lcd1602a_lines_t;

typedef enum {
	LCD1602A_INTERFACE_4B = 0x0,
	LCD1602A_INTERFACE_8B = 0x10,
} lcd1602a_interface_t;

typedef struct _lcd1602a_pin_t {
	unsigned int port;
	unsigned int pin;
} struct_lcd1602a_pin_t;

typedef struct _lcd1602a_ctx_t {
	void                  (*gpio_write_pin)(unsigned int port, unsigned int pin, unsigned int state);
	unsigned int          (*gpio_read_pin) (unsigned int port, unsigned int pin);
	void                  (*wait_ns)       (unsigned int ns);
	struct_lcd1602a_pin_t data_pins[8];
	struct_lcd1602a_pin_t e_pin;
	struct_lcd1602a_pin_t rs_pin;
	struct_lcd1602a_pin_t rw_pin;
} lcd1602a_ctx_t;

lcd1602a_error_t lcd1602a_xfer(const lcd1602a_ctx_t * ctx, lcd1602a_rs_state_t rs_state, lcd1602a_rw_state_t rw_state,
		                       unsigned int *data_eight_bits, lcd1602a_wait_t wait);

lcd1602a_error_t lcd1602a_clear_display (const lcd1602a_ctx_t * ctx, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_return_home   (const lcd1602a_ctx_t * ctx, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_entry_mode_set(const lcd1602a_ctx_t * ctx, lcd1602a_id_t id, lcd1602a_shift_t shift, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_display_onoff (const lcd1602a_ctx_t * ctx, lcd1602a_display_onoff_t display,
		                                 lcd1602a_cursor_onoff_t cursor, lcd1602a_cursor_blink_onoff_t cursor_blink, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_cursor_display_shift(const lcd1602a_ctx_t * ctx, lcd1602a_cursor_display_shift_t shift, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_function_set  (const lcd1602a_ctx_t * ctx, lcd1602a_interface_t interface,
		                                 lcd1602a_lines_t lines, lcd1602a_font_t font, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_set_cgram     (const lcd1602a_ctx_t * ctx, unsigned int cgram_addr, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_set_ddram     (const lcd1602a_ctx_t * ctx, unsigned int ddram_addr, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_read_busy_addr(const lcd1602a_ctx_t * ctx, unsigned int *busy, unsigned int *addr, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_write_data_ram(const lcd1602a_ctx_t * ctx, unsigned int data_eight_bits, lcd1602a_wait_t wait);
lcd1602a_error_t lcd1602a_read_data_ram (const lcd1602a_ctx_t * ctx, unsigned int *data_eight_bits, lcd1602a_wait_t wait);

lcd1602a_error_t lcd1602a_init(const lcd1602a_ctx_t * ctx);

#endif /* INC_LCD1602A_STM32_H_ */
