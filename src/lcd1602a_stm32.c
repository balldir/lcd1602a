/*
 * lcd1602a_stm32.c
 *
 *  Created on: 24 квіт. 2016 р.
 *      Author: balldir
 */

#include "lcd1602a_stm32.h"

#define LCD1602A_TAS_NS  1
#define LCD1602A_TDSW_NS 40
#define LCD1602A_TPW_NS  140
#define LCD1602A_TH_NS   10
#define LCD1602A_TC_NS   1200
#define LCD1602A_TDDR_NS 100
#define LCD1602A_4_BITS_OFFSET 4

#define US_TO_NS(x) (x * 1000)
#define MS_TO_NS(x) (x * 1000000)

static inline lcd1602a_error_t lcd1602a_check_ctx(const lcd1602a_ctx_t * ctx)
{
	if (!ctx || !ctx->gpio_read_pin || !ctx->gpio_write_pin || !ctx->wait_ns) {
		return LCD1602A_ERROR;
	}
	return LCD1602A_OK;
}

lcd1602a_error_t lcd1602a_xfer(const lcd1602a_ctx_t * ctx, lcd1602a_rs_state_t rs_state, lcd1602a_rw_state_t rw_state, unsigned int *data_eight_bits, lcd1602a_wait_t wait)
{
	if (lcd1602a_check_ctx(ctx) != LCD1602A_OK) {
		return LCD1602A_ERROR;
	}
	if (ctx->interface == LCD1602A_INTERFACE_8B) {
		ctx->gpio_write_pin(ctx->rs_pin.port, ctx->rs_pin.pin, rs_state);
		ctx->gpio_write_pin(ctx->rw_pin.port, ctx->rw_pin.pin, rw_state);
		ctx->wait_ns(LCD1602A_TAS_NS);
		ctx->gpio_write_pin(ctx->e_pin.port, ctx->e_pin.pin, 1);
		ctx->wait_ns(LCD1602A_TPW_NS - LCD1602A_TDSW_NS);
		if (rw_state == LCD1602A_RW_WRITE) {
			for (unsigned int i = 0; i < 8; i++) {
				ctx->gpio_write_pin(ctx->data_pins[i].port, ctx->data_pins[i].pin, (*data_eight_bits >> i) & 1);
			}
		}
		else {
			for (unsigned int i = 0; i < 8; i++) {
				*data_eight_bits |= (!!ctx->gpio_read_pin(ctx->data_pins[i].port, ctx->data_pins[i].pin)) << i;
			}
		}
		ctx->wait_ns(LCD1602A_TDSW_NS);
		ctx->gpio_write_pin(ctx->e_pin.port, ctx->e_pin.pin, 0);
		if (wait == LCD1602A_WAIT)  {
			ctx->wait_ns(LCD1602A_TC_NS - LCD1602A_TPW_NS);
		}
	}
	else {
		for (int j=1; j >= 0; j--) {
			ctx->gpio_write_pin(ctx->rs_pin.port, ctx->rs_pin.pin, rs_state);
			ctx->gpio_write_pin(ctx->rw_pin.port, ctx->rw_pin.pin, rw_state);
			ctx->wait_ns(LCD1602A_TAS_NS);
			ctx->gpio_write_pin(ctx->e_pin.port, ctx->e_pin.pin, 1);
			ctx->wait_ns(LCD1602A_TPW_NS - LCD1602A_TDSW_NS);
			if (rw_state == LCD1602A_RW_WRITE) {
				for (unsigned int i = 0; i < 4; i++) {
					ctx->gpio_write_pin(ctx->data_pins[i + LCD1602A_4_BITS_OFFSET].port, ctx->data_pins[i + LCD1602A_4_BITS_OFFSET].pin, (*data_eight_bits >> (i + j*LCD1602A_4_BITS_OFFSET) & 1));
				}
			}
			else {
				for (unsigned int i = 0; i < 4; i++) {
					*data_eight_bits |= (!!ctx->gpio_read_pin(ctx->data_pins[i + LCD1602A_4_BITS_OFFSET].port, ctx->data_pins[i + LCD1602A_4_BITS_OFFSET].pin)) << (i + j*LCD1602A_4_BITS_OFFSET);
				}
			}
			ctx->wait_ns(LCD1602A_TDSW_NS);
			ctx->gpio_write_pin(ctx->e_pin.port, ctx->e_pin.pin, 0);
			if (j || wait == LCD1602A_WAIT)  {
				ctx->wait_ns(LCD1602A_TC_NS - LCD1602A_TPW_NS);
			}
		}
	}
	return LCD1602A_OK;
}

lcd1602a_error_t lcd1602a_clear_display(const lcd1602a_ctx_t * ctx, lcd1602a_wait_t wait)
{
	unsigned int data = 1;
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_CLEAR_DISPLAY_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_return_home(const lcd1602a_ctx_t * ctx, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 1);
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_RETURN_HOME_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_entry_mode_set(const lcd1602a_ctx_t * ctx, lcd1602a_id_t id, lcd1602a_shift_t shift, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 2) | id | shift;
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_ENTRY_MODE_SET_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_display_onoff(const lcd1602a_ctx_t * ctx, lcd1602a_display_onoff_t display, lcd1602a_cursor_onoff_t cursor, lcd1602a_cursor_blink_onoff_t cursor_blink, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 3) | display | cursor | cursor_blink;
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_DISPLAY_ONOFF_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_cursor_display_shift(const lcd1602a_ctx_t * ctx,  lcd1602a_cursor_display_shift_t shift, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 4) | shift;
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_DISPLAY_SHIFT_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_function_set(const lcd1602a_ctx_t * ctx, lcd1602a_interface_t interface, lcd1602a_lines_t lines, lcd1602a_font_t font, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 5) | interface | lines |  font;
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_FUNC_SET_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_set_cgram(const lcd1602a_ctx_t * ctx, unsigned int cgram_addr, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 6) | (cgram_addr & 0x3F);
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_SET_CGRAM_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_set_ddram(const lcd1602a_ctx_t * ctx, unsigned int ddram_addr, lcd1602a_wait_t wait)
{
	unsigned int data = (1 << 7) | (ddram_addr & 0x7F);
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_WRITE, &data, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_SET_DDRAM_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_read_busy_addr(const lcd1602a_ctx_t * ctx, unsigned int *busy, unsigned int *addr, lcd1602a_wait_t wait)
{
	unsigned int data = 0;
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_LOW, LCD1602A_RW_READ, &data, LCD1602A_NO_WAIT);
	*busy = !!(data & (1<<7));
	*addr = (data & ((1<<7) - 1));
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_READ_BUSY_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_write_data_ram(const lcd1602a_ctx_t * ctx, unsigned int data_eight_bits, lcd1602a_wait_t wait)
{
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_HIGH, LCD1602A_RW_WRITE, &data_eight_bits, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_WRITERAM_WAIT_US));
	}
	return error;
}

lcd1602a_error_t lcd1602a_read_data_ram (const lcd1602a_ctx_t * ctx, unsigned int *data_eight_bits, lcd1602a_wait_t wait)
{
	lcd1602a_error_t error = lcd1602a_xfer(ctx, LCD1602A_RS_HIGH, LCD1602A_RW_READ, data_eight_bits, LCD1602A_NO_WAIT);
	if (error == LCD1602A_OK && wait == LCD1602A_WAIT) {
		ctx->wait_ns(US_TO_NS(LCD1602A_READRAM_WAIT_US));
	}
	return error;
}

int lcd1602a_print_data(const lcd1602a_ctx_t * ctx, const char* data, unsigned int datalen)
{
	lcd1602a_error_t status = LCD1602A_OK;
	int i = 0;
	for (; i < datalen && status == LCD1602A_OK; i++) {
		status = lcd1602a_write_data_ram(ctx, data[i], LCD1602A_WAIT);
	}
	return (status == LCD1602A_OK)? i : status;
}

lcd1602a_error_t lcd1602a_init(const lcd1602a_ctx_t * ctx)
{
	if (lcd1602a_check_ctx(ctx) != LCD1602A_OK) {
		return LCD1602A_ERROR;
	}
	if (ctx->interface == LCD1602A_INTERFACE_8B) {
		ctx->wait_ns(MS_TO_NS(15));
		unsigned int data = 0b11 << 4;
		lcd1602a_xfer(ctx, 0, 0, &data,  LCD1602A_WAIT);
		ctx->wait_ns(MS_TO_NS(4.1));
		lcd1602a_xfer(ctx, 0, 0, &data,  LCD1602A_WAIT);
		ctx->wait_ns(US_TO_NS(4.1));
		lcd1602a_xfer(ctx, 0, 0, &data,  LCD1602A_WAIT);
		lcd1602a_function_set(ctx,LCD1602A_INTERFACE_8B, LCD1602A_LINES_2, LCD1602A_FONT_5X11, LCD1602A_WAIT);
		lcd1602a_display_onoff(ctx, LCD1602A_DISPLAY_OFF, LCD1602A_CURSOR_OFF, LCD1602A_CURSOR_BLINK_OFF, LCD1602A_WAIT);
		lcd1602a_clear_display(ctx, LCD1602A_WAIT);
		lcd1602a_entry_mode_set(ctx, LCD1602A_INCREMENT_ADDR, LCD1602A_NO_SHIFT, LCD1602A_WAIT);
		lcd1602a_display_onoff(ctx, LCD1602A_DISPLAY_ON, LCD1602A_CURSOR_OFF, LCD1602A_CURSOR_BLINK_ON, LCD1602A_WAIT);
	}
	else {
		lcd1602a_ctx_t tmp_ctx = *ctx;
		tmp_ctx.interface = LCD1602A_INTERFACE_8B;
		unsigned int data = 0b11 << 4;
		lcd1602a_xfer(&tmp_ctx, 0, 0, &data,  LCD1602A_WAIT);
		ctx->wait_ns(MS_TO_NS(4.1));
		lcd1602a_xfer(&tmp_ctx, 0, 0, &data,  LCD1602A_WAIT);
		ctx->wait_ns(US_TO_NS(4.1));
		lcd1602a_xfer(&tmp_ctx, 0, 0, &data,  LCD1602A_WAIT);
		ctx->wait_ns(US_TO_NS(4.1));
		data = 0b10 << 4;
		lcd1602a_xfer(&tmp_ctx, 0, 0, &data,  LCD1602A_WAIT);
		ctx->wait_ns(MS_TO_NS(4.1));
		lcd1602a_function_set(ctx, LCD1602A_INTERFACE_4B, LCD1602A_LINES_2, LCD1602A_FONT_5X11, LCD1602A_WAIT);
		lcd1602a_display_onoff(ctx, LCD1602A_DISPLAY_OFF, LCD1602A_CURSOR_OFF, LCD1602A_CURSOR_BLINK_OFF, LCD1602A_WAIT);
		lcd1602a_clear_display(ctx, LCD1602A_WAIT);
		lcd1602a_entry_mode_set(ctx, LCD1602A_INCREMENT_ADDR, LCD1602A_NO_SHIFT, LCD1602A_WAIT);
		lcd1602a_display_onoff(ctx, LCD1602A_DISPLAY_ON, LCD1602A_CURSOR_OFF, LCD1602A_CURSOR_BLINK_ON, LCD1602A_WAIT);
	}
	return LCD1602A_OK;
}
