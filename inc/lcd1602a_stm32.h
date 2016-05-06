/*
 * lcd1602a_stm32.h
 *
 *  Created on: 24 квіт. 2016 р.
 *      Author: balldir
 */

#ifndef INC_LCD1602A_STM32_H_
#define INC_LCD1602A_STM32_H_

/**
 * @warning DEF_GPIO_TYPE and DEF_PORT_TYPE should be defined as part of this
 * lib porting
 */
#if !defined(DEF_GPIO_TYPE) || !defined(DEF_PORT_TYPE)
#error DEF_GPIO_TYPE or DEF_PORT_TYPE not defined
#endif

typedef DEF_GPIO_TYPE gpio_t;
typedef DEF_PORT_TYPE gpio_port_t;

/**
 * Time that should pass after @ref lcd1602a_clear_display before next command
 */
#define LCD1602A_CLEAR_DISPLAY_WAIT_US  1520

/**
 * Time that should pass after @ref lcd1602a_return_home before next command
 */
#define LCD1602A_RETURN_HOME_WAIT_US    1520

/**
 * Time that should pass after @ref lcd1602a_entry_mode_set before next command
 */
#define LCD1602A_ENTRY_MODE_SET_WAIT_US 37

/**
 * Time that should pass after @ref lcd1602a_display_onoff before next command
 */
#define LCD1602A_DISPLAY_ONOFF_WAIT_US  37

/**
 * Time that should pass after @ref lcd1602a_cursor_display_shift before next 
 * command
 */
#define LCD1602A_DISPLAY_SHIFT_WAIT_US  37

/**
 * Time that should pass after @ref lcd1602a_function_set before next command
 */
#define LCD1602A_FUNC_SET_WAIT_US       37

/**
 * Time that should pass after @ref lcd1602a_set_cgram before next command
 */
#define LCD1602A_SET_CGRAM_WAIT_US      37

/**
 * Time that should pass after @ref lcd1602a_set_ddram before next command
 */
#define LCD1602A_SET_DDRAM_WAIT_US      37

/**
 * Time that should pass after @ref lcd1602a_read_busy_addr before next command
 */
#define LCD1602A_READ_BUSY_WAIT_US      2

/**
 * Time that should pass after @ref lcd1602a_write_data_ram before next command
 */
#define LCD1602A_WRITERAM_WAIT_US       37

/**
 * Time that should pass after @ref lcd1602a_read_data_ram before next command
 */
#define LCD1602A_READRAM_WAIT_US        37

/**
 * Possible states of rs line
 */
typedef enum {
	LCD1602A_RS_LOW = 0,
	LCD1602A_RS_HIGH = 1,
} lcd1602a_rs_state_t;

/**
 * Possible states of r/w line
 */
typedef enum {
	LCD1602A_RW_WRITE = 0,
	LCD1602A_RW_READ  = 1,
} lcd1602a_rw_state_t;

/**
 * LCD1602 library error codes
 */
typedef enum {
	LCD1602A_ERROR = -1,
	LCD1602A_OK = 0,
} lcd1602a_error_t;

/**
 * Wait config
 */
typedef enum {
	LCD1602A_NO_WAIT = 0,   /**< Function should not wait after finish*/
	LCD1602A_WAIT = 1,      /**< Function should wait after finish */
} lcd1602a_wait_t;

/**
 * Address change config
 */
typedef enum {
	LCD1602A_DECREMENT_ADDR = 0,
	LCD1602A_INCREMENT_ADDR = 0x2,
} lcd1602a_id_t;

/**
 * Shift config
 */
typedef enum {
	LCD1602A_NO_SHIFT = 0,
	LCD1602A_SHIFT = 1,
} lcd1602a_shift_t;

/**
 * Display config
 */
typedef enum {
	LCD1602A_DISPLAY_OFF = 0x0,
	LCD1602A_DISPLAY_ON = 0x4,
} lcd1602a_display_onoff_t;

/**
 * Cursor config
 */
typedef enum {
	LCD1602A_CURSOR_OFF = 0x0,
	LCD1602A_CURSOR_ON =  0x2,
} lcd1602a_cursor_onoff_t;

/**
 * Cursor blink config
 */
typedef enum {
	LCD1602A_CURSOR_BLINK_OFF    = 0x0,
	LCD1602A_CURSOR_BLINK_ON     = 0x1,
} lcd1602a_cursor_blink_onoff_t;

/**
 * Shift config
 */
typedef enum {
	LCD1602A_SHIFT_CURSOR_LEFT   = 0x0,
	LCD1602A_SHIFT_CURSOR_RIGHT  = 0x4,
	LCD1602A_SHIFT_DISPLAY_LEFT  = 0x8,
	LCD1602A_SHIFT_DISPLAY_RIGHT = 0xC,
} lcd1602a_cursor_display_shift_t;

/**
 * Font size config
 */
typedef enum {
	LCD1602A_FONT_5X11 = 0x4,
	LCD1602A_FONT_5X8  = 0x0,
} lcd1602a_font_t;

/**
 * Number of lines config
 */
typedef enum {
	LCD1602A_LINES_1  = 0x0,
	LCD1602A_LINES_2  = 0x8,
} lcd1602a_lines_t;

/**
 * Avaliable interface config
 */
typedef enum {
	LCD1602A_INTERFACE_4B = 0x0,  /**< Use 4 lines interface
	                               *(lcd1602a_ctx_t.data_pins[4-7] should be inited) */
	LCD1602A_INTERFACE_8B = 0x10, /**< Use 8 lines interface */
} lcd1602a_interface_t;

/** Pin description */
typedef struct _lcd1602a_pin_t {
	gpio_port_t port;
	gpio_t pin;
} struct_lcd1602a_pin_t;

/**
 * LCD1602 library context
 */
typedef struct _lcd1602a_ctx_t {
	lcd1602a_interface_t  interface;
	/** Function for setting pin state */
	void                  (*gpio_write_pin)(gpio_port_t port, gpio_t pin, 
                              unsigned int state);
	/** Function for reading pin state */
	unsigned int          (*gpio_read_pin) (gpio_port_t port, gpio_t pin);
	/** Function for waiting */
	void                  (*wait_ns)       (unsigned int ns);
	struct_lcd1602a_pin_t data_pins[8];    /**< Array of D[0-7] lines */
	struct_lcd1602a_pin_t e_pin;           /**< E line   */
	struct_lcd1602a_pin_t rs_pin;          /**< RS line  */
	struct_lcd1602a_pin_t rw_pin;          /**< R/W line */
} lcd1602a_ctx_t;

/**
 * Performs low level read/write
 * @param ctx              Pointer to user inited context
 * @param rs_state         State of rs pin @ref lcd1602a_rs_state_t
 * @param rw_state         State of rw pin @ref lcd1602a_rw_state_t
 * @param data_eight_bits  Pointer to memory with data
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_xfer(const lcd1602a_ctx_t * ctx, lcd1602a_rs_state_t rs_state, lcd1602a_rw_state_t rw_state,
		                       unsigned int *data_eight_bits, lcd1602a_wait_t wait);

/**
 * Write "20H" to DDRAM, and set DDRAM address to "00H" from AC
 * @param ctx              Pointer to user inited context
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_clear_display (const lcd1602a_ctx_t * ctx, lcd1602a_wait_t wait);

/**
 * Set DDRAM address to "00H" from AC and return cursor to its original position
 * if shifted. The contents of DDRAM are not changed.
 * @param ctx              Pointer to user inited context
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_return_home   (const lcd1602a_ctx_t * ctx, lcd1602a_wait_t wait);

/**
 * Sets cursor move direction and specifies display shift. These operations are
 * performed during data write and read.
 * @param ctx              Pointer to user inited context
 * @param id               One of @ref lcd1602a_id_t
 * @param shift            One of @ref lcd1602a_shift_t
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_entry_mode_set(const lcd1602a_ctx_t * ctx, lcd1602a_id_t id, lcd1602a_shift_t shift, lcd1602a_wait_t wait);

/**
 * Manages display state
 * @param ctx              Pointer to user inited context
 * @param display          Display state. One of  @ref lcd1602a_display_onoff_t
 * @param cursor           Cursor state. One of @ref lcd1602a_cursor_onoff_t
 * @param cursor_blink     Cursor bling state. One of @ref lcd1602a_cursor_blink_onoff_t
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_display_onoff (const lcd1602a_ctx_t * ctx, lcd1602a_display_onoff_t display,
		                                 lcd1602a_cursor_onoff_t cursor, lcd1602a_cursor_blink_onoff_t cursor_blink, lcd1602a_wait_t wait);

/**
 * Set cursor moving and display shift control bit, and direction, without
 * changing DDRAM data
 * @param ctx              Pointer to user inited context
 * @param shift            One of @ref lcd1602a_cursor_display_shift_t
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_cursor_display_shift(const lcd1602a_ctx_t * ctx, lcd1602a_cursor_display_shift_t shift, lcd1602a_wait_t wait);

/**
 * LCD module config
 * @param ctx              Pointer to user inited context
 * @param interface        Interface config (4/8 lines). One of 
 *                         @ref lcd1602a_interface_t
 * @param lines            Number of lines. One of @ref lcd1602a_lines_t
 * @param font             Font size. One of @ref lcd1602a_font_t
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_function_set  (const lcd1602a_ctx_t * ctx, lcd1602a_interface_t interface,
		                                 lcd1602a_lines_t lines, lcd1602a_font_t font, lcd1602a_wait_t wait);

/**
 * Set CGRAM address in address counter
 * @param ctx              Pointer to user inited context.
 * @param cgram_addr       6 bit CGRAM address.
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_set_cgram     (const lcd1602a_ctx_t * ctx, unsigned int cgram_addr, lcd1602a_wait_t wait);

/**
 * Set DDRAM address in address counter
 * @param ctx              Pointer to user inited context.
 * @param ddram_addr       6 bit CGRAM address.
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_set_ddram     (const lcd1602a_ctx_t * ctx, unsigned int ddram_addr, lcd1602a_wait_t wait);

/**
 * Whether during internal operation or not can be known by reading BF. The 
 * contents of address counter can also be read.
 * @param ctx              Pointer to user inited context.
 * @param busy             Busy flag.
 * @param addr             Address counter
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_read_busy_addr(const lcd1602a_ctx_t * ctx, unsigned int *busy, unsigned int *addr, lcd1602a_wait_t wait);

/**
 * Write data into internal RAM (DDRAM/CGRAM)
 * @param ctx              Pointer to user inited context.
 * @param data_eight_bits  Data with 8 meaningful bits
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_write_data_ram(const lcd1602a_ctx_t * ctx, unsigned int data_eight_bits, lcd1602a_wait_t wait);

/**
 * 
 * @param ctx              Pointer to user inited context.
 * @param data_eight_bits  Data that was read with 8 meaningful bits
 * @param wait             @ref lcd1602a_wait_t If this function should call 
 *                         ctx.wait_ns with proper time. If not user should not
 *                         call other functions before proper time passed, or
 *                         should poll busy flag. 
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_read_data_ram (const lcd1602a_ctx_t * ctx, unsigned int *data_eight_bits, lcd1602a_wait_t wait);

/**
 * Inits LCD screen
 * @param ctx              Pointer to user inited context
 * @retval LCD1602A_OK     Success
 * @retval LCD1602A_ERROR  Error happened
 */
lcd1602a_error_t lcd1602a_init(const lcd1602a_ctx_t * ctx);

/**
 * Print data to display. This function always waits.
 * @param ctx              Pointer to user inited context
 * @param data             Pointer to data
 * @param datalen          Size of data
 * @retval >=0             Number of data bytes that was send
 * @retval -1              Error
 */
int lcd1602a_print_data(const lcd1602a_ctx_t * ctx, const char* data, unsigned int datalen);
#endif /* INC_LCD1602A_STM32_H_ */
