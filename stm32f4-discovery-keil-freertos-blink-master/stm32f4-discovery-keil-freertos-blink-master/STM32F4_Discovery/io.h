#ifndef IO_H__
#define IO_H__

#define DEVICE1                     1
#define DEVICE2                     2
#define DEVICE3                     3
#define DEVICE4                     4
#define DEVICE5                     5
#define DEVICE6                     6
#define DEVICE7                     7
#define DEVICE8                     8
#define DEVICE9                     9

#define DISPLAY_TIMEOUT_TIME        5
#define DISPLAY_TIMEOUT_MAIN        10
typedef enum {
    
	DISPLAY_BT_START = 0,
    DISPLAY_BT_MAIN_TIME,
	DISPLAY_BT_MAIN,
	DISPLAY_BT_DEVICE1,
    DISPLAY_BT_DEVICE2,
    DISPLAY_BT_DEVICE3,
    DISPLAY_BT_DEVICE4,
    DISPLAY_BT_DEVICE5,
    DISPLAY_BT_DEVICE6,
    DISPLAY_BT_DEVICE7,
    DISPLAY_BT_DEVICE8,
    DISPLAY_BT_DEVICE9,
    DISPLAY_BT_TIME_OUT,
	DISPLAY_BT_ERROR,
	
}display_button;



void io_init(void);
void io_test(void);
void adc_init(void);
void timer_init(void);
int read_adc(void);
void io_on(uint8_t device);
void io_off(uint8_t device);
void io_irq_init(void);
void io_led_debug_init(void);
void io_button_init(void);
void xButtonProc(void * pvParameters );
void io_button_proc(void);
void io_lcd_printf(uint8_t device);
#endif
