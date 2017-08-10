#ifndef BQ32000_H__
#define BQ32000_H__

/**
 * CONSTANTS
 */

#define RTC_BQ32000_OK       0
#define RTC_BQ32000_BAD     -1

#define BQ32000_ADDRESS      0x68

#define SECOND_REGISTER    0x00
#define MINUTE_REGISTER    0x01
#define HOUR_REGISTER      0x02
#define DAY_REGISTER       0x03
#define DATE_REGISTER      0x04
#define MONTH_REGISTER     0x05
#define YEAR_REGISTER      0x06
#define CONTROL_REGISTER   0x07

#define SECOND_MASK        0x7F
#define MINUTE_MASK        0x7F
#define HOUR_MASK          0x3F
#define DAY_MASK           0x07
#define DATE_MASK          0x3F
#define MONTH_MASK         0x1F
#define YEAR_MASK          0xFF
#define FLAG_MASK          0xFF

#define RESET_FLAG_ON      0x80
#define RESET_FLAG_OFF     0x00
#define TRICKLE_SETTING    0xA7

/**
 * FUNCTION PROTOTYPES
 */
typedef struct {
    uint8_t year;       // year (0 - 99)
    uint8_t month;      // month (01 - 12)
    uint8_t date;       // date (01 - 31)
    uint8_t day;        // day (01 - 07)
    uint8_t hour;       // hour (00 - 23)
    uint8_t minutes;    // minutes (00 - 59)
    uint8_t seconds;    // seconds (00 - 59)
} rtc_bq32000_datetime_t;

unsigned int bq32000_bcd2bin (unsigned int bcd_value);
unsigned int bq32000_bin2bcd (unsigned int binary_value);
int bq32000_set_rtc_data (char register_value, char data);
int bq32000_get_rtc_data (char register_value, char register_mask,
    unsigned int *return_value);
void bq32000_set_rtc_second (unsigned int value);
void bq32000_set_rtc_minute (unsigned int value);
void bq32000_set_rtc_hour (unsigned int value);
void bq32000_set_rtc_day (unsigned int value);
void bq32000_set_rtc_date (unsigned int value);
void bq32000_set_rtc_month (unsigned int value);
void bq32000_set_rtc_year (unsigned int value);
void bq32000_reset_osf (void);
int bq32000_set_rtc_datetime (rtc_bq32000_datetime_t* datetime);
int bq32000_get_rtc_second (void);
int bq32000_get_rtc_minute (void);
int bq32000_get_rtc_hour (void);
int bq32000_get_rtc_day (void);
int bq32000_get_rtc_date (void);
int bq32000_get_rtc_month (void);
int bq32000_get_rtc_year (void);
int bq32000_get_rtc_datetime (rtc_bq32000_datetime_t *datetime);
int bq32000_init_rtc (int first_run_flag);
void xBQ32000_Read(void *pvParameters);
#endif
