/*
 * DS3231 RTC driver for ESP-IDF (>= 5.2, new i2c_master driver)
 * Tested target: ESP32-C6. The RTC stores whatever time you give it; the
 * epoch helpers assume UTC.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DS3231_I2C_ADDR 0x68

typedef struct ds3231_dev *ds3231_handle_t;

/* Alarm mask encodings. Bit layout (Alarm1): b0=A1M1 b1=A1M2 b2=A1M3 b3=A1M4 b4=DY/DT */
typedef enum {
    DS3231_A1_EVERY_SECOND = 0x0F,  /* every second                          */
    DS3231_A1_SECONDS      = 0x0E,  /* when seconds match                    */
    DS3231_A1_MIN_SEC      = 0x0C,  /* when minutes + seconds match          */
    DS3231_A1_HOUR_MIN_SEC = 0x08,  /* when hours + minutes + seconds match  */
    DS3231_A1_DATE_TIME    = 0x00,  /* day-of-month + h:m:s match            */
    DS3231_A1_DAY_TIME     = 0x10,  /* day-of-week  + h:m:s match            */
} ds3231_alarm1_mode_t;

/* Alarm2 has no seconds field. Bit layout: b0=A2M2 b1=A2M3 b2=A2M4 b4=DY/DT */
typedef enum {
    DS3231_A2_EVERY_MINUTE = 0x07,
    DS3231_A2_MINUTES      = 0x06,  /* when minutes match                    */
    DS3231_A2_HOUR_MIN     = 0x04,  /* when hours + minutes match            */
    DS3231_A2_DATE_TIME    = 0x00,
    DS3231_A2_DAY_TIME     = 0x10,
} ds3231_alarm2_mode_t;

typedef enum {
    DS3231_SQW_1HZ = 0,
    DS3231_SQW_1024HZ,
    DS3231_SQW_4096HZ,
    DS3231_SQW_8192HZ,
} ds3231_sqw_t;

/* ---- Easy mode: no I2C code needed in your app ---- */

/** Create the I2C bus (I2C_NUM_0, 400 kHz) on the given pins and attach the RTC. */
esp_err_t ds3231_begin(int sda_pin, int scl_pin, ds3231_handle_t *out);

/** Set the time ONLY if the RTC lost power / was never set. Otherwise leaves it running. */
esp_err_t ds3231_set_datetime_if_lost(ds3231_handle_t h, int year, int month, int day,
                                      int hour, int minute, int second);

/** Same, but uses the firmware build time. Call via the macro below so __DATE__/__TIME__ are YOUR file's. */
esp_err_t ds3231_set_build_time_if_lost(ds3231_handle_t h, const char *date, const char *time);
#define DS3231_SET_BUILD_TIME_IF_LOST(h) ds3231_set_build_time_if_lost((h), __DATE__, __TIME__)

/** Write "YYYY-MM-DD HH:MM:SS" into buf (needs >= 20 bytes). */
esp_err_t ds3231_get_string(ds3231_handle_t h, char *buf, size_t len);

/* ---- Advanced: bring your own bus ---- */

/** Attach to an existing I2C bus, verify the chip answers, make sure the oscillator runs on battery. */
esp_err_t ds3231_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz, ds3231_handle_t *out);
esp_err_t ds3231_deinit(ds3231_handle_t h);

/* Time. struct tm: tm_year = years since 1900, tm_mon = 0..11. Range 2000..2199. */
esp_err_t ds3231_get_time(ds3231_handle_t h, struct tm *out);
esp_err_t ds3231_set_time(ds3231_handle_t h, const struct tm *in);  /* wday/yday are computed for you */
esp_err_t ds3231_lost_power(ds3231_handle_t h, bool *lost);          /* OSF flag: time is untrustworthy */

/* Epoch helpers (RTC treated as UTC) */
time_t    ds3231_tm_to_epoch(const struct tm *t);
esp_err_t ds3231_get_epoch(ds3231_handle_t h, time_t *out);
esp_err_t ds3231_set_epoch(ds3231_handle_t h, time_t epoch);
esp_err_t ds3231_sync_system_time(ds3231_handle_t h);         /* RTC -> ESP32 system clock */
esp_err_t ds3231_update_from_system_time(ds3231_handle_t h);  /* ESP32 system clock -> RTC */

/* Temperature (internal sensor, +/-3 C, 0.25 C steps, auto-updates every 64 s) */
esp_err_t ds3231_get_temperature(ds3231_handle_t h, float *celsius);
esp_err_t ds3231_force_temp_conversion(ds3231_handle_t h);

/* Alarms. Unused fields of 't' are ignored by the chosen mode. For *_DAY_TIME use t->tm_wday (0..6). */
esp_err_t ds3231_set_alarm1(ds3231_handle_t h, ds3231_alarm1_mode_t mode, const struct tm *t);
esp_err_t ds3231_set_alarm2(ds3231_handle_t h, ds3231_alarm2_mode_t mode, const struct tm *t);
esp_err_t ds3231_enable_alarm_interrupt(ds3231_handle_t h, int alarm, bool enable); /* routes to INT/SQW pin */
esp_err_t ds3231_alarm_fired(ds3231_handle_t h, int alarm, bool *fired);
esp_err_t ds3231_clear_alarm(ds3231_handle_t h, int alarm);

/* Pin outputs / calibration */
esp_err_t ds3231_set_square_wave(ds3231_handle_t h, ds3231_sqw_t rate);  /* disables alarm interrupt output */
esp_err_t ds3231_enable_32khz(ds3231_handle_t h, bool enable);
esp_err_t ds3231_get_aging_offset(ds3231_handle_t h, int8_t *offset);
esp_err_t ds3231_set_aging_offset(ds3231_handle_t h, int8_t offset);

#ifdef __cplusplus
}
#endif