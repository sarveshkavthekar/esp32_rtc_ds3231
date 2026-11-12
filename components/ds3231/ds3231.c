#include "ds3231.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ds3231";

#define I2C_TIMEOUT_MS 100

/* Register map */
#define REG_SECONDS   0x00   /* 0x00..0x06: sec, min, hour, wday, date, month|century, year */
#define REG_ALARM1    0x07   /* 0x07..0x0A: sec, min, hour, day/date */
#define REG_ALARM2    0x0B   /* 0x0B..0x0D: min, hour, day/date      */
#define REG_CONTROL   0x0E
#define REG_STATUS    0x0F
#define REG_AGING     0x10
#define REG_TEMP_MSB  0x11   /* 0x11 MSB, 0x12 LSB */

/* Control register bits */
#define CTRL_EOSC   (1u << 7)
#define CTRL_CONV   (1u << 5)
#define CTRL_RS_MSK (3u << 3)
#define CTRL_INTCN  (1u << 2)
#define CTRL_A2IE   (1u << 1)
#define CTRL_A1IE   (1u << 0)

/* Status register bits */
#define STAT_OSF    (1u << 7)
#define STAT_EN32K  (1u << 3)
#define STAT_BSY    (1u << 2)
#define STAT_A2F    (1u << 1)
#define STAT_A1F    (1u << 0)

struct ds3231_dev {
    i2c_master_dev_handle_t i2c;
    i2c_master_bus_handle_t bus;   /* only set when created by ds3231_begin() */
    bool owns_bus;
};

/* ---------- small helpers ---------- */

static inline uint8_t bin2bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static inline int     bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }

/* Howard Hinnant's days_from_civil: days since 1970-01-01, valid for any proleptic Gregorian date */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static void fill_derived(struct tm *t)
{
    int64_t d  = days_from_civil(t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
    int64_t d0 = days_from_civil(t->tm_year + 1900, 1, 1);
    t->tm_yday  = (int)(d - d0);
    t->tm_wday  = (int)(((d % 7) + 7 + 4) % 7);   /* 1970-01-01 was a Thursday (4) */
    t->tm_isdst = 0;
}

static bool tm_valid(const struct tm *t)
{
    static const uint8_t dim[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int y = t->tm_year + 1900;
    if (y < 2000 || y > 2199) return false;
    if (t->tm_mon < 0 || t->tm_mon > 11) return false;
    if (t->tm_hour < 0 || t->tm_hour > 23) return false;
    if (t->tm_min < 0 || t->tm_min > 59) return false;
    if (t->tm_sec < 0 || t->tm_sec > 59) return false;
    int max_d = dim[t->tm_mon];
    if (t->tm_mon == 1 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) max_d = 29;
    return t->tm_mday >= 1 && t->tm_mday <= max_d;
}

static esp_err_t reg_read(ds3231_handle_t h, uint8_t reg, uint8_t *buf, size_t len)
{
    /* One transaction: START, addr+W, reg, REPEATED START, addr+R, data..., STOP */
    return i2c_master_transmit_receive(h->i2c, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t reg_write(ds3231_handle_t h, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t frame[8];                       /* reg pointer + up to 7 data bytes */
    ESP_RETURN_ON_FALSE(len <= 7, ESP_ERR_INVALID_ARG, TAG, "write too long");
    frame[0] = reg;
    memcpy(&frame[1], data, len);
    return i2c_master_transmit(h->i2c, frame, len + 1, I2C_TIMEOUT_MS);
}

static esp_err_t reg_update(ds3231_handle_t h, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t v;
    ESP_RETURN_ON_ERROR(reg_read(h, reg, &v, 1), TAG, "rmw read");
    v = (uint8_t)((v & ~mask) | (value & mask));
    return reg_write(h, reg, &v, 1);
}

/*
 * Status register needs care: A1F/A2F are "write 0 to clear, write 1 = leave alone".
 * Always writing 1 to both, except the one we clear on purpose, means an alarm that
 * fires between our read and our write is never lost.
 */
static esp_err_t status_update(ds3231_handle_t h, uint8_t set, uint8_t clear)
{
    uint8_t v;
    ESP_RETURN_ON_ERROR(reg_read(h, REG_STATUS, &v, 1), TAG, "status read");
    v |= STAT_A1F | STAT_A2F;
    v = (uint8_t)((v | set) & (uint8_t)~clear);
    return reg_write(h, REG_STATUS, &v, 1);
}

static esp_err_t wait_bit_clear(ds3231_handle_t h, uint8_t reg, uint8_t mask, int timeout_ms)
{
    for (int t = 0; t < timeout_ms; t += 10) {
        uint8_t v;
        ESP_RETURN_ON_ERROR(reg_read(h, reg, &v, 1), TAG, "poll read");
        if (!(v & mask)) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

/* ---------- lifecycle ---------- */

esp_err_t ds3231_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz, ds3231_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && out, ESP_ERR_INVALID_ARG, TAG, "null arg");

    ds3231_handle_t h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "no mem");

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DS3231_I2C_ADDR,
        .scl_speed_hz    = scl_speed_hz ? scl_speed_hz : 100000,   /* chip max: 400 kHz */
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &h->i2c);
    if (err != ESP_OK) { free(h); return err; }

    /* First real transfer doubles as a presence check. EOSC=0 keeps the oscillator alive on VBAT. */
    err = reg_update(h, REG_CONTROL, CTRL_EOSC, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no ACK from 0x%02X (%s)", DS3231_I2C_ADDR, esp_err_to_name(err));
        i2c_master_bus_rm_device(h->i2c);
        free(h);
        return err;
    }
    *out = h;
    return ESP_OK;
}

esp_err_t ds3231_deinit(ds3231_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null arg");
    esp_err_t err = i2c_master_bus_rm_device(h->i2c);
    if (h->owns_bus) i2c_del_master_bus(h->bus);
    free(h);
    return err;
}

/* ---------- easy mode ---------- */

esp_err_t ds3231_begin(int sda_pin, int scl_pin, ds3231_handle_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "null arg");
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &bus), TAG, "bus create");
    esp_err_t err = ds3231_init(bus, 400000, out);
    if (err != ESP_OK) { i2c_del_master_bus(bus); return err; }
    (*out)->bus = bus;
    (*out)->owns_bus = true;
    return ESP_OK;
}

esp_err_t ds3231_set_datetime_if_lost(ds3231_handle_t h, int year, int month, int day,
                                      int hour, int minute, int second)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null arg");
    bool lost;
    ESP_RETURN_ON_ERROR(ds3231_lost_power(h, &lost), TAG, "lost_power");
    if (!lost) return ESP_OK;
    struct tm t = {
        .tm_year = year - 1900, .tm_mon = month - 1, .tm_mday = day,
        .tm_hour = hour, .tm_min = minute, .tm_sec = second,
    };
    ESP_LOGI(TAG, "RTC was not set, writing initial time");
    return ds3231_set_time(h, &t);
}

esp_err_t ds3231_set_build_time_if_lost(ds3231_handle_t h, const char *date, const char *time_str)
{
    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mon[4] = {0};
    int d, y, hh, mm, ss;
    ESP_RETURN_ON_FALSE(date && time_str, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(sscanf(date, "%3s %d %d", mon, &d, &y) == 3 &&
                        sscanf(time_str, "%d:%d:%d", &hh, &mm, &ss) == 3 &&
                        strstr(months, mon), ESP_ERR_INVALID_ARG, TAG, "bad date/time string");
    int m = (int)(strstr(months, mon) - months) / 3 + 1;
    return ds3231_set_datetime_if_lost(h, y, m, d, hh, mm, ss);
}

esp_err_t ds3231_get_string(ds3231_handle_t h, char *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(buf && len >= 20, ESP_ERR_INVALID_ARG, TAG, "buffer too small");
    struct tm t;
    ESP_RETURN_ON_ERROR(ds3231_get_time(h, &t), TAG, "get time");
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &t);
    return ESP_OK;
}

/* ---------- time ---------- */

esp_err_t ds3231_get_time(ds3231_handle_t h, struct tm *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "null arg");

    uint8_t r[7];
    ESP_RETURN_ON_ERROR(reg_read(h, REG_SECONDS, r, sizeof r), TAG, "read time");

    memset(out, 0, sizeof *out);
    out->tm_sec = bcd2bin(r[0] & 0x7F);
    out->tm_min = bcd2bin(r[1] & 0x7F);

    if (r[2] & 0x40) {                         /* chip is in 12 h mode (someone else configured it) */
        int hr = bcd2bin(r[2] & 0x1F);
        if (hr == 12) hr = 0;
        out->tm_hour = (r[2] & 0x20) ? hr + 12 : hr;
    } else {
        out->tm_hour = bcd2bin(r[2] & 0x3F);
    }

    /* r[3] is the weekday register: user-defined, so we ignore it and compute from the date */
    out->tm_mday = bcd2bin(r[4] & 0x3F);
    out->tm_mon  = bcd2bin(r[5] & 0x1F) - 1;
    out->tm_year = bcd2bin(r[6]) + 100 + ((r[5] & 0x80) ? 100 : 0);
    fill_derived(out);
    return ESP_OK;
}

esp_err_t ds3231_set_time(ds3231_handle_t h, const struct tm *in)
{
    ESP_RETURN_ON_FALSE(h && in, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(tm_valid(in), ESP_ERR_INVALID_ARG, TAG, "time out of range");

    struct tm t = *in;
    fill_derived(&t);
    int y = t.tm_year + 1900;

    uint8_t r[7] = {
        bin2bcd(t.tm_sec),
        bin2bcd(t.tm_min),
        bin2bcd(t.tm_hour),                    /* bit6 = 0 -> 24 h mode */
        (uint8_t)(t.tm_wday + 1),              /* chip wants 1..7 */
        bin2bcd(t.tm_mday),
        (uint8_t)(bin2bcd(t.tm_mon + 1) | (y >= 2100 ? 0x80 : 0)),
        bin2bcd(y % 100),
    };
    ESP_RETURN_ON_ERROR(reg_write(h, REG_SECONDS, r, sizeof r), TAG, "write time");
    return status_update(h, 0, STAT_OSF);      /* time is valid again */
}

esp_err_t ds3231_lost_power(ds3231_handle_t h, bool *lost)
{
    ESP_RETURN_ON_FALSE(h && lost, ESP_ERR_INVALID_ARG, TAG, "null arg");
    uint8_t s;
    ESP_RETURN_ON_ERROR(reg_read(h, REG_STATUS, &s, 1), TAG, "status read");
    *lost = (s & STAT_OSF) != 0;
    return ESP_OK;
}

/* ---------- epoch helpers ---------- */

time_t ds3231_tm_to_epoch(const struct tm *t)
{
    int64_t days = days_from_civil(t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
    return (time_t)(days * 86400 + t->tm_hour * 3600 + t->tm_min * 60 + t->tm_sec);
}

esp_err_t ds3231_get_epoch(ds3231_handle_t h, time_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "null arg");
    struct tm t;
    ESP_RETURN_ON_ERROR(ds3231_get_time(h, &t), TAG, "get time");
    *out = ds3231_tm_to_epoch(&t);
    return ESP_OK;
}

esp_err_t ds3231_set_epoch(ds3231_handle_t h, time_t epoch)
{
    struct tm t;
    gmtime_r(&epoch, &t);
    return ds3231_set_time(h, &t);
}

esp_err_t ds3231_sync_system_time(ds3231_handle_t h)
{
    time_t e;
    ESP_RETURN_ON_ERROR(ds3231_get_epoch(h, &e), TAG, "get epoch");
    struct timeval tv = { .tv_sec = e, .tv_usec = 0 };
    return settimeofday(&tv, NULL) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t ds3231_update_from_system_time(ds3231_handle_t h)
{
    time_t now;
    time(&now);
    return ds3231_set_epoch(h, now);
}

/* ---------- temperature ---------- */

esp_err_t ds3231_get_temperature(ds3231_handle_t h, float *celsius)
{
    ESP_RETURN_ON_FALSE(h && celsius, ESP_ERR_INVALID_ARG, TAG, "null arg");
    uint8_t r[2];
    ESP_RETURN_ON_ERROR(reg_read(h, REG_TEMP_MSB, r, 2), TAG, "temp read");
    /* 10-bit two's complement, 0.25 C/LSB: signed MSB is the integer part, top 2 bits of LSB are the fraction */
    *celsius = (float)(int8_t)r[0] + (float)(r[1] >> 6) * 0.25f;
    return ESP_OK;
}

esp_err_t ds3231_force_temp_conversion(ds3231_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_ERROR(wait_bit_clear(h, REG_STATUS, STAT_BSY, 300), TAG, "busy");
    ESP_RETURN_ON_ERROR(reg_update(h, REG_CONTROL, CTRL_CONV, CTRL_CONV), TAG, "start conv");
    return wait_bit_clear(h, REG_CONTROL, CTRL_CONV, 300);
}

/* ---------- alarms ---------- */

static bool alarm_fields_ok(const struct tm *t)
{
    return t && t->tm_sec >= 0 && t->tm_sec <= 59 && t->tm_min >= 0 && t->tm_min <= 59 &&
           t->tm_hour >= 0 && t->tm_hour <= 23 && t->tm_mday >= 0 && t->tm_mday <= 31 &&
           t->tm_wday >= 0 && t->tm_wday <= 6;
}

esp_err_t ds3231_set_alarm1(ds3231_handle_t h, ds3231_alarm1_mode_t mode, const struct tm *t)
{
    ESP_RETURN_ON_FALSE(h && alarm_fields_ok(t), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t m = (uint8_t)mode;
    int dydt  = (m >> 4) & 1;
    int day   = dydt ? t->tm_wday + 1 : t->tm_mday;

    uint8_t r[4] = {
        (uint8_t)(bin2bcd(t->tm_sec)  | ((m & 0x01) << 7)),   /* A1M1 */
        (uint8_t)(bin2bcd(t->tm_min)  | ((m & 0x02) << 6)),   /* A1M2 */
        (uint8_t)(bin2bcd(t->tm_hour) | ((m & 0x04) << 5)),   /* A1M3 */
        (uint8_t)(bin2bcd(day) | ((m & 0x08) << 4) | (dydt << 6)), /* A1M4 + DY/DT */
    };
    return reg_write(h, REG_ALARM1, r, sizeof r);
}

esp_err_t ds3231_set_alarm2(ds3231_handle_t h, ds3231_alarm2_mode_t mode, const struct tm *t)
{
    ESP_RETURN_ON_FALSE(h && alarm_fields_ok(t), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t m = (uint8_t)mode;
    int dydt  = (m >> 4) & 1;
    int day   = dydt ? t->tm_wday + 1 : t->tm_mday;

    uint8_t r[3] = {
        (uint8_t)(bin2bcd(t->tm_min)  | ((m & 0x01) << 7)),   /* A2M2 */
        (uint8_t)(bin2bcd(t->tm_hour) | ((m & 0x02) << 6)),   /* A2M3 */
        (uint8_t)(bin2bcd(day) | ((m & 0x04) << 5) | (dydt << 6)), /* A2M4 + DY/DT */
    };
    return reg_write(h, REG_ALARM2, r, sizeof r);
}

esp_err_t ds3231_enable_alarm_interrupt(ds3231_handle_t h, int alarm, bool enable)
{
    ESP_RETURN_ON_FALSE(h && (alarm == 1 || alarm == 2), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t bit = (alarm == 1) ? CTRL_A1IE : CTRL_A2IE;
    if (enable) {
        return reg_update(h, REG_CONTROL, CTRL_INTCN | bit, CTRL_INTCN | bit);  /* INTCN=1: pin = alarm */
    }
    return reg_update(h, REG_CONTROL, bit, 0);
}

esp_err_t ds3231_alarm_fired(ds3231_handle_t h, int alarm, bool *fired)
{
    ESP_RETURN_ON_FALSE(h && fired && (alarm == 1 || alarm == 2), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t s;
    ESP_RETURN_ON_ERROR(reg_read(h, REG_STATUS, &s, 1), TAG, "status read");
    *fired = (s & (alarm == 1 ? STAT_A1F : STAT_A2F)) != 0;
    return ESP_OK;
}

esp_err_t ds3231_clear_alarm(ds3231_handle_t h, int alarm)
{
    ESP_RETURN_ON_FALSE(h && (alarm == 1 || alarm == 2), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    return status_update(h, 0, alarm == 1 ? STAT_A1F : STAT_A2F);
}

/* ---------- pins / calibration ---------- */

esp_err_t ds3231_set_square_wave(ds3231_handle_t h, ds3231_sqw_t rate)
{
    ESP_RETURN_ON_FALSE(h && rate <= DS3231_SQW_8192HZ, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    /* INTCN=0 selects the square wave on the INT/SQW pin; RS2:RS1 pick the rate */
    return reg_update(h, REG_CONTROL, CTRL_INTCN | CTRL_RS_MSK, (uint8_t)(rate << 3));
}

esp_err_t ds3231_enable_32khz(ds3231_handle_t h, bool enable)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null arg");
    return enable ? status_update(h, STAT_EN32K, 0) : status_update(h, 0, STAT_EN32K);
}

esp_err_t ds3231_get_aging_offset(ds3231_handle_t h, int8_t *offset)
{
    ESP_RETURN_ON_FALSE(h && offset, ESP_ERR_INVALID_ARG, TAG, "null arg");
    uint8_t v;
    ESP_RETURN_ON_ERROR(reg_read(h, REG_AGING, &v, 1), TAG, "aging read");
    *offset = (int8_t)v;
    return ESP_OK;
}

esp_err_t ds3231_set_aging_offset(ds3231_handle_t h, int8_t offset)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null arg");
    uint8_t v = (uint8_t)offset;
    return reg_write(h, REG_AGING, &v, 1);
}