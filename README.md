# esp32_rtc_ds3231

DS3231 RTC driver for ESP-IDF 5.2+ (tested on ESP32-C6).

## Install
Copy `components/ds3231` into your project's `components/` folder.

## Usage
```c
ds3231_handle_t rtc;
ds3231_begin(6, 7, &rtc);                                   // SDA, SCL
ds3231_set_datetime_if_lost(rtc, 2026, 11, 12, 14, 30, 0);  // only if RTC was never set
char now[20];
ds3231_get_string(rtc, now, sizeof now);                    // "YYYY-MM-DD HH:MM:SS"
```

## License
MIT
