//wrapper for rtc.c, built in library for Pi Pico W

#ifndef RTC_TIME_H
#define RTC_TIME_H

#include "hardware/rtc.h"
#include <stdio.h>
#include <stdbool.h>

void alarm_callback() {
    //rtc alarm functionality, not used (custom alarm was made)
}

inline void rtc_time_init() {
    rtc_init();
}

inline void rtc_time_set(datetime_t *dt) {
    rtc_set_datetime(dt);
}

inline void rtc_time_get(datetime_t *dt) {
    rtc_get_datetime(dt);
}


inline void rtc_schedule_alarm(datetime_t *alarm_time) {
    rtc_set_alarm(alarm_time, alarm_callback);
}

inline void rtc_cancel_alarm() {
    rtc_disable_alarm();  // Disable the alarm
}



#endif //RTC_TIME_H
