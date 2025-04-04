/******************************************************************************
**
** Copyright (C) 2022 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Quick Ultralite module.
**
** $QT_BEGIN_LICENSE:COMM$
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** $QT_END_LICENSE$
**
******************************************************************************/
#include "platform_time.h"

#include <time.h>

#ifdef FSL_RTOS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif /* FSL_RTOS_FREE_RTOS */

#define NXP_TIMER_PRECISION configTICK_RATE_HZ

uint64_t qul_timestamp()
{
    return xTaskGetTickCount() / (NXP_TIMER_PRECISION / 1000);
}

#if defined(__ICCARM__)
/*
 * Note from the IAR C/C++ Development Guide:
 * To make the time and date functions work, you must implement the three functions
 * clock, time, and __getzone.
 */

// The number of times an internal timing event occurs per second
int const CLOCKS_PER_SECOND = 1000;

clock_t clock(void)
{
    // This function should return a value, which after division by CLOCKS_PER_SECOND,
    // is the processor time in seconds.
    return (clock_t) qul_timestamp();
}

#if _DLIB_TIME_USES_64
time_t __time64(time_t *t)
#else
time_t __time32(time_t *t)
#endif
{
    uint64_t currentTimestamp = qul_timestamp();
    // same timestamp as _gettimeofday
    time_t curtime = (time_t) (60 * (60 * 13 + 33) + currentTimestamp / NXP_TIMER_PRECISION);

    if (t)
        *t = curtime;

    return curtime;
}

char const *__getzone()
{
    // See <IAR>/src/lib/time/getzone.c for documentation
    // For Germany as a default timezone
    return ":GMT+1:GMT+2:0100:032502+0:102502+0";
}

__ATTRIBUTES char *_DstMalloc(size_t);
__ATTRIBUTES void _DstFree(char *);

char *_DstMalloc(size_t s)
{
    // Return a buffer that can hold the maximum number of DST entries of
    // of any timezone available on the device.
    // Each DST entry takes up a structure of 5 bytes plus regular alignment.
    // Instead of a static buffer a dynamically allocated memory can be used as well.

    // With the two entries shown above the required buffer size would be
    // 2 * (5 bytes size + 3 bytes alignment) = 16 bytes

    static char buffert[8 * 4];
    return buffert;
}

void _DstFree(char *p)
{
    // Nothing required here because of static buffer in _DstMalloc
}

#else
int _gettimeofday(struct timeval *tp, void *ts)
{
    uint64_t currentTimestamp = qul_timestamp();
    // set the time to begin somewhere during the Berlin afternoon,
    // to make the clock example more interesting
    tp->tv_sec = (60 * (60 * 13 + 33) + currentTimestamp / NXP_TIMER_PRECISION);
    tp->tv_usec =(uint64_t)(currentTimestamp % NXP_TIMER_PRECISION) * (1000000 / NXP_TIMER_PRECISION);
    return 0;
}
#endif
