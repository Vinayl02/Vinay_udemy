/******************************************************************************
**
** Copyright (C) 2021 The Qt Company Ltd.
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
#include "platform_os.h"
#include "platform_power.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <portmacro.h>
#include "FreeRTOSConfig.h"
#include "fsl_common.h"

#include <stdint.h>
#include <limits.h>

static SemaphoreHandle_t mainloopSemaphore = NULL;
static SemaphoreHandle_t lcdPxpSemaphore = NULL;

void qul_delayMs(uint32_t ms);

void initSuspension()
{
    mainloopSemaphore = xSemaphoreCreateBinary();
    lcdPxpSemaphore = xSemaphoreCreateBinary();
}

void suspend(SemaphoreType semaphoreType, uint64_t timeout)
{
    SemaphoreHandle_t semaphore;
    switch (semaphoreType) {
    case MAINLOOP_SEMAPHORE:
        semaphore = mainloopSemaphore;
        break;
    case LCD_PXP_SEMAPHORE:
        semaphore = lcdPxpSemaphore;
        break;
    default:
        semaphore = NULL;
    }

    TickType_t timeoutTicks = 0;

#if configUSE_16_BIT_TICKS == 1
    timeoutTicks = (timeout >= USHRT_MAX) ? portMAX_DELAY : (timeout / portTICK_PERIOD_MS);
#else
    timeoutTicks = (timeout >= UINT_MAX) ? portMAX_DELAY : (timeout / portTICK_PERIOD_MS);
#endif

    if (semaphore)
        xSemaphoreTake(semaphore, timeoutTicks);
}

void resume(SemaphoreType semaphoreType)
{
    SemaphoreHandle_t semaphore;
    switch (semaphoreType) {
    case MAINLOOP_SEMAPHORE:
        semaphore = mainloopSemaphore;
        break;
    case LCD_PXP_SEMAPHORE:
        semaphore = lcdPxpSemaphore;
        break;
    default:
        semaphore = NULL;
    }

    if (semaphore) {
        if (xPortIsInsideInterrupt() == pdTRUE)
            xSemaphoreGiveFromISR(semaphore, NULL);
        else
            xSemaphoreGive(semaphore);
    }
}

__attribute__((weak)) void vApplicationIdleHook()
{
    enterSleepMode();
}

#if (INCLUDE_xTaskGetSchedulerState != 1)
#error INCLUDE_xTaskGetSchedulerState FreeRTOS feature must be enabled
#endif /* INCLUDE_xTaskGetSchedulerState */
extern void xPortSysTickHandler();
void systick(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

__attribute__((noinline)) static void Qul_DelayAtLeastUs(uint32_t delayTime_us)
{
    SDK_DelayAtLeastUs(delayTime_us, SystemCoreClock);
}

void qul_delayMs(uint32_t ms)
{
    TickType_t ticks;
    ticks = (ms * configTICK_RATE_HZ) / 1000;
    if (!ms || !ticks) {
        return;
    } else if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        // Fall back to basic delay loop if scheduler is not yet running.
        while (ms--) {
            Qul_DelayAtLeastUs(1000);
        }
    } else {
        vTaskDelay(ticks);
    }
}
