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
#pragma once

#include <stdint.h>

#ifdef __cplusplus

constexpr uint64_t SEMAPHORE_SUSPEND_INTERVAL_MAX = std::numeric_limits<uint64_t>::max();

extern "C" {
#endif
typedef enum { MAINLOOP_SEMAPHORE, LCD_PXP_SEMAPHORE } SemaphoreType;

void setInterruptPriorities();

void initSuspension();
void suspend(SemaphoreType semaphoreType, uint64_t timeout);
void resume(SemaphoreType semaphoreType);
void systick(void);
#ifdef __cplusplus
}
#endif
