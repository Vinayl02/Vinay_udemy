/******************************************************************************
**
** Copyright (C) 2020 The Qt Company Ltd.
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

#ifndef QUL_PLATFORM_SKIP_PLATFORM_CONFIG
#include "platform_config.h"
#endif

#include <platform/private/mem_info.h>

#include <platform/platform.h>

#include <utility>

#include "FreeRTOS.h"
#include "task.h"

#ifdef QUL_ENABLE_PERFORMANCE_LOGGING
#ifndef QUL_STACK_SIZE
#error QUL_STACK_SIZE must be defined for performance logging.
#endif
#if (INCLUDE_uxTaskGetStackHighWaterMark != 1)
#error uxTaskGetStackHighWaterMark must be enabled for performance logging. Please include "#define INCLUDE_uxTaskGetStackHighWaterMark 1" in your FreeRTOSConfig.h.
#endif
#endif

namespace Qul {
namespace Platform {
namespace Private {

uint32_t getStackUsage(void)
{
#ifdef QUL_ENABLE_PERFORMANCE_LOGGING
    const UBaseType_t waterMark = uxTaskGetStackHighWaterMark(NULL);

    return ((uint32_t) QUL_STACK_SIZE - (uint32_t) waterMark) * sizeof(StackType_t);
#else
    return 0;
#endif
}

} // namespace Private
} // namespace Platform
} // namespace Qul
