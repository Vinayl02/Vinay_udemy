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
#include <platform/mem.h>

#include <platform/private/mem-freertos.h>

namespace Qul {
namespace Platform {

void printHeapStats(void)
{
    Private::FreeRtos::printHeapStats();
}

void printStackStats(void)
{
    Private::FreeRtos::printStackStats();
}

void *qul_malloc(std::size_t size)
{
    return Private::FreeRtos::allocate(size);
}

void qul_free(void *ptr)
{
    Private::FreeRtos::deallocate(ptr);
}

void *qul_realloc(void *ptr, size_t size)
{
    return Private::FreeRtos::reallocate(ptr, size);
}

} // namespace Platform
} // namespace Qul
