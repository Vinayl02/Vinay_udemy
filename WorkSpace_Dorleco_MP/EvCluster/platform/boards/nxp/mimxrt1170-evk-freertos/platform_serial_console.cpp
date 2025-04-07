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
#include "platform_serial_console.h"
#include <platform/devicelink.h>

#include "fsl_debug_console.h"

extern "C" int _write(int file, char *ptr, int len)
{
    (void) file;

    auto deviceLink = Qul::Platform::DeviceLink::instance();
    if (deviceLink) {
        deviceLink->printMessage(ptr, len);
    } else {
        for (int i = 0; i < len; i++)
            DbgConsole_Putchar(*ptr++);
    }

    return len;
}

namespace Qul {
namespace Platform {
namespace Private {

void writeChar(char &character)
{
    _write(0 /*unused*/, &character, 1);
}

} // namespace Private
} // namespace Platform
} // namespace Qul
