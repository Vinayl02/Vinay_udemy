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

#include <platforminterface/devicelink.h>
#include <platform/devicelink.h>
#include <platforminterface/log.h>
#include <platforminterface/printf.h>

#include "platform_layers.h"

#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "board.h"

#include "platform_config.h"
#ifdef QUL_PLATFORM_DEVICELINK_ENABLED

extern "C" {

// Called from interrupt
void handleSerialPortReceive()
{
    // The generated board.h is broken because it converts the UART pointer to int when it should be a pointer.
    while (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags((LPUART_Type *) BOARD_DEBUG_UART_BASEADDR)) {
        uint8_t data = LPUART_ReadByte((LPUART_Type *) BOARD_DEBUG_UART_BASEADDR);
        Qul::PlatformInterface::deviceLinkBytesReceived(&data, 1);
    }

    NVIC_ClearPendingIRQ(BOARD_UART_IRQ);
}
}

struct NxpDeviceLinkInterface : Qul::Platform::DeviceLinkInterface
{
    void platformInit()
    {
        /* Enable RX interrupt. */
        // Values defined in 3rdparty/board.h
        // Generated board.h is broken as it hardcodes a conversion to uint for the base addr when really a pointer is needed.
        LPUART_EnableInterrupts((LPUART_Type *) BOARD_DEBUG_UART_BASEADDR, kLPUART_RxDataRegFullInterruptEnable);
        EnableIRQ(BOARD_UART_IRQ);
    }

    void transmitChars(const uint8_t *data, uint32_t size)
    {
        for (int i = 0; i < size; i++)
            DbgConsole_Putchar(*data++);
    }

    Qul::Platform::FramebufferFormat framebufferFormat(
        const Qul::PlatformInterface::LayerEngine::ItemLayer *layer) override
    {
        return Qul::Platform::Private::Lcdifv2Layer::framebufferFormat(layer);
    }
};
#endif

namespace Qul {
namespace Platform {

DeviceLinkInterface *getDeviceLinkInterface()
{
#ifdef QUL_PLATFORM_DEVICELINK_ENABLED
    static NxpDeviceLinkInterface deviceLink;
    return &deviceLink;
#else
    return nullptr;
#endif
}

} // namespace Platform
} // namespace Qul
