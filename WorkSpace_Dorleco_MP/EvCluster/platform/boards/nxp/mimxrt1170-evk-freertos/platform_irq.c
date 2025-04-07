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
#include "platform_os.h"
#include "platform_touch_c.h"
#include "board.h"

#include "platform_config.h"
#ifdef QUL_PLATFORM_DEVICELINK_ENABLED
#include "platform_devicelink.h"
#endif

#include "vg_lite_platform.h"

#include "FreeRTOSConfig.h"

extern void Lcdifv2Layer_IRQHandler();

void setInterruptPriorities()
{
    NVIC_SetPriority(GPIO2_Combined_16_31_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#ifdef QUL_PLATFORM_DEVICELINK_ENABLED
    NVIC_SetPriority(BOARD_UART_IRQ, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif
}

// This IRQhandler is made to be overridden by e.g. freertos_app_switch example
// where we need to have different handling for LCDIFv2 interrupt when Application 2
// is run.
__attribute__((weak)) void QUL_LCDIFv2_IRQHandler()
{
    Lcdifv2Layer_IRQHandler();
}

void LCDIFv2_IRQHandler(void)
{
    QUL_LCDIFv2_IRQHandler();
}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    handleTouchInterrupt();
}

void GPU2D_IRQHandler(void)
{
    vg_lite_IRQHandler();
}

#ifdef QUL_PLATFORM_DEVICELINK_ENABLED
// Values defined in 3rdparty/board.h
void BOARD_UART_IRQ_HANDLER(void)
{
    handleSerialPortReceive();
    SDK_ISR_EXIT_BARRIER;
}
#endif
