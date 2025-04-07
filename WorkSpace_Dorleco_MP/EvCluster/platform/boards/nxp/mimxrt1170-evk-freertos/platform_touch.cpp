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
#include "platform_touch.h"
#include "platform_os.h"
#include "platform_time.h"

#include "board.h"
#include "FreeRTOSConfig.h"

#include "fsl_gt911.h"
#include "fsl_video_common.h"

#include <platforminterface/log.h>

gt911_handle_t s_touchHandle;

//static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp);
//static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode);

//static const gt911_config_t s_touchConfig = {
//    .I2C_SendFunc = BOARD_MIPIPanelTouch_I2C_Send,
//    .I2C_ReceiveFunc = BOARD_MIPIPanelTouch_I2C_Receive,
//    .timeDelayMsFunc = VIDEO_DelayMs,
//    .intPinFunc = BOARD_ConfigMIPIPanelTouchIntPin,
//    .pullResetPinFunc = BOARD_PullMIPIPanelTouchResetPin,
//    .touchPointNum = 1,
//    .i2cAddrMode = kGT911_I2cAddrMode0,
//    .intTrigMode = kGT911_IntRisingEdge,
//};

namespace Qul {
namespace Platform {
namespace Private {

void BOARD_Touch_Init()
{
//    status_t status;
//    status = kStatus_Success;

//    const gpio_pin_config_t touch_config = {.direction = kGPIO_DigitalInput,
//                                            .outputLogic = 0,
//                                            .interruptMode = kGPIO_IntRisingEdge};
//
//    const gpio_pin_config_t resetPinConfig = {.direction = kGPIO_DigitalOutput,
//                                              .outputLogic = 0,
//                                              .interruptMode = kGPIO_NoIntmode};
//
//    GPIO_PinInit(CLUSTER_DISPLAY_TOUCH_RST_GPIO, CLUSTER_DISPLAY_TOUCH_RST_PIN, &resetPinConfig);
//
//    status = GT911_Init(&s_touchHandle, &s_touchConfig);
//    if (kStatus_Success != status) {
//        PlatformInterface::log("Touch IC initialization failed\r\n");
//    }
//
//    /* Init touch GPIO. */
//    GPIO_PinInit(CLUSTER_DISPLAY_TOUCH_INT_GPIO, CLUSTER_DISPLAY_TOUCH_INT_PIN, &touch_config);
//    GPIO_PortEnableInterrupts(CLUSTER_DISPLAY_TOUCH_INT_GPIO, 1U << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//
//    NVIC_SetPriority(GPIO2_Combined_16_31_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
//    EnableIRQ(GPIO2_Combined_16_31_IRQn);
}

static SinglePointTouchEventQueue touchEventQueue;

static volatile bool touchDataAvailable = false;

extern "C" void handleTouchInterrupt()
{
//    GPIO_PortClearInterruptFlags(CLUSTER_DISPLAY_TOUCH_INT_GPIO, 1U << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//    touchDataAvailable = true;
//    resume(MAINLOOP_SEMAPHORE);
}

void requestTouchData()
{
//    if (touchDataAvailable) {
//        static int lastPressed = false;
//        static uint64_t lastTimeStamp = 0;
//        const uint64_t touchPeriod = 20;
//
//        uint64_t timeStamp = qul_timestamp();
//
//        static int lastX = 0, lastY = 0;
//        int x, y;
//
//        GPIO_PortDisableInterrupts(CLUSTER_DISPLAY_TOUCH_INT_GPIO, 1U << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//
//        if ((lastTimeStamp + touchPeriod > timeStamp) && lastPressed == true) {
//            touchDataAvailable = false;
//            GPIO_PortEnableInterrupts(CLUSTER_DISPLAY_TOUCH_INT_GPIO, 1U << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//            return;
//        }
//
//        lastTimeStamp = timeStamp;
//
//        status_t status = GT911_GetSingleTouch(&s_touchHandle, &x, &y);
//
//        touchDataAvailable = false;
//        GPIO_PortEnableInterrupts(CLUSTER_DISPLAY_TOUCH_INT_GPIO, 1U << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//
//        if (status == kStatus_Success) {
//            if (lastPressed == false || x != lastX || y != lastY) {
//                touchEventQueue.postEvent(SinglePointTouchEvent{timeStamp, x, y, true});
//                lastX = x;
//                lastY = y;
//                lastPressed = true;
//            }
//
//        } else if (lastPressed == true) {
//            touchEventQueue.postEvent(SinglePointTouchEvent{timeStamp, lastX, lastY, false});
//            lastPressed = false;
//        }
//    }
}

} // namespace Private
} // namespace Platform
} // namespace Qul

//static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp)
//{
//    if (pullUp) {
//        GPIO_PinWrite(CLUSTER_DISPLAY_TOUCH_RST_GPIO, CLUSTER_DISPLAY_TOUCH_RST_PIN, 1);
//    } else {
//        GPIO_PinWrite(CLUSTER_DISPLAY_TOUCH_RST_GPIO, CLUSTER_DISPLAY_TOUCH_RST_PIN, 0);
//    }
//}

//static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode)
//{
//    if (mode == kGT911_IntPinInput) {
//        CLUSTER_DISPLAY_TOUCH_INT_GPIO->GDIR &= ~(1UL << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//    } else {
//        if (mode == kGT911_IntPinPullDown) {
//            GPIO_PinWrite(CLUSTER_DISPLAY_TOUCH_INT_GPIO, CLUSTER_DISPLAY_TOUCH_INT_PIN, 0);
//        } else {
//            GPIO_PinWrite(CLUSTER_DISPLAY_TOUCH_INT_GPIO, CLUSTER_DISPLAY_TOUCH_INT_PIN, 1);
//        }
//
//        CLUSTER_DISPLAY_TOUCH_INT_GPIO->GDIR |= (1UL << CLUSTER_DISPLAY_TOUCH_INT_PIN);
//    }
//}
