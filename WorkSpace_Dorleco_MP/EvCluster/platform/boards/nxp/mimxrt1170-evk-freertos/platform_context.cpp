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
//#include "cache.h"
#include "platform_cache.h"
#include "display/platform_display.h"
#include "platform_drawing.h"
#include "platform_irq.h"
#include "platform_memory.h"
#include "platform_os.h"
#include "platform_rng.h"
#include "platform_serial_console.h"
#include "platform_touch.h"
#include "platform_time.h"
#include "vglitesupport.h"

#include "board.h"
#include "pin_mux.h"

#include <platform/platform.h>
#include <platform/devicelink.h>
#include <platforminterface/log.h>

namespace Qul {
namespace Platform {
namespace Private {
static uint64_t nextUpdate = 0;
} // namespace Private

struct NxpRt1170Platform : PlatformContext
{
    void initializeHardware() override
    {
        BOARD_InitBootPins();
        BOARD_ConfigMPU();
        BOARD_BootClockRUN();
        setInterruptPriorities();
       // BOARD_MIPIPanelTouch_I2C_Init();
        BOARD_InitDebugConsole();

        if (Qul::Platform::DeviceLink::instance())
            Qul::Platform::DeviceLink::instance()->init();
    }

    void initializePlatform() override
    {
        initSuspension();
        Qul::PlatformInterface::init16bppRendering();
        Qul::PlatformInterface::init32bppRendering();

        Private::preloadModuleResourceData();
    }

    void initializeDisplay(const PlatformInterface::Screen *) override { Private::BOARD_Touch_Init(); }

    uint64_t update() override
    {
        uint64_t timestamp = currentTimestamp();

        if (timestamp >= Private::nextUpdate) {
            Qul::PlatformInterface::updateEngine(timestamp);
        }

        return Private::nextUpdate;
    }

    void exec() override
    {
#ifndef NDEBUG
        Qul::PlatformInterface::log("exec \r\n");
#endif

        while (true) {
            Private::requestTouchData();

            const uint64_t nextUpdate = this->update();

            if (nextUpdate > currentTimestamp())
                suspend(MAINLOOP_SEMAPHORE, nextUpdate - currentTimestamp());
        }
    }

    double rand() override { return Private::rand(); }

    void scheduleEngineUpdate(uint64_t timeout) override
    {
        Private::nextUpdate = timeout;

        if (currentTimestamp() >= Private::nextUpdate) {
            resume(MAINLOOP_SEMAPHORE);
        }
    }

    uint64_t currentTimestamp() override { return qul_timestamp(); }

    FrameBufferingType frameBufferingType(const PlatformInterface::LayerEngine::ItemLayer *layer) const override
    {
        QUL_UNUSED(layer);
        return FlippedDoubleBuffering;
    }

    Qul::PlatformInterface::Screen *availableScreens(size_t *screenCount) const override
    {
        return Private::availableScreens(screenCount);
    }

    void waitUntilAsyncReadFinished(const void *begin, const void *end) override
    {
        QUL_UNUSED(end);
#ifdef VGLITE_DISABLE_ROTATION_PREPROCESS
        QUL_UNUSED(begin);
#else
        Private::Vglite::removeTextureFromRotationCache(begin);
#endif
        Private::Vglite::finish();
    }

    void flushCachesForAsyncRead(const void *addr, size_t length) override
    {
        Private::cleanInvalidateDCacheByAddr(const_cast<void *>(addr), length);
    }

    PlatformInterface::LayerEngine *layerEngine() override { return Private::layerEngine(); }

    PlatformInterface::DrawingDevice *beginFrame(const PlatformInterface::LayerEngine::ItemLayer *layer,
                                                 const PlatformInterface::Rect &rect,
                                                 int refreshInterval) override
    {
        QUL_UNUSED(rect);

        return Private::beginFrame(layer, refreshInterval);
    }

    void endFrame(const PlatformInterface::LayerEngine::ItemLayer *layer) override { Private::endFrame(layer); }

    FrameStatistics presentFrame(const PlatformInterface::Screen *screen, const PlatformInterface::Rect &rect) override
    {
        QUL_UNUSED(screen);
        QUL_UNUSED(rect);

        return Private::presentFrame();
    }

    static PlatformInterface::MemoryAllocator *preprocessAllocator() { return Private::preprocessAllocator(); }

    PlatformInterface::MemoryAllocator *memoryAllocator(PlatformInterface::MemoryAllocator::AllocationType type)
    {
        return Private::memoryAllocator(type);
    }

    void consoleWrite(char character) override { Private::writeChar(character); }
};

PlatformContext *getPlatformInstance()
{
    static NxpRt1170Platform platform;
    return &platform;
}
} // namespace Platform
} // namespace Qul
