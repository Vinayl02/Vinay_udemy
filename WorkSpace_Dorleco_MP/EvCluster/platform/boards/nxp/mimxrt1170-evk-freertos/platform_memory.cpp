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
#include "platform_memory.h"
#include "platform_cache.h"

#include <platform/private/preloadallocator.h>

#include <cstring>

#if defined(__ICCARM__)
#pragma section = "NCACHE_HEAP"
static char *heapMemoryBegin = reinterpret_cast<char *>(__section_begin("NCACHE_HEAP"));
static char *heapMemoryEnd = reinterpret_cast<char *>(__section_end("NCACHE_HEAP"));
#else
extern unsigned char __noncacheheap_start;
extern unsigned char __noncacheheap_end;
static char *heapMemoryBegin = reinterpret_cast<char *>(&__noncacheheap_start);
static char *heapMemoryEnd = reinterpret_cast<char *>(&__noncacheheap_end);
#endif

extern uint8_t __preloadRamEnd, __preloadRamStart;
void *preloadRamEnd = &__preloadRamEnd, *preloadRamStart = &__preloadRamStart;

extern "C" {
char __StackTop;
char __StackLimit;
}

namespace Qul {
namespace Platform {
namespace Private {

NonCacheableAllocator::NonCacheableAllocator()
    : FixedSizeBlockAllocator(heapMemoryBegin, heapMemoryEnd)
{}

PlatformInterface::MemoryAllocator *preprocessAllocator()
{
    // 16k blocks should be a reasonable trade-off between allocation speed and avoiding wasting memory
    const auto block = preprocessCacheRange();
    static Private::FixedSizeBlockAllocator<16384> allocator(block.first, block.second);
    return &allocator;
}

PlatformInterface::MemoryAllocator *memoryAllocator(PlatformInterface::MemoryAllocator::AllocationType type)
{
    static PlatformInterface::MemoryAllocator defaultAllocator;
    static Private::ReversePreloadAllocator<4> preloadAllocator(preloadRamEnd, preloadRamStart);

    switch (type) {
    case PlatformInterface::MemoryAllocator::DefaultPreload:
        return &preloadAllocator;
    case PlatformInterface::MemoryAllocator::Custom:
        return preprocessAllocator();
    default:
        return &defaultAllocator;
    }
}

std::pair<void *, void *> stackRange()
{
    return std::make_pair(&__StackTop, &__StackLimit);
}

#if defined(__ICCARM__)
void preloadModuleResourceData()
{
#pragma section = "QulModuleResourceData"
#pragma section = "QulModuleResourceData_init"
    char *__ModuleResourceDataStart = reinterpret_cast<char *>((__section_begin("QulModuleResourceData_init")));
    char *__ModuleResourceDataCacheStart = reinterpret_cast<char *>((__section_begin("QulModuleResourceData")));
    char *__ModuleResourceDataCacheEnd = reinterpret_cast<char *>((__section_end("QulModuleResourceData")));

    memcpy(__ModuleResourceDataCacheStart,
           __ModuleResourceDataStart,
           __ModuleResourceDataCacheEnd - __ModuleResourceDataCacheStart);
#else
extern "C" unsigned char __ModuleResourceDataStart;
extern "C" unsigned char __ModuleResourceDataCacheStart;
extern "C" unsigned char __ModuleResourceDataCacheEnd;

void preloadModuleResourceData()
{
    memcpy(&__ModuleResourceDataCacheStart,
           &__ModuleResourceDataStart,
           &__ModuleResourceDataCacheEnd - &__ModuleResourceDataCacheStart);
#endif
}

} // namespace Private
} // namespace Platform
} // namespace Qul
