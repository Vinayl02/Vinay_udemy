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
#include "platform_cache.h"

#include "fsl_pxp.h"

#include <iterator>
#include <stdint.h>

#if !defined(__ICCARM__)
extern unsigned char __preprocessCacheStart;
extern unsigned char __preprocessCacheEnd;
#endif

namespace Qul {
namespace Platform {
namespace Private {

std::pair<char *, char *> preprocessCacheRange()
{
#if defined(__ICCARM__)
#pragma section = "QulPreprocessCache"
    char *begin = reinterpret_cast<char *>(__section_begin("QulPreprocessCache"));
    char *end = reinterpret_cast<char *>(__section_end("QulPreprocessCache"));
#else
    char *begin = reinterpret_cast<char *>(&__preprocessCacheStart);
    char *end = reinterpret_cast<char *>(&__preprocessCacheEnd);
#endif
    return std::make_pair(begin, end);
}

size_t preprocessCacheSize()
{
    const auto block = preprocessCacheRange();
    return std::distance(block.first, block.second);
}

void cleanInvalidateDCacheByAddr(void *addr, int size)
{
    uint32_t a = (uint32_t) addr;

    // addr must be 32 byte aligned
    int delta = a & 0x1f;
    a &= ~0x1f;

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *) a, size + delta);
}

} // namespace Private
} // namespace Platform
} // namespace Qul
