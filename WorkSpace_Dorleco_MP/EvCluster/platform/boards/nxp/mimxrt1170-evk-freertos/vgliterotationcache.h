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
#include <platforminterface/allocator.h>

namespace Qul {
namespace Platform {
namespace Private {

size_t preprocessCacheSize();

class VgliteRotationCache
{
public:
    struct Node
    {
        ~Node();

        uint16_t timestamp() const;
        vg_lite_buffer_t *buffer();

    private:
        uint64_t _timestamp;
        vg_lite_buffer_t _buffer;
    };

    bool available(const unsigned char *original);
    VgliteRotationCache::Node *get(const unsigned char *original);
    VgliteRotationCache::Node *add(const unsigned char *original, vg_lite_buffer_t *texture);
    void remove(const unsigned char *original);

private:
    void removeOldest();

    using ContainerType = PlatformInterface::Map<const unsigned char *, Node>;
    ContainerType _container;
};

} // namespace Private
} // namespace Platform
} // namespace Qul
