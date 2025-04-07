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
#include "vglitesupport.h"
#include "vgliterotationcache.h"
#include <platform/platform.h>
#include <platforminterface/allocator.h>
#include <platforminterface/log.h>
#include <qul/private/memutil.h>
#include <qul/image.h>

//#define VGLITE_ROTATION_CACHE_DEBUG

namespace Qul {
namespace Platform {
namespace Private {

uint16_t VgliteRotationCache::Node::timestamp() const
{
    return _timestamp;
}

vg_lite_buffer_t *VgliteRotationCache::Node::buffer()
{
    _timestamp = Qul::Platform::getPlatformInstance()->currentTimestamp();
    return &_buffer;
}

VgliteRotationCache::Node::~Node()
{
    if (_buffer.memory) {
        Qul::Private::memutil_aligned_free(_buffer.memory,
                                           Qul::Platform::getPlatformInstance()->memoryAllocator(
                                               Qul::PlatformInterface::MemoryAllocator::Custom));
    }
}

VgliteRotationCache::Node *VgliteRotationCache::get(const unsigned char *original)
{
    const auto it = _container.find(original);
    return it == _container.end() ? nullptr : &it->second;
}

bool VgliteRotationCache::available(const unsigned char *original)
{
    return _container.find(original) != _container.end();
}

void VgliteRotationCache::removeOldest()
{
    if (_container.empty()) {
        return;
    }

    const auto it = std::min_element(_container.begin(),
                                     _container.end(),
                                     [](const ContainerType::value_type &lhs, const ContainerType::value_type &rhs) {
                                         return lhs.second.timestamp() < rhs.second.timestamp();
                                     });

    const auto buffer = it->second.buffer();
    const auto size = buffer->stride * buffer->height;

    Qul::Platform::getPlatformInstance()->waitUntilAsyncReadFinished(buffer->memory,
                                                                     (uint8_t *) (buffer->memory) + size);
#ifdef VGLITE_ROTATION_CACHE_DEBUG
    Qul::PlatformInterface::log("Remove node: %#x\r\n", it->first);
#endif
    _container.erase(it);
}

VgliteRotationCache::Node *VgliteRotationCache::add(const unsigned char *original, vg_lite_buffer_t *texture)
{
    const auto size = texture->stride * texture->height;

    if (size > Qul::Platform::Private::preprocessCacheSize()) {
        Qul::PlatformInterface::log("Texture (%zu KB) cannot fit in the preprocess cache (%zu KB)\r\n",
                                    static_cast<size_t>(size / 1024),
                                    static_cast<size_t>(Qul::Platform::Private::preprocessCacheSize() / 1024));
        return nullptr;
    }

    auto allocate_memory = [&size] {
        return Qul::Private::memutil_aligned_alloc(Image::requiredAlignment,
                                                   size,
                                                   Qul::Platform::getPlatformInstance()->memoryAllocator(
                                                       Qul::PlatformInterface::MemoryAllocator::Custom));
    };

    Qul::Private::AlignedAllocation allocation = allocate_memory();
    for (; !allocation.ptr; allocation = allocate_memory()) {
        removeOldest();
    }

    auto buffer = _container[original].buffer();
    memcpy(buffer, texture, sizeof(vg_lite_buffer_t));

    buffer->handle = NULL;
    buffer->memory = allocation.ptr;
    buffer->address = (uint32_t) buffer->memory;
    buffer->tiled = VG_LITE_TILED;
    buffer->image_mode = VG_LITE_MULTIPLY_IMAGE_MODE;
    buffer->transparency_mode = VG_LITE_IMAGE_TRANSPARENT;

    // Important, can't blit into tiled format from different mode
    texture->image_mode = VG_LITE_NORMAL_IMAGE_MODE;
    vg_lite_blit(buffer, texture, nullptr, VG_LITE_BLEND_NONE, 0, VG_LITE_FILTER_POINT);
#ifdef VGLITE_ROTATION_CACHE_DEBUG
    Qul::PlatformInterface::log("Add node: original %#x, size %#x\r\n", original, size);
#endif
    return &_container[original];
}

void VgliteRotationCache::remove(const unsigned char *original)
{
    auto it = _container.find(original);

    if (it == _container.end())
        return;

#ifdef VGLITE_ROTATION_CACHE_DEBUG
    Qul::PlatformInterface::log("Remove node: %#x\r\n", it->first);
#endif
    _container.erase(it);
}

} // namespace Private
} // namespace Platform
} // namespace Qul
