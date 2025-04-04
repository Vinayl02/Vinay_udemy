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
#include "platform_layers.h"
#include "display/platform_display.h"
#include "platform_memory.h"

#include "fsl_lcdifv2.h"

#include <platforminterface/screen.h>
#include <platforminterface/error.h>
#include "vglitedrawingengine.h"
#include "platform_os.h"

#include <platform/mem.h>
#include <algorithm>
#include <limits>
#include <bitset>

using namespace Qul::Platform::Private;
using namespace Qul::PlatformInterface;

namespace {
const std::size_t MAX_LAYER_COUNT = 8;
const std::size_t FB_ALIGNMENT = 32;
volatile bool waitingForVsync = false;
volatile unsigned int currentFrame = 0;
VgLiteDrawingEngine vgliteEngine;
NonCacheableAllocator nonCacheableHeapAllocator;

lcdifv2_pixel_format_t to_lcdifv2_pixel_format(Qul::PixelFormat format)
{
    switch (format) {
    case Qul::PixelFormat_ARGB32:
    case Qul::PixelFormat_ARGB32_Premultiplied:
    case Qul::PixelFormat_RGB32:
        return kLCDIFV2_PixelFormatARGB8888;
    case Qul::PixelFormat_ARGB4444:
    case Qul::PixelFormat_ARGB4444_Premultiplied:
        return kLCDIFV2_PixelFormatARGB4444;
    case Qul::PixelFormat_RGB16:
        return kLCDIFV2_PixelFormatRGB565;
    case Qul::PixelFormat_RGB888:
        return kLCDIFV2_PixelFormatRGB888;
    default:
        QUL_ASSERT(false, QulError_Lcdifv2Layer_UnsupportedPixelFormat, static_cast<int>(format));
        return kLCDIFV2_PixelFormatARGB8888;
    }
}

uint8_t bits_per_pixel(LayerEngine::ColorDepth depth)
{
    switch (depth) {
    case LayerEngine::Bpp8:
    case LayerEngine::Bpp16:
    case LayerEngine::Bpp16Alpha:
        return 16u;
    case LayerEngine::Bpp32:
    case LayerEngine::Bpp32Alpha:
    case LayerEngine::Bpp24:
    default:
        return 32u;
    }
}

Qul::PixelFormat depth_to_pixel_format(LayerEngine::ColorDepth depth)
{
    switch (depth) {
    case LayerEngine::Bpp8:
    case LayerEngine::Bpp16:
        return Qul::PixelFormat_RGB16;
    case LayerEngine::Bpp16Alpha:
        return Qul::PixelFormat_ARGB4444;
    case LayerEngine::Bpp24:
    case LayerEngine::Bpp32:
    case LayerEngine::Bpp32Alpha:
    default:
        return Qul::PixelFormat_ARGB32;
    }
}

vg_lite_buffer_format_t to_vglite_format(Qul::PixelFormat format)
{
    switch (format) {
    case Qul::PixelFormat_ARGB32:
    case Qul::PixelFormat_ARGB32_Premultiplied:
        return VG_LITE_PIXEL_FORMAT;
    case Qul::PixelFormat_RGB32:
        return VG_LITE_BGRX8888;
    case Qul::PixelFormat_RGB16:
        return VG_LITE_BGR565;
    case Qul::PixelFormat_ARGB4444:
    case Qul::PixelFormat_ARGB4444_Premultiplied:
        return VG_LITE_BGRA4444;
    case Qul::PixelFormat_Alpha8:
        return VG_LITE_A8;
    default:
        return VG_LITE_BGRA8888;
    }
}

void waitVsync()
{
    while (waitingForVsync)
        suspend(LCD_PXP_SEMAPHORE, SEMAPHORE_SUSPEND_INTERVAL_MAX);
}

template<typename T, std::size_t N>
struct Array
{
    Array()
        : _count(0)
    {}

    bool add(const T &node)
    {
        if (end() == last()) {
            return false;
        }

        _data[_count] = node;
        ++_count;
        return true;
    }

    void remove(const T &node)
    {
        auto it = std::remove(begin(), end(), node);
        _count = std::distance(begin(), it);
    }

    T *begin() { return _data; }
    T *end() { return &_data[_count]; }

private:
    T *last() { return &_data[N]; }

    T _data[N];
    std::size_t _count;
};

Array<Lcdifv2Layer *, MAX_LAYER_COUNT> _observers;
} // namespace

extern "C" {
void Lcdifv2Layer_IRQHandler()
{
    uint32_t intStatus = LCDIFV2_GetInterruptStatus(LCDIFV2, 0);
    if (0 != (intStatus & kLCDIFV2_VerticalBlankingInterrupt)) {
        ++currentFrame;
        waitingForVsync = false;
        LCDIFV2_ClearInterruptStatus(LCDIFV2, 0, kLCDIFV2_VerticalBlankingInterrupt);
    }
    resume(LCD_PXP_SEMAPHORE);
}
} // extern "C"

uint16_t Lcdifv2LayerProperties::stride() const
{
    return (bpp() * size().width() + 7) >> 3;
}

Lcdifv2Sprite::Lcdifv2Sprite(const LayerEngine::SpriteLayerProperties &properties)
    : Lcdifv2LayerUpdater(properties)
{}

void Lcdifv2Sprite::onUpdated()
{
    notify();
}

float Lcdifv2Sprite::opacity() const
{
    return properties().opacity;
}

bool Lcdifv2Sprite::enabled() const
{
    return properties().enabled;
}

Qul::PlatformInterface::Point Lcdifv2Sprite::position() const
{
    return properties().position;
}

void Lcdifv2Sprite::notify() const
{
    for (auto observer : _observers) {
        observer->onSpriteChanged(this);
    }
}

void Lcdifv2Sprite::registerObserver(Lcdifv2Layer *observer)
{
    if (_observers.add(observer)) {
        observer->onSpriteAttached(this);
    }
}

void Lcdifv2Sprite::unregisterObserver(Lcdifv2Layer *observer)
{
    _observers.remove(observer);
    observer->onSpriteDetached(this);
}

Lcdifv2Layer *Lcdifv2Layer::_head = nullptr;
uint8_t Lcdifv2Layer::_count = 0;

Lcdifv2Layer::Lcdifv2Layer(const Qul::PlatformInterface::Screen *screen, uint8_t index)
    : _dirty(true)
    , _index(Lcdifv2Layer::requestIndex(index))
    , _parent(nullptr)
    , _screen(screen)
    , _next(nullptr)
{
    append();
}

uint8_t Lcdifv2Layer::index() const
{
    return _index;
}

void Lcdifv2Layer::onSpriteChanged(const Lcdifv2Sprite *)
{
    driverCommonUpdate();
}

void Lcdifv2Layer::onSpriteAttached(Lcdifv2Sprite *sprite)
{
    _parent = sprite;
    driverCommonUpdate();
}

void Lcdifv2Layer::onSpriteDetached(const Lcdifv2Sprite *)
{
    _parent = nullptr;
}

void Lcdifv2Layer::append()
{
    _next = _head;
    _head = this;
    ++_count;
}

bool Lcdifv2Layer::dirty() const
{
    return _dirty;
}

void Lcdifv2Layer::setDirty(bool dirty) const
{
    _dirty = dirty;
}

void Lcdifv2Layer::driverCommonUpdate() const
{
    setDirty(true);

#ifdef USE_GLOBAL_ALPHA
    auto pOpacity = opacity();
#endif
    auto pEnabled = enabled();
    auto pPosition = position();
    const auto pSize = size();
    const auto pIndex = index();

    if (_parent) {
#ifdef USE_GLOBAL_ALPHA
        pOpacity *= _parent->opacity();
#endif
        pEnabled &= _parent->enabled();
        pPosition += _parent->position();
    }

    if (_screen) {
        const coord_t xMax = _screen->size().width() - pSize.width();
        const coord_t yMax = _screen->size().height() - pSize.height();
        pPosition.setX(std::min(pPosition.x(), xMax));
        pPosition.setY(std::min(pPosition.y(), yMax));
    }

    lcdifv2_blend_config_t alphaCfg;
#ifdef USE_GLOBAL_ALPHA
    alphaCfg.alphaMode = kLCDIFV2_AlphaOverride;
    alphaCfg.globalAlpha = std::numeric_limits<uint8_t>::max() * pOpacity;
#else
    alphaCfg.alphaMode = kLCDIFV2_AlphaEmbedded;
#endif

    LCDIFV2_EnableLayer(CLUSTER_LCDIFV2, pIndex, pEnabled);
    LCDIFV2_SetLayerBlendConfig(CLUSTER_LCDIFV2, pIndex, &alphaCfg);
    LCDIFV2_SetLayerSize(CLUSTER_LCDIFV2, pIndex, pSize.width(), pSize.height());
    LCDIFV2_SetLayerOffset(CLUSTER_LCDIFV2, pIndex, pPosition.x(), pPosition.y());
}

void Lcdifv2Layer::driverBufferUpdate() const
{
    setDirty(true);

    const auto pAddr = data();
    const auto pIndex = index();
    const auto pFormat = format();
    const auto tFormat = to_lcdifv2_pixel_format(pFormat);

    const lcdifv2_buffer_config_t bufferCfg = {.strideBytes = stride(), .pixelFormat = tFormat};

    LCDIFV2_SetLayerBufferConfig(CLUSTER_LCDIFV2, pIndex, &bufferCfg);
    LCDIFV2_SetLayerBufferAddr(CLUSTER_LCDIFV2, pIndex, (uint32_t) pAddr);
}

void Lcdifv2Layer::commit() const
{
    setDirty(false);
    LCDIFV2_TriggerLayerShadowLoad(CLUSTER_LCDIFV2, index());
}

void Lcdifv2Layer::setSwapFrame(int frame)
{
    _swapFrame = frame;
}

unsigned int Lcdifv2Layer::swapFrame() const
{
    return _swapFrame;
}

void Lcdifv2Layer::flush()
{
    const auto frame = currentFrame;
    for (Lcdifv2Layer *node = _head; node; node = node->_next) {
        if (node->dirty()) {
            node->commit();
            node->setSwapFrame(frame);
        }
    }
    waitingForVsync = true;
}

Qul::Platform::FramebufferFormat Lcdifv2Layer::framebufferFormat(
    const Qul::PlatformInterface::LayerEngine::ItemLayer *layer)
{
    Qul::Platform::FramebufferFormat format = {};

    auto l = reinterpret_cast<const Lcdifv2ItemLayer *>(layer);
    format.address = l->frontBuffer();
    auto size = l->size();
    format.width = size.width();
    format.height = size.height();
    format.bytesPerLine = l->stride();
    format.bitsPerPixel = l->bpp();

    switch (l->format()) {
    case Qul::PixelFormat_ARGB32:
    case Qul::PixelFormat_ARGB32_Premultiplied:
    case Qul::PixelFormat_RGB32:
        format.redChannel = {16, 8};
        format.greenChannel = {8, 8};
        format.blueChannel = {0, 8};
        format.swapBytes = 0;
        break;
    case Qul::PixelFormat_ARGB4444:
    case Qul::PixelFormat_ARGB4444_Premultiplied:
        format.redChannel = {8, 4};
        format.greenChannel = {4, 4};
        format.blueChannel = {0, 4};
        format.swapBytes = 2;
        break;
    case Qul::PixelFormat_RGB16:
        format.redChannel = {0, 5};
        format.greenChannel = {5, 6};
        format.blueChannel = {11, 5};
        format.swapBytes = 2;
        break;
    default:
        QUL_ASSERT(false, QulError_Lcdifv2Layer_UnsupportedPixelFormat, static_cast<int>(l->format()));
    }

    return format;
}

bool Lcdifv2Layer::full()
{
    return _count > MAX_LAYER_COUNT - 1;
}

uint8_t Lcdifv2Layer::requestIndex(uint8_t z)
{
    if (full()) {
        QUL_ASSERT(false, QulError_Lcdifv2Layer_LayerCountExceeded);
        return 0;
    }

    std::bitset<MAX_LAYER_COUNT> map(0);
    for (Lcdifv2Layer *node = _head; node; node = node->_next) {
        map.set(node->index(), true);
    }

    if (map.all()) {
        QUL_ASSERT(false, QulError_Lcdifv2Layer_LayerCountExceeded);
        return 0;
    }

    for (size_t i = 0; i < map.size(); i++, z = (z + 1) % map.size()) {
        if (!map.test(z))
            return z;
    }

    QUL_ASSERT(false, QulError_Lcdifv2Layer_LayerCountExceeded);
    return 0;
}

bool Lcdifv2Layer::valid(const Qul::PlatformInterface::Screen *screen, const Qul::PlatformInterface::Size &size)
{
    const auto screenSize = screen->size();
    return screenSize.height() >= size.height() && screenSize.width() >= size.width();
}

Lcdifv2ImageLayer::Lcdifv2ImageLayer(const Qul::PlatformInterface::Screen *screen,
                                     const LayerEngine::ImageLayerProperties &properties)
    : Lcdifv2LayerCommon(properties, screen, properties.z)
{
    onUpdated();
}

void Lcdifv2ImageLayer::onUpdated()
{
    driverCommonUpdate();
    driverBufferUpdate();
}

Size Lcdifv2ImageLayer::size() const
{
    return properties().texture.size();
}

uint8_t Lcdifv2ImageLayer::bpp() const
{
    return properties().texture.bitsPerPixel();
}

Qul::PixelFormat Lcdifv2ImageLayer::format() const
{
    return properties().texture.format();
}

uint8_t *Lcdifv2ImageLayer::data() const
{
    return const_cast<uint8_t *>(properties().texture.data());
}

Lcdifv2ItemLayer::ItemLayerBuffer::ItemLayerBuffer()
    : _data({nullptr, nullptr})
{
    memset(&_vgbuffer, 0, sizeof(_vgbuffer));
}

Lcdifv2ItemLayer::ItemLayerBuffer::~ItemLayerBuffer()
{
    if (_data.ptr) {
        Qul::Private::memutil_aligned_free(_data.ptr, &nonCacheableHeapAllocator);
        _data.original = nullptr;
        _data.ptr = nullptr;
    }
}

void Lcdifv2ItemLayer::ItemLayerBuffer::allocate(Lcdifv2ItemLayer *layer)
{
    QUL_ASSERT(layer, QulError_Lcdifv2Layer_LayerNotSet);
    const auto size = layer->stride() * layer->size().height();

    _data = Qul::Private::memutil_aligned_alloc(FB_ALIGNMENT, size, &nonCacheableHeapAllocator);
    QUL_ASSERT((_data.ptr && _data.original), QulError_Lcdifv2Layer_LayerAllocationFailed);

    _vgbuffer.width = layer->size().width();
    _vgbuffer.height = layer->size().height();
    _vgbuffer.stride = layer->stride();
    _vgbuffer.format = to_vglite_format(layer->format());
    _vgbuffer.handle = data();
    _vgbuffer.memory = _vgbuffer.handle;
    _vgbuffer.address = (uint32_t) _vgbuffer.handle;
}

uint8_t *Lcdifv2ItemLayer::ItemLayerBuffer::data() const
{
    return static_cast<uint8_t *>(_data.ptr);
}

Lcdifv2ItemLayer::Lcdifv2ItemLayer(const Qul::PlatformInterface::Screen *screen,
                                   const Qul::PlatformInterface::LayerEngine::ItemLayerProperties &properties)
    : Lcdifv2LayerCommon(properties, screen, properties.z)
    , _bufferId(0)
    , _drawingDevice(format(), size(), nullptr, stride(), &vgliteEngine)
{
    for (int i = 0; i < _buffersCount; ++i) {
        _buffers[i].allocate(this);
    }
    onUpdated();
}

void Lcdifv2ItemLayer::onUpdated()
{
    driverCommonUpdate();
}

Size Lcdifv2ItemLayer::size() const
{
    return properties().size;
}

uint8_t Lcdifv2ItemLayer::bpp() const
{
    return bits_per_pixel(properties().colorDepth);
}

Qul::PixelFormat Lcdifv2ItemLayer::format() const
{
    return depth_to_pixel_format(properties().colorDepth);
}

Qul::PlatformInterface::DrawingDevice *Lcdifv2ItemLayer::beginFrame(int interval) const
{
    if (swapFrame() == currentFrame)
        waitVsync();

    if (interval == -1)
        _targetFrame = currentFrame + 1;
    else
        _targetFrame = swapFrame() + interval;

    vgliteEngine.setBuffer(const_cast<vg_lite_buffer_t *>(vgbuffer()));
    _drawingDevice.setBits(static_cast<uchar *>(data()));

    return &_drawingDevice;
}

const vg_lite_buffer_t *Lcdifv2ItemLayer::vgbuffer() const
{
    return &(_buffers[_bufferId]._vgbuffer);
}

void Lcdifv2ItemLayer::endFrame() const
{
    while (waitForTargetFrame())
        suspend(LCD_PXP_SEMAPHORE, SEMAPHORE_SUSPEND_INTERVAL_MAX);
    swap();
}

bool Lcdifv2ItemLayer::waitForTargetFrame() const
{
    const unsigned int delta = _targetFrame - currentFrame;
    // Guard against wrap-around (swap intervals above 0x100 are unlikely)
    return delta != 0 && delta < 0x100;
}

uint8_t *Lcdifv2ItemLayer::data() const
{
    return _buffers[_bufferId].data();
}

uint8_t *Lcdifv2ItemLayer::frontBuffer() const
{
    return _buffers[_bufferId ^ 1].data();
}

void Lcdifv2ItemLayer::swap() const
{
    driverBufferUpdate();
    _bufferId ^= 1;
}
