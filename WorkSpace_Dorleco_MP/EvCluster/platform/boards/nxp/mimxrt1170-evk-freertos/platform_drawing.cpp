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
#include "platform_drawing.h"
#include "display/platform_display.h"
#include "platform_layers.h"

#include <platforminterface/log.h>
#include <platforminterface/allocator.h>
#include <platforminterface/error.h>
#include <platform/mem.h>
#include <platform/platform.h>

#include "vglitesupport.h"
#include "fsl_lcdifv2.h"

using namespace Qul::PlatformInterface;

namespace Qul {
namespace Platform {
namespace Private {

Lcdif2LayerEngine::Lcdif2LayerEngine()
{
    Platform::Private::prepareDisplayController();

    QUL_ASSERT(Private::Vglite::init(), QulError_Vglite_InitializationFailed);
}

LayerEngine::ItemLayer *Lcdif2LayerEngine::allocateItemLayer(const Screen *screen,
                                                             const ItemLayerProperties &props,
                                                             SpriteLayer *spriteLayer)
{
    if (Lcdifv2Layer::full() || !Lcdifv2Layer::valid(screen, props.size)) {
        return nullptr;
    }

    auto l = qul_new<Lcdifv2ItemLayer>(screen, props);
    QUL_ASSERT(l, QulError_Lcdifv2Layer_LayerAllocationFailed);
    if (spriteLayer && l) {
        auto s = reinterpret_cast<Lcdifv2Sprite *>(spriteLayer);
        s->registerObserver(l);
    }
    return l;
}

LayerEngine::ImageLayer *Lcdif2LayerEngine::allocateImageLayer(
    const Qul::PlatformInterface::Screen *screen,
    const Qul::PlatformInterface::LayerEngine::ImageLayerProperties &props,
    Qul::PlatformInterface::LayerEngine::SpriteLayer *spriteLayer)
{
    if (Lcdifv2Layer::full() || !Lcdifv2Layer::valid(screen, props.texture.size())) {
        return nullptr;
    }

    auto l = qul_new<Lcdifv2ImageLayer>(screen, props);
    QUL_ASSERT(l, QulError_Lcdifv2Layer_LayerAllocationFailed);
    if (spriteLayer && l) {
        auto s = reinterpret_cast<Lcdifv2Sprite *>(spriteLayer);
        s->registerObserver(l);
    }
    return l;
}

LayerEngine::SpriteLayer *Lcdif2LayerEngine::allocateSpriteLayer(const Screen *screen,
                                                                 const SpriteLayerProperties &props)
{
    if (Lcdifv2Layer::full()) {
        return nullptr;
    }

    auto layer = qul_new<Lcdifv2Sprite>(props);
    QUL_ASSERT(layer, QulError_Lcdifv2Layer_LayerAllocationFailed);
    return layer;
}

void Lcdif2LayerEngine::deallocateItemLayer(ItemLayer *layer)
{
    qul_delete(reinterpret_cast<Lcdifv2ItemLayer *>(layer));
}

void Lcdif2LayerEngine::deallocateImageLayer(ImageLayer *layer)
{
    qul_delete(reinterpret_cast<Lcdifv2ImageLayer *>(layer));
}

void Lcdif2LayerEngine::deallocateSpriteLayer(SpriteLayer *layer)
{
    qul_delete(reinterpret_cast<Lcdifv2Sprite *>(layer));
}

void Lcdif2LayerEngine::updateItemLayer(ItemLayer *layer, const ItemLayerProperties &props)
{
    if (layer) {
        auto l = reinterpret_cast<Lcdifv2ItemLayer *>(layer);
        l->update(props);
    }
}

void Lcdif2LayerEngine::updateImageLayer(ImageLayer *layer, const ImageLayerProperties &props)
{
    if (layer) {
        auto l = reinterpret_cast<Lcdifv2ImageLayer *>(layer);
        l->update(props);
    }
}

void Lcdif2LayerEngine::updateSpriteLayer(SpriteLayer *layer, const SpriteLayerProperties &props)
{
    if (layer) {
        auto l = reinterpret_cast<Lcdifv2Sprite *>(layer);
        l->update(props);
    }
}

void Lcdif2LayerEngine::commit()
{
    Lcdifv2Layer::flush();
}

LayerEngine *layerEngine()
{
    static Lcdif2LayerEngine engine;
    return &engine;
}

DrawingDevice *beginFrame(const LayerEngine::ItemLayer *layer, int refreshInterval)
{
    if (!layer)
        return nullptr;

    auto l = reinterpret_cast<const Lcdifv2ItemLayer *>(layer);
    return l->beginFrame(refreshInterval);
}

void endFrame(const LayerEngine::ItemLayer *layer)
{
    if (!layer)
        return;

    Vglite::flush();
    auto l = reinterpret_cast<const Lcdifv2ItemLayer *>(layer);
    l->endFrame();
}

Platform::FrameStatistics presentFrame()
{
    Vglite::finish();
    Lcdif2LayerEngine::commit();
    return Platform::FrameStatistics();
}

} // namespace Private
} // namespace Platform
} // namespace Qul
