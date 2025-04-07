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
#pragma once

#include "platform_layers.h"

#include <platform/platform.h>
#include <platforminterface/layerengine.h>

namespace Qul {
namespace Platform {
namespace Private {

class Lcdif2LayerEngine : public Qul::PlatformInterface::LayerEngine
{
public:
    Lcdif2LayerEngine();

    ItemLayer *allocateItemLayer(const Qul::PlatformInterface::Screen *screen,
                                 const Qul::PlatformInterface::LayerEngine::ItemLayerProperties &props,
                                 Qul::PlatformInterface::LayerEngine::SpriteLayer *spriteLayer = nullptr) override;

    ImageLayer *allocateImageLayer(const Qul::PlatformInterface::Screen *screen,
                                   const Qul::PlatformInterface::LayerEngine::ImageLayerProperties &props,
                                   Qul::PlatformInterface::LayerEngine::SpriteLayer *spriteLayer = nullptr) override;
    SpriteLayer *allocateSpriteLayer(const Qul::PlatformInterface::Screen *screen,
                                     const SpriteLayerProperties &props) override;

    void deallocateItemLayer(ItemLayer *layer) override;
    void deallocateImageLayer(ImageLayer *layer) override;
    void deallocateSpriteLayer(SpriteLayer *layer) override;

    void updateItemLayer(ItemLayer *layer, const ItemLayerProperties &props) override;
    void updateImageLayer(ImageLayer *layer, const ImageLayerProperties &props) override;
    void updateSpriteLayer(SpriteLayer *layer, const SpriteLayerProperties &props) override;

    static void commit();
};

PlatformInterface::LayerEngine *layerEngine();
PlatformInterface::DrawingDevice *beginFrame(const PlatformInterface::LayerEngine::ItemLayer *layer,
                                             int refreshInterval);
void endFrame(const PlatformInterface::LayerEngine::ItemLayer *layer);
FrameStatistics presentFrame();

} // namespace Private
} // namespace Platform
} // namespace Qul
