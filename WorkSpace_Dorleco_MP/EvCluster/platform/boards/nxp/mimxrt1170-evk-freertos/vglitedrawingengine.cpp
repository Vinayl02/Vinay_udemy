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

#include "vglitedrawingengine.h"

using namespace Qul::Platform::Private;

void VgLiteDrawingEngine::setBuffer(vg_lite_buffer_t *buffer)
{
    Vglite::setBuffer(buffer);
}

void VgLiteDrawingEngine::blendRect(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                    const Qul::PlatformInterface::Rect &rect,
                                    Qul::PlatformInterface::Rgba32 color,
                                    BlendMode blendMode)
{
    if (blendMode == DrawingEngine::BlendMode_Source || color.alpha() == 0xff)
        Vglite::rectFill(drawingDevice, rect, color);
    else
        Vglite::rectBlend(drawingDevice, rect, color);
}
void VgLiteDrawingEngine::blendRoundedRect(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                           const Qul::PlatformInterface::Rect &rect,
                                           const Qul::PlatformInterface::Rect &clipRect,
                                           Qul::PlatformInterface::Rgba32 color,
                                           int radius,
                                           BlendMode blendMode)
{
    Vglite::blendRoundedRect(drawingDevice, rect, clipRect, color, radius, blendMode);
}
void VgLiteDrawingEngine::blendImage(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                     const Qul::PlatformInterface::Point &pos,
                                     const Qul::PlatformInterface::Texture &source,
                                     const Qul::PlatformInterface::Rect &sourceRect,
                                     int sourceOpacity,
                                     Qul::PlatformInterface::DrawingEngine::BlendMode blendMode)
{
    QUL_UNUSED(blendMode);
    if (source.format() == Qul::PixelFormat_RGB332 || source.format() == Qul::PixelFormat_RLE_ARGB32_Premultiplied
        || source.format() == Qul::PixelFormat_RLE_RGB32 || source.format() == Qul::PixelFormat_RLE_RGB888) {
        DrawingEngine::blendImage(drawingDevice, pos, source, sourceRect, sourceOpacity, blendMode);
    } else {
        Vglite::blend_ARGB32(drawingDevice, pos, source, sourceRect, sourceOpacity);
    }
}

void VgLiteDrawingEngine::blendAlphaMap(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                        const Qul::PlatformInterface::Point &pos,
                                        const Qul::PlatformInterface::Texture &source,
                                        const Qul::PlatformInterface::Rect &sourceRect,
                                        Qul::PlatformInterface::Rgba32 color,
                                        Qul::PlatformInterface::DrawingEngine::BlendMode blendMode)
{
    QUL_UNUSED(blendMode);
    if (source.format() == Qul::PixelFormat_Alpha8) {
        Vglite::blendTexture(drawingDevice, pos, source, sourceRect, color);
    } else {
        DrawingEngine::blendAlphaMap(drawingDevice, pos, source, sourceRect, color, blendMode);
    }
}

void VgLiteDrawingEngine::blendTransformedImage(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                                const Qul::PlatformInterface::Transform &transform,
                                                const Qul::PlatformInterface::RectF &destinationRect,
                                                const Qul::PlatformInterface::Texture &source,
                                                const Qul::PlatformInterface::RectF &sourceRect,
                                                const Qul::PlatformInterface::Rect &clipRect,
                                                int sourceOpacity,
                                                Qul::PlatformInterface::DrawingEngine::BlendMode blendMode)
{
    QUL_UNUSED(blendMode);
    if (source.format() == Qul::PixelFormat_RGB332 || source.format() == Qul::PixelFormat_RLE_ARGB32_Premultiplied
        || source.format() == Qul::PixelFormat_RLE_RGB32 || source.format() == Qul::PixelFormat_RLE_RGB888) {
        DrawingEngine::blendTransformedImage(drawingDevice,
                                             transform,
                                             destinationRect,
                                             source,
                                             sourceRect,
                                             clipRect,
                                             sourceOpacity,
                                             blendMode);
    } else {
        Vglite::blendTransformed(drawingDevice,
                                 transform,
                                 destinationRect,
                                 source,
                                 sourceRect,
                                 clipRect,
                                 Qul::PlatformInterface::Rgba32(uint32_t(255),
                                                                uint32_t(255),
                                                                uint32_t(255),
                                                                uint32_t(255)),
                                 sourceOpacity);
    }
}

void VgLiteDrawingEngine::blendTransformedAlphaMap(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                                   const Qul::PlatformInterface::Transform &transform,
                                                   const Qul::PlatformInterface::RectF &destinationRect,
                                                   const Qul::PlatformInterface::Texture &source,
                                                   const Qul::PlatformInterface::RectF &sourceRect,
                                                   const Qul::PlatformInterface::Rect &clipRect,
                                                   Qul::PlatformInterface::Rgba32 color,
                                                   BlendMode blendMode)
{
    QUL_UNUSED(blendMode);
    if (source.format() == Qul::PixelFormat_Alpha8) {
        Vglite::blendTransformed(drawingDevice, transform, destinationRect, source, sourceRect, clipRect, color, 256);
    } else {
        DrawingEngine::blendTransformedAlphaMap(drawingDevice,
                                                transform,
                                                destinationRect,
                                                source,
                                                sourceRect,
                                                clipRect,
                                                color,
                                                blendMode);
    }
}

void VgLiteDrawingEngine::synchronizeForCpuAccess(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                                  const Qul::PlatformInterface::Rect &rect)
{
    Vglite::finish();
}

Qul::PlatformInterface::DrawingEngine::Path *VgLiteDrawingEngine::allocatePath(
    const Qul::PlatformInterface::PathData *pathData, Qul::PlatformInterface::PathFillRule fillRule)
{
    return Vglite::allocatePath(*pathData, fillRule);
}

void VgLiteDrawingEngine::setStrokeProperties(Qul::PlatformInterface::DrawingEngine::Path *path,
                                              const Qul::PlatformInterface::StrokeProperties &strokeProperties)
{
    Vglite::processStroke(path, strokeProperties);
}

void VgLiteDrawingEngine::blendPath(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                    Qul::PlatformInterface::DrawingEngine::Path *path,
                                    const Qul::PlatformInterface::Transform &transform,
                                    const Qul::PlatformInterface::Rect &clipRect,
                                    const Qul::PlatformInterface::Brush *fillBrush,
                                    const Qul::PlatformInterface::Brush *strokeBrush,
                                    int sourceOpacity,
                                    Qul::PlatformInterface::DrawingEngine::BlendMode blendMode)
{
    Vglite::blendPath(drawingDevice, path, transform, clipRect, fillBrush, strokeBrush, sourceOpacity, blendMode);
}
