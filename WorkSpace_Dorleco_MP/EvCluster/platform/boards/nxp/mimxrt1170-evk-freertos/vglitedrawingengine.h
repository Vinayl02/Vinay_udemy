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
#include <platforminterface/drawingengine.h>

class VgLiteDrawingEngine : public Qul::PlatformInterface::DrawingEngine
{
public:
    void setBuffer(vg_lite_buffer_t *buffer);

    void blendRect(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                   const Qul::PlatformInterface::Rect &rect,
                   Qul::PlatformInterface::Rgba32 color,
                   BlendMode blendMode) override;

    void blendRoundedRect(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                          const Qul::PlatformInterface::Rect &rect,
                          const Qul::PlatformInterface::Rect &clipRect,
                          Qul::PlatformInterface::Rgba32 color,
                          int radius,
                          BlendMode blendMode = BlendMode_SourceOver) override;

    void blendImage(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                    const Qul::PlatformInterface::Point &pos,
                    const Qul::PlatformInterface::Texture &source,
                    const Qul::PlatformInterface::Rect &sourceRect,
                    int sourceOpacity,
                    DrawingEngine::BlendMode blendMode) override;

    void blendAlphaMap(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                       const Qul::PlatformInterface::Point &pos,
                       const Qul::PlatformInterface::Texture &source,
                       const Qul::PlatformInterface::Rect &sourceRect,
                       Qul::PlatformInterface::Rgba32 color,
                       DrawingEngine::BlendMode blendMode) override;

    void blendTransformedImage(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                               const Qul::PlatformInterface::Transform &transform,
                               const Qul::PlatformInterface::RectF &destinationRect,
                               const Qul::PlatformInterface::Texture &source,
                               const Qul::PlatformInterface::RectF &sourceRect,
                               const Qul::PlatformInterface::Rect &clipRect,
                               int sourceOpacity,
                               BlendMode blendMode) override;

    void blendTransformedAlphaMap(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                  const Qul::PlatformInterface::Transform &transform,
                                  const Qul::PlatformInterface::RectF &destinationRect,
                                  const Qul::PlatformInterface::Texture &source,
                                  const Qul::PlatformInterface::RectF &sourceRect,
                                  const Qul::PlatformInterface::Rect &clipRect,
                                  Qul::PlatformInterface::Rgba32 color,
                                  BlendMode blendMode) override;

    void synchronizeForCpuAccess(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                                 const Qul::PlatformInterface::Rect &rect) override;

    Qul::PlatformInterface::DrawingEngine::Path *allocatePath(const Qul::PlatformInterface::PathData *pathData,
                                                              Qul::PlatformInterface::PathFillRule fillRule);

    void setStrokeProperties(Qul::PlatformInterface::DrawingEngine::Path *path,
                             const Qul::PlatformInterface::StrokeProperties &strokeProperties);

    void blendPath(Qul::PlatformInterface::DrawingDevice *drawingDevice,
                   Qul::PlatformInterface::DrawingEngine::Path *path,
                   const Qul::PlatformInterface::Transform &transform,
                   const Qul::PlatformInterface::Rect &clipRect,
                   const Qul::PlatformInterface::Brush *fillBrush,
                   const Qul::PlatformInterface::Brush *strokeBrush,
                   int sourceOpacity,
                   Qul::PlatformInterface::DrawingEngine::BlendMode blendMode
                   = Qul::PlatformInterface::DrawingEngine::BlendMode_SourceOver) override;
};
