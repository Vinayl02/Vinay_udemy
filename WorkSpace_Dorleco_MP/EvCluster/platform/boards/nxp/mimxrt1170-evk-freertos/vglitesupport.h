/******************************************************************************
**
** Copyright (C) 2020 The Qt Company Ltd.
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

#include "vg_lite.h"
#include <platforminterface/drawingdevice.h>
#include <platforminterface/drawingengine.h>

namespace Qul {
namespace Platform {
namespace Private {
namespace Vglite {

bool init();
bool finish();
bool flush();

void setBuffer(vg_lite_buffer_t *buffer);

#ifndef VGLITE_DISABLE_ROTATION_PREPROCESS
void removeTextureFromRotationCache(const void *ptr);
#endif

void blendTransformed(PlatformInterface::DrawingDevice *buffer,
                      const PlatformInterface::Transform &transform,
                      const PlatformInterface::RectF &destRect,
                      const PlatformInterface::Texture &texture,
                      const PlatformInterface::RectF &sourceRect,
                      const PlatformInterface::Rect &clipRect,
                      PlatformInterface::Rgba32 color,
                      int const_alpha);

void blendTexture(PlatformInterface::DrawingDevice *buffer,
                  const PlatformInterface::Point &pos,
                  const PlatformInterface::Texture &source,
                  const PlatformInterface::Rect &sourceRect,
                  PlatformInterface::Rgba32 color);

void rectFill(PlatformInterface::DrawingDevice *buffer,
              const PlatformInterface::Rect &rect,
              PlatformInterface::Rgba32 color);

void rectBlend(PlatformInterface::DrawingDevice *buffer,
               const PlatformInterface::Rect &rect,
               PlatformInterface::Rgba32 color);

void blendRoundedRect(PlatformInterface::DrawingDevice *buffer,
                      const PlatformInterface::Rect &rect,
                      const PlatformInterface::Rect &clipRect,
                      PlatformInterface::Rgba32 color,
                      int radius,
                      PlatformInterface::DrawingEngine::BlendMode blendMode);

PlatformInterface::DrawingEngine::Path *allocatePath(const PlatformInterface::PathData &pathData,
                                                     PlatformInterface::PathFillRule fillRule);
void processStroke(PlatformInterface::DrawingEngine::Path *path,
                   const PlatformInterface::StrokeProperties &strokeProperties);

void blendPath(PlatformInterface::DrawingDevice *buffer,
               PlatformInterface::DrawingEngine::Path *path,
               const PlatformInterface::Transform &transform,
               const PlatformInterface::Rect &clipRect,
               const PlatformInterface::Brush *fillBrush,
               const PlatformInterface::Brush *strokeBrush,
               int sourceOpacity,
               PlatformInterface::DrawingEngine::BlendMode blendMode);

void blend_ARGB32(PlatformInterface::DrawingDevice *buffer,
                  const PlatformInterface::Point &pos,
                  const PlatformInterface::Texture &source,
                  const PlatformInterface::Rect &srcRect,
                  int const_alpha);

void blend_ARGB32_Premultiplied(PlatformInterface::DrawingDevice *buffer,
                                const PlatformInterface::Point &pos,
                                const PlatformInterface::Texture &source,
                                const PlatformInterface::Rect &srcRect,
                                int const_alpha);

void blend_RGB32(PlatformInterface::DrawingDevice *buffer,
                 const PlatformInterface::Point &pos,
                 const PlatformInterface::Texture &source,
                 const PlatformInterface::Rect &srcRect,
                 int const_alpha);

void blend_RGB888(PlatformInterface::DrawingDevice *,
                  const PlatformInterface::Point &,
                  const PlatformInterface::Texture &,
                  const PlatformInterface::Rect &,
                  int);

void blend_RGB16(PlatformInterface::DrawingDevice *,
                 const PlatformInterface::Point &,
                 const PlatformInterface::Texture &,
                 const PlatformInterface::Rect &,
                 int);

void blend_Alpha8(PlatformInterface::DrawingDevice *,
                  const PlatformInterface::Point &,
                  const PlatformInterface::Texture &,
                  const PlatformInterface::Rect &,
                  int);
} // namespace Vglite
} // namespace Private
} // namespace Platform
} // namespace Qul
