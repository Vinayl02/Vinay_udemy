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

#include "vglitesupport.h"
#include "vglitevector.h"
#include "display/vglitecontroller.h"
#include "vgliterotationcache.h"

#include <platform/mem.h>
#include <platforminterface/arc.h>
#include <platforminterface/log.h>
#include <platforminterface/pathdata.h>
#include <platforminterface/error.h>

#include <cfloat>
#include <array>

#ifdef NDEBUG
#define debug_bsp(...) ((void) 0)
#else
#define debug_bsp(fmt, ...) Qul::PlatformInterface::log(fmt, ##__VA_ARGS__)
#endif

namespace {

vg_lite_matrix_t matrix;
vg_lite_buffer_t *vgliteBufferRt;

const uint32_t kMaxConstAlpha = 256;

vg_lite_buffer_t *prepareGpuTexture(
    uchar *memory, int32_t width, int32_t height, vg_lite_buffer_format_t format, int32_t stride)
{
    static const uint8_t textureCount = 16;
    static uint8_t currentTexture = textureCount - 1;
    static vg_lite_buffer_t gpuTextures[textureCount];

    (++currentTexture) %= textureCount;
    vg_lite_buffer_t &texture = gpuTextures[currentTexture];

    texture.width = width;
    texture.height = height;
    texture.stride = stride;
    texture.format = format;

    texture.image_mode = VG_LITE_MULTIPLY_IMAGE_MODE;

    switch (format) {
    case VG_LITE_A8:
    case VG_LITE_BGRA8888:
    case VG_LITE_BGRA4444:
        texture.transparency_mode = VG_LITE_IMAGE_TRANSPARENT;
        break;
    default:
        break;
    }

    texture.memory = memory;
    texture.address = reinterpret_cast<uint32_t>(texture.memory);

    return &texture;
}

vg_lite_buffer_t *prepareGpuTexture(const Qul::PlatformInterface::Texture &source)
{
    vg_lite_buffer_format_t format;

    switch (source.format()) {
    case Qul::PixelFormat_ARGB32:
    case Qul::PixelFormat_ARGB32_Premultiplied:
        format = VG_LITE_BGRA8888;
        break;
    case Qul::PixelFormat_RGB32:
        format = VG_LITE_BGRX8888;
        break;
    case Qul::PixelFormat_RGB16:
        format = VG_LITE_BGR565;
        break;
    case Qul::PixelFormat_ARGB4444:
    case Qul::PixelFormat_ARGB4444_Premultiplied:
        format = VG_LITE_BGRA4444;
        break;
    case Qul::PixelFormat_Alpha8:
        format = VG_LITE_A8;
        break;
    default:
        debug_bsp("ERROR: Unsupported Texture Format!\r\n");
        format = VG_LITE_BGRA8888;
    }

    return prepareGpuTexture(const_cast<uchar *>(source.data()),
                             source.width(),
                             source.height(),
                             format,
                             source.bytesPerLine());
}

void setScissor(const Qul::PlatformInterface::Rect &clipRect)
{
    vg_lite_set_scissor(clipRect.x(), clipRect.y(), clipRect.width(), clipRect.height());
}
} // namespace

namespace Qul {
namespace Platform {
namespace Private {
namespace Vglite {

bool init()
{
    status_t status;
    vg_lite_error_t error;

    status = BOARD_PrepareVGLiteController();
    if (status != kStatus_Success) {
        PlatformInterface::log("Prepare VGlite contolor error\r\n");
        return false;
    }

    error = vg_lite_init(64, 64);
    if (error) {
        PlatformInterface::log("vg_lite engine init failed: vg_lite_init() returned error %d\n", error);
        vg_lite_close();
        return false;
    }

#ifndef VGLITE_DISABLE_PREMULTIPLY
    error = vg_lite_enable_premultiply();
    if (error) {
        PlatformInterface::log("vg_lite_enable_premultiply can't enable premultiplied alpha, error %d\n", error);
        vg_lite_close();
        return false;
    }
#endif
    return true;
}

bool finish()
{
    return vg_lite_finish() == VG_LITE_SUCCESS;
}

bool flush()
{
    return vg_lite_flush() == VG_LITE_SUCCESS;
}

void setBuffer(vg_lite_buffer_t *buffer)
{
    vgliteBufferRt = buffer;
}

#ifndef VGLITE_DISABLE_ROTATION_PREPROCESS
static VgliteRotationCache cache;

void removeTextureFromRotationCache(const void *ptr)
{
    cache.remove(static_cast<const unsigned char *>(ptr));
}
#endif

void blendTransformed(PlatformInterface::DrawingDevice * /*buffer*/,
                      const PlatformInterface::Transform &transform,
                      const PlatformInterface::RectF &destRect,
                      const PlatformInterface::Texture &texture,
                      const PlatformInterface::RectF &sourceRect,
                      const PlatformInterface::Rect &clipRect,
                      PlatformInterface::Rgba32 color,
                      int const_alpha)
{
    vg_lite_buffer_t *gpuTexture = prepareGpuTexture(texture);

#ifndef VGLITE_DISABLE_ROTATION_PREPROCESS
    if (texture.flags() & PlatformInterface::Texture::Rotated) {
        VgliteRotationCache::Node *node = nullptr;

        if (cache.available(texture.data())) {
            node = cache.get(texture.data());
        } else {
            node = cache.add(texture.data(), gpuTexture);
        }

        if (node) {
            gpuTexture = node->buffer();
        }
    }
#endif

    if (gpuTexture->stride % 16 != 0) {
        debug_bsp("stride not rounded: %d\r\n", static_cast<int>(gpuTexture->stride));
    }

    if (gpuTexture->address % 8 != 0) {
        debug_bsp("address not aligned: %X\r\n", static_cast<unsigned int>(gpuTexture->address));
    }

    vg_lite_float_t(&m)[3][3] = matrix.m;
    m[0][0] = transform.m11();
    m[0][1] = transform.m21();
    m[0][2] = transform.dx() + transform.m11() * destRect.x() + transform.m21() * destRect.y();
    m[1][0] = transform.m12();
    m[1][1] = transform.m22();
    m[1][2] = transform.dy() + transform.m12() * destRect.x() + transform.m22() * destRect.y();
    m[2][0] = transform.m13();
    m[2][1] = transform.m23();
    m[2][2] = transform.m33();

    /* mimxrt1170 is facing GPU hardware issue while using blit_rect together with bi-linear filtering.
    /  It interpolates endge pixels of provided rect always with transparent pixels, even if whole
    /  texture contains more pixels to interpolate with.
    /  To avoid inconsistency beetwen other platforms and desktop we would like to disable bi-linear
    /  filtering for scaling with at least one scale factor of 1.0.
    */
    vg_lite_filter_t filter = VG_LITE_FILTER_BI_LINEAR;
#if VGLITE_POINT_FILTERING_FOR_SIMPLE_SCALE
    if (transform.type() == PlatformInterface::Transform::Scale) {
        if (Qul::Private::testEqual(std::abs(transform.m11()), 1.0)
            || Qul::Private::testEqual(std::abs(transform.m22()), 1.0)) {
            filter = VG_LITE_FILTER_POINT; /* Just for scale transform with at least one scale factor 1.0 */
        }
    }
#endif

    const PlatformInterface::Rgba32 abgr32(color.blue(),
                                           color.green(),
                                           color.red(),
                                           color.alpha() * const_alpha / kMaxConstAlpha);

    uint32_t rect[] = {static_cast<uint32_t>(sourceRect.x()),
                       static_cast<uint32_t>(sourceRect.y()),
                       static_cast<uint32_t>(sourceRect.width()),
                       static_cast<uint32_t>(sourceRect.height())};

    setScissor(clipRect);
    vg_lite_enable_scissor();
    QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
    vg_lite_blit_rect(vgliteBufferRt, gpuTexture, rect, &matrix, VG_LITE_BLEND_SRC_OVER, abgr32.value, filter);
    vg_lite_disable_scissor();
}

void blendTexture(PlatformInterface::DrawingDevice * /*buffer*/,
                  const PlatformInterface::Point &pos,
                  const PlatformInterface::Texture &source,
                  const PlatformInterface::Rect &sourceRect,
                  PlatformInterface::Rgba32 color)
{
    vg_lite_buffer_t *gpuTexture = prepareGpuTexture(source);

    if (gpuTexture->stride % 16 != 0) {
        debug_bsp("stride not rounded: %d\r\n", static_cast<int>(gpuTexture->stride));
    }
    if (gpuTexture->address % 8 != 0) {
        debug_bsp("adress not alligned: %X\r\n", static_cast<unsigned int>(gpuTexture->address));
    }

    vg_lite_identity(&matrix);
    vg_lite_translate(pos.x(), pos.y(), &matrix);

    const PlatformInterface::Rgba32 abgr32(color.blue(), color.green(), color.red(), color.alpha());

    uint32_t rect[] = {static_cast<uint32_t>(sourceRect.x()),
                       static_cast<uint32_t>(sourceRect.y()),
                       static_cast<uint32_t>(sourceRect.width()),
                       static_cast<uint32_t>(sourceRect.height())};

    QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
    vg_lite_blit_rect(vgliteBufferRt,
                      gpuTexture,
                      rect,
                      &matrix,
                      VG_LITE_BLEND_SRC_OVER,
                      abgr32.value,
                      VG_LITE_FILTER_POINT);
}

void rectFill(PlatformInterface::DrawingDevice * /*buffer*/,
              const PlatformInterface::Rect &rect,
              PlatformInterface::Rgba32 color)
{
    const PlatformInterface::Rgba32 abgr32(color.blue(), color.green(), color.red(), color.alpha());
    vg_lite_rectangle clearRect = {rect.x(), rect.y(), rect.width(), rect.height()};
    QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
    vg_lite_clear(vgliteBufferRt, &clearRect, abgr32.value);
}

void rectBlend(PlatformInterface::DrawingDevice * /*buffer*/,
               const PlatformInterface::Rect &rect,
               PlatformInterface::Rgba32 color)
{
    const Vglite::Vector::BoundingBox bounding_box{static_cast<vg_lite_float_t>(rect.x()),
                                                   static_cast<vg_lite_float_t>(rect.y()),
                                                   static_cast<vg_lite_float_t>(rect.x() + rect.width()),
                                                   static_cast<vg_lite_float_t>(rect.y() + rect.height())};
    const PlatformInterface::Rgba32 abgr32(color.blue(), color.green(), color.red(), color.alpha());
    const auto builder = Vglite::Vector::PathBuilder<std::int16_t, 0>{}
                             .moveTo(rect.x(), rect.y())
                             .lineTo(rect.x(), rect.y() + rect.height())
                             .lineTo(rect.x() + rect.width(), rect.y() + rect.height())
                             .lineTo(rect.x() + rect.width(), rect.y())
                             .finish();
    vg_lite_identity(&matrix);
    auto path = builder.path(bounding_box);
    QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
    vg_lite_draw(vgliteBufferRt, &path, VG_LITE_FILL_EVEN_ODD, &matrix, VG_LITE_BLEND_SRC_OVER, abgr32.value);
}

void blendRoundedRect(PlatformInterface::DrawingDevice *buffer,
                      const PlatformInterface::Rect &rect,
                      const PlatformInterface::Rect &clipRect,
                      PlatformInterface::Rgba32 color,
                      int radius,
                      PlatformInterface::DrawingEngine::BlendMode blendMode)
{
    const PlatformInterface::Point point1{rect.x(), static_cast<PlatformInterface::coord_t>(rect.y() + radius)};
    const PlatformInterface::Point point2{static_cast<int16_t>(rect.x() + radius), static_cast<int16_t>(rect.y())};
    const PlatformInterface::Point point3{static_cast<int16_t>(rect.x() + rect.width() - radius),
                                          static_cast<int16_t>(rect.y())};
    const PlatformInterface::Point point4{static_cast<int16_t>(rect.x() + rect.width()),
                                          static_cast<int16_t>(rect.y() + radius)};
    const PlatformInterface::Point point5{static_cast<int16_t>(rect.x() + rect.width()),
                                          static_cast<int16_t>(rect.y() + rect.height() - radius)};
    const PlatformInterface::Point point6{static_cast<int16_t>(rect.x() + rect.width() - radius),
                                          static_cast<int16_t>(rect.y() + rect.height())};
    const PlatformInterface::Point point7{static_cast<int16_t>(rect.x() + radius),
                                          static_cast<int16_t>(rect.y() + rect.height())};
    const PlatformInterface::Point point8{static_cast<int16_t>(rect.x()),
                                          static_cast<int16_t>(rect.y() + rect.height() - radius)};
    // clang-format off
    const auto builder = Vglite::Vector::PathBuilder<std::int16_t, 0>{}
                             .moveTo(point1.x(), point1.y())
                             .arcTo(point1.x(),
                                    point1.y(),
                                    radius,
                                    radius,
                                    0,
                                    false,
                                    true,
                                    point2.x(),
                                    point2.y())
                             .lineTo(point3.x(), point3.y())
                             .arcTo(point3.x(),
                                    point3.y(),
                                    radius,
                                    radius,
                                    0,
                                    false,
                                    true,
                                    point4.x(),
                                    point4.y())
                             .lineTo(point5.x(), point5.y())
                             .arcTo(point5.x(),
                                    point5.y(),
                                    radius,
                                    radius,
                                    0,
                                    false,
                                    true,
                                    point6.x(),
                                    point6.y())
                             .lineTo(point7.x(), point7.y())
                             .arcTo(point7.x(),
                                    point7.y(),
                                    radius,
                                    radius,
                                    0,
                                    false,
                                    true,
                                    point8.x(),
                                    point8.y())
                             .lineTo(point1.x(), point1.y())
                             .finish();
    // clang-format on

    const PlatformInterface::Rgba32 abgr32(color.blue(), color.green(), color.red(), color.alpha());
    const Vglite::Vector::BoundingBox bounding_box{static_cast<vg_lite_float_t>(rect.x()),
                                                   static_cast<vg_lite_float_t>(rect.y()),
                                                   static_cast<vg_lite_float_t>(rect.x() + rect.width()),
                                                   static_cast<vg_lite_float_t>(rect.y() + rect.height())};
    vg_lite_identity(&matrix);
    auto path = builder.path(bounding_box);

    setScissor(clipRect);
    vg_lite_enable_scissor();
    QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
    vg_lite_draw(vgliteBufferRt,
                 &path,
                 VG_LITE_FILL_EVEN_ODD,
                 &matrix,
                 (blendMode == PlatformInterface::DrawingEngine::BlendMode_SourceOver ? VG_LITE_BLEND_SRC_OVER
                                                                                      : VG_LITE_BLEND_NONE),
                 abgr32.value);
    vg_lite_disable_scissor();
}

PlatformInterface::DrawingEngine::Path *allocatePath(const PlatformInterface::PathData &pathData,
                                                     PlatformInterface::PathFillRule fillRule)
{
    auto path = PlatformInterface::qul_new<VglitePath>(&pathData, fillRule);
    QUL_ASSERT(path, QulError_Vglite_MemoryAllocationFailed);
    return path;
}

void processStroke(PlatformInterface::DrawingEngine::Path *path,
                   const PlatformInterface::StrokeProperties &strokeProperties)
{
    VglitePath *destinationPath = static_cast<VglitePath *>(path);

    VglitePathDataStroker stroker(destinationPath);
    stroker.setStrokeProperties(strokeProperties);
    stroker.stroke();

    destinationPath->processStrokePath();
}

static void addPathSegment(void *data,
                           const PlatformInterface::PointF &c1,
                           const PlatformInterface::PointF &c2,
                           const PlatformInterface::PointF &target)
{
    VglitePath *path = static_cast<VglitePath *>(data);

    path->addFillElement(static_cast<int32_t>(VLC_OP_CUBIC));
    path->addFillElement(c1.x());
    path->addFillElement(c1.y());
    path->addFillElement(c2.x());
    path->addFillElement(c2.y());
    path->addFillElement(target.x());
    path->addFillElement(target.y());
}

static void addStrokePathSegment(void *data,
                                 const PlatformInterface::PointF &c1,
                                 const PlatformInterface::PointF &c2,
                                 const PlatformInterface::PointF &target)
{
    VglitePath *path = static_cast<VglitePath *>(data);

    path->addStrokeElement(static_cast<int32_t>(VLC_OP_CUBIC));
    path->addStrokeElement(c1.x());
    path->addStrokeElement(c1.y());
    path->addStrokeElement(c2.x());
    path->addStrokeElement(c2.y());
    path->addStrokeElement(target.x());
    path->addStrokeElement(target.y());
}

VglitePath::VglitePath(const PlatformInterface::PathData *pathData, PlatformInterface::PathFillRule fillRule_)
    : fillRule(fillRule_ == PlatformInterface::PathWindingFill ? VG_LITE_FILL_NON_ZERO : VG_LITE_FILL_EVEN_ODD)
    , path(pathData)
    , processingDone(false)
    , vgFillPath({0})
    , vgStrokePath({0})
{}

void VglitePath::processFillPath()
{
    if (processingDone)
        return;

    clearFill();

    PlatformInterface::PointF current(0.0, 0.0);

    auto arcHelper = [&](const Qul::PlatformInterface::PathDataArcSegment *segment, bool clockwise, bool large) {
        pathArcToBeziers(&addPathSegment,
                         this,
                         segment->xRadius(),
                         segment->yRadius(),
                         segment->rotation(),
                         large,
                         clockwise,
                         current,
                         segment->target());
        current = segment->target();
    };

    PlatformInterface::PathDataIterator it(path);
    while (it.hasNext()) {
        PlatformInterface::PathDataSegment segment = it.next();
        switch (segment.type()) {
        case PlatformInterface::PathData::CloseSegment: {
            // According to NXP VGLite API documentation Move and End (0x00, 0x02 and 0x03)
            // operations also close any open path(s). Using VLC_OP_CLOSE followed by
            // VLC_OP_MOVE seems to cause a glitch when the following shape is filled.
            //
            // Check that the next segment is not move segment before adding VLC_OP_CLOSE to the
            // queue.
            if (!it.hasNext() || it.peek().type() != PlatformInterface::PathData::MoveSegment)
                addFillElement(static_cast<int32_t>(VLC_OP_CLOSE));
            break;
        }
        case PlatformInterface::PathData::MoveSegment: {
            const PlatformInterface::PathDataMoveSegment *moveSegment
                = segment.as<PlatformInterface::PathDataMoveSegment>();
            addFillElement(static_cast<int32_t>(VLC_OP_MOVE));
            addFillElement(moveSegment->target().x());
            addFillElement(moveSegment->target().y());
            current = moveSegment->target();
            break;
        }
        case PlatformInterface::PathData::LineSegment: {
            const PlatformInterface::PathDataLineSegment *lineSegment
                = segment.as<PlatformInterface::PathDataLineSegment>();
            addFillElement(static_cast<int32_t>(VLC_OP_LINE));
            addFillElement(lineSegment->target().x());
            addFillElement(lineSegment->target().y());
            current = lineSegment->target();
            break;
        }
        case PlatformInterface::PathData::QuadraticBezierSegment: {
            const PlatformInterface::PathDataQuadraticBezierSegment *bezierSegment
                = segment.as<PlatformInterface::PathDataQuadraticBezierSegment>();
            addFillElement(static_cast<int32_t>(VLC_OP_QUAD));
            addFillElement(bezierSegment->controlPoint().x());
            addFillElement(bezierSegment->controlPoint().y());
            addFillElement(bezierSegment->target().x());
            addFillElement(bezierSegment->target().y());
            current = bezierSegment->target();
            break;
        }
        case PlatformInterface::PathData::CubicBezierSegment: {
            const PlatformInterface::PathDataCubicBezierSegment *bezierSegment
                = segment.as<PlatformInterface::PathDataCubicBezierSegment>();
            addFillElement(static_cast<int32_t>(VLC_OP_CUBIC));
            addFillElement(bezierSegment->firstControlPoint().x());
            addFillElement(bezierSegment->firstControlPoint().y());
            addFillElement(bezierSegment->secondControlPoint().x());
            addFillElement(bezierSegment->secondControlPoint().y());
            addFillElement(bezierSegment->target().x());
            addFillElement(bezierSegment->target().y());
            current = bezierSegment->target();
            break;
        }
        case PlatformInterface::PathData::SmallCounterClockWiseArcSegment: {
            arcHelper(segment.as<PlatformInterface::PathDataSmallCounterClockWiseArcSegment>(), false, false);
            break;
        }
        case PlatformInterface::PathData::SmallClockWiseArcSegment: {
            arcHelper(segment.as<PlatformInterface::PathDataSmallClockWiseArcSegment>(), true, false);
            break;
        }
        case PlatformInterface::PathData::LargeCounterClockWiseArcSegment: {
            arcHelper(segment.as<PlatformInterface::PathDataLargeCounterClockWiseArcSegment>(), false, true);
            break;
        }
        case PlatformInterface::PathData::LargeClockWiseArcSegment: {
            arcHelper(segment.as<PlatformInterface::PathDataLargeClockWiseArcSegment>(), true, true);
            break;
        }
        case PlatformInterface::PathData::PathSeparatorSegment:
            break;
        }
    }

    addFillElement(static_cast<int32_t>(VLC_OP_END));
    vg_lite_error_t err = vg_lite_init_path(&vgFillPath,
                                            VG_LITE_FP32,
                                            VG_LITE_MEDIUM, // quality
                                            fillPathSize(),
                                            fillElements.data(),
                                            -FLT_MAX,
                                            -FLT_MAX,
                                            FLT_MAX,
                                            FLT_MAX);
    if (err != VG_LITE_SUCCESS)
        debug_bsp("vg_lite_init_path (fill) failed (%d)\r\n", err);
    err = vg_lite_upload_path(&vgFillPath);
    if (err != VG_LITE_SUCCESS)
        debug_bsp("vg_lite_upload_path (fill) failed (%d)\r\n", err);

    processingDone = true;
}

void VglitePath::processStrokePath()
{
    vg_lite_error_t err = vg_lite_init_path(&vgStrokePath,
                                            VG_LITE_FP32,
                                            VG_LITE_MEDIUM, // quality
                                            strokePathSize(),
                                            strokeElements.data(),
                                            -FLT_MAX,
                                            -FLT_MAX,
                                            FLT_MAX,
                                            FLT_MAX);
    if (err != VG_LITE_SUCCESS)
        debug_bsp("vg_lite_init_path (stroke) failed (%d)\r\n", err);
    err = vg_lite_upload_path(&vgStrokePath);
    if (err != VG_LITE_SUCCESS)
        debug_bsp("vg_lite_upload_path (stroke) failed (%d)\r\n", err);
}

void VglitePath::clearFill()
{
    vg_lite_error_t err = vg_lite_clear_path(&vgFillPath);
    if (err != VG_LITE_SUCCESS)
        debug_bsp("Vglite fill clear failed (%d)\r\n", err);
    fillElements.clear();
}

void VglitePath::clearStroke()
{
    vg_lite_error_t err = vg_lite_clear_path(&vgStrokePath);
    if (err != VG_LITE_SUCCESS)
        debug_bsp("Vglite stroke clear failed (%d)\r\n", err);
    strokeElements.clear();
}

void blendPath(PlatformInterface::DrawingDevice *buffer,
               PlatformInterface::DrawingEngine::Path *path,
               const PlatformInterface::Transform &transform,
               const PlatformInterface::Rect &clipRect,
               const PlatformInterface::Brush *fillBrush,
               const PlatformInterface::Brush *strokeBrush,
               int sourceOpacity,
               PlatformInterface::DrawingEngine::BlendMode blendMode)
{
    VglitePath *destinationPath = static_cast<VglitePath *>(path);

    vg_lite_float_t(&m)[3][3] = matrix.m;
    m[0][0] = transform.m11();
    m[0][1] = transform.m21();
    m[0][2] = transform.m31();
    m[1][0] = transform.m12();
    m[1][1] = transform.m22();
    m[1][2] = transform.m32();
    m[2][0] = transform.m13();
    m[2][1] = transform.m23();
    m[2][2] = transform.m33();

    setScissor(clipRect);
    vg_lite_enable_scissor();

    if (fillBrush) {
        PlatformInterface::Rgba32 fillColor = fillBrush->color();
        destinationPath->processFillPath();

        const PlatformInterface::Rgba32 abgr32(fillColor.blue(),
                                               fillColor.green(),
                                               fillColor.red(),
                                               fillColor.alpha() * sourceOpacity / kMaxConstAlpha);
        QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
        vg_lite_draw(vgliteBufferRt,
                     destinationPath->fillPathData(),
                     static_cast<vg_lite_fill_t>(destinationPath->fillRule),
                     &matrix,
                     (blendMode == PlatformInterface::DrawingEngine::BlendMode_SourceOver ? VG_LITE_BLEND_SRC_OVER
                                                                                          : VG_LITE_BLEND_NONE),
                     abgr32.value);
    }

    if (strokeBrush) {
        PlatformInterface::Rgba32 strokeColor = strokeBrush->color();

        const PlatformInterface::Rgba32 abgr32(strokeColor.blue(),
                                               strokeColor.green(),
                                               strokeColor.red(),
                                               strokeColor.alpha() * sourceOpacity / kMaxConstAlpha);
        QUL_ASSERT(vgliteBufferRt, QulError_Vglite_DrawingBufferNotSet);
        vg_lite_draw(vgliteBufferRt,
                     destinationPath->strokePathData(),
                     VG_LITE_FILL_NON_ZERO,
                     &matrix,
                     (blendMode == PlatformInterface::DrawingEngine::BlendMode_SourceOver ? VG_LITE_BLEND_SRC_OVER
                                                                                          : VG_LITE_BLEND_NONE),
                     abgr32.value);
    }
    vg_lite_disable_scissor();
}

void blend_ARGB32(PlatformInterface::DrawingDevice *buffer,
                  const PlatformInterface::Point &pos,
                  const PlatformInterface::Texture &source,
                  const PlatformInterface::Rect &srcRect,
                  int const_alpha)
{
    blendTexture(buffer,
                 pos,
                 source,
                 srcRect,
                 PlatformInterface::Rgba32(uint32_t(0xFF),
                                           uint32_t(0xFF),
                                           uint32_t(0xFF),
                                           uint32_t(const_alpha * 255 / static_cast<int>(kMaxConstAlpha))));
}

void blend_ARGB32_Premultiplied(PlatformInterface::DrawingDevice *buffer,
                                const PlatformInterface::Point &pos,
                                const PlatformInterface::Texture &source,
                                const PlatformInterface::Rect &srcRect,
                                int const_alpha)
{
    blend_ARGB32(buffer, pos, source, srcRect, const_alpha);
}

void blend_RGB32(PlatformInterface::DrawingDevice *buffer,
                 const PlatformInterface::Point &pos,
                 const PlatformInterface::Texture &source,
                 const PlatformInterface::Rect &srcRect,
                 int const_alpha)
{
    blend_ARGB32(buffer, pos, source, srcRect, const_alpha);
}

void blend_RGB888(PlatformInterface::DrawingDevice *,
                  const PlatformInterface::Point &,
                  const PlatformInterface::Texture &,
                  const PlatformInterface::Rect &,
                  int)
{
    debug_bsp("NOT IMPLEMENTED: blend_RGB888(...))\r\n");
}

void blend_RGB16(PlatformInterface::DrawingDevice *,
                 const PlatformInterface::Point &,
                 const PlatformInterface::Texture &,
                 const PlatformInterface::Rect &,
                 int)
{
    debug_bsp("NOT IMPLEMENTED: lend_RGB16(...))\r\n");
}

void blend_Alpha8(PlatformInterface::DrawingDevice *,
                  const PlatformInterface::Point &,
                  const PlatformInterface::Texture &,
                  const PlatformInterface::Rect &,
                  int)
{
    debug_bsp("NOT IMPLEMENTED: blend_Alpha8(...))\r\n");
}

VglitePathDataStroker::VglitePathDataStroker(VglitePath *data)
    : PathDataStroker(data->getPathData())
    , destinationPath(data)
{}

void VglitePathDataStroker::beginStroke()
{
    destinationPath->clearStroke();
}

void VglitePathDataStroker::endStroke()
{
    destinationPath->addStrokeElement(static_cast<int32_t>(VLC_OP_END));
}

void VglitePathDataStroker::lineTo(float x, float y)
{
    destinationPath->addStrokeElement(static_cast<int32_t>(VLC_OP_LINE));
    destinationPath->addStrokeElement(x);
    destinationPath->addStrokeElement(y);
    current.setX(x);
    current.setY(y);
}

void VglitePathDataStroker::moveTo(float x, float y)
{
    destinationPath->addStrokeElement(static_cast<int32_t>(VLC_OP_MOVE));
    destinationPath->addStrokeElement(x);
    destinationPath->addStrokeElement(y);
    current.setX(x);
    current.setY(y);
}

void VglitePathDataStroker::cubicTo(float c1x, float c1y, float c2x, float c2y, float ex, float ey)
{
    destinationPath->addStrokeElement(static_cast<int32_t>(VLC_OP_CUBIC));
    destinationPath->addStrokeElement(c1x);
    destinationPath->addStrokeElement(c1y);
    destinationPath->addStrokeElement(c2x);
    destinationPath->addStrokeElement(c2y);
    destinationPath->addStrokeElement(ex);
    destinationPath->addStrokeElement(ey);
    current.setX(ex);
    current.setY(ey);
}

void VglitePathDataStroker::arcTo(float x, float y, float rx, float ry, float rotation, bool largeArc, bool clockwise)
{
    pathArcToBeziers(&addStrokePathSegment,
                     destinationPath,
                     rx,
                     ry,
                     rotation,
                     largeArc,
                     clockwise,
                     current,
                     PlatformInterface::PointF(x, y));
    current.setX(x);
    current.setY(y);
}

} // namespace Vglite
} // namespace Private
} // namespace Platform
} // namespace Qul
