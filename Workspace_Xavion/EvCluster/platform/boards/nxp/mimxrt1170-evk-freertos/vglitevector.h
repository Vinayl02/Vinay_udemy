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

#include <platforminterface/drawingengine.h>
#include <platforminterface/allocator.h>
#include <platforminterface/pathdatastroker.h>
#include <platforminterface/arc.h>

#include <cstddef>
#include <cmath>
#include <array>
#include <cstdint>
#include "vg_lite.h"

namespace Qul {
namespace Platform {
namespace Private {
namespace Vglite {

struct VglitePath : public PlatformInterface::DrawingEngine::Path
{
    const int16_t fillRule;

    VglitePath(const PlatformInterface::PathData *pathData, PlatformInterface::PathFillRule fillRule);

    void free()
    {
        clearFill();
        clearStroke();
        PlatformInterface::qul_delete(this);
    }

    const PlatformInterface::PathData *getPathData() const { return path; }
    vg_lite_path_t *fillPathData(void) { return &vgFillPath; }
    vg_lite_path_t *strokePathData(void) { return &vgStrokePath; }

    inline void addFillElement(int32_t opCode)
    {
        float opConversion;
        // Little-endian byte order makes op-code the first byte and handles alignment requirement
        // of the VGLite API at the same time.
        memcpy(&opConversion, &opCode, sizeof(float));
        fillElements.push_back(opConversion);
    }
    inline void addFillElement(float element) { fillElements.push_back(element); }

    inline void addStrokeElement(int32_t opCode)
    {
        float opConversion;
        // Little-endian byte order makes op-code the first byte and handles alignment requirement
        // of the VGLite API at the same time.
        memcpy(&opConversion, &opCode, sizeof(float));
        strokeElements.push_back(opConversion);
    }
    inline void addStrokeElement(float element) { strokeElements.push_back(element); }

    void processFillPath();
    void processStrokePath();
    int32_t fillPathSize() { return fillElements.size() * sizeof(float); }
    int32_t strokePathSize() { return strokeElements.size() * sizeof(float); }
    void clearStroke();
    void clearFill();

private:
    const PlatformInterface::PathData *const path;
    bool processingDone;
    PlatformInterface::Vector<float> fillElements;
    PlatformInterface::Vector<float> strokeElements;
    vg_lite_path_t vgFillPath;
    vg_lite_path_t vgStrokePath;
};

class VglitePathDataStroker : public PlatformInterface::PathDataStroker
{
public:
    VglitePathDataStroker(VglitePath *data);

protected:
    void beginStroke() override;
    void endStroke() override;
    void lineTo(float x, float y) override;
    void moveTo(float x, float y) override;
    void cubicTo(float c1x, float c1y, float c2x, float c2y, float ex, float ey) override;
    void arcTo(float x, float y, float rx, float ry, float rotation, bool largeArc, bool clockwise) override;

private:
    VglitePath *destinationPath;
    PlatformInterface::PointF current;
};

namespace Vector {
using BoundingBox = std::array<vg_lite_float_t, 4>;

namespace Tags {
template<typename T>
struct Line
{
    constexpr static const T id = 4;
    constexpr static const std::size_t NumberOfArgs = 2;
};

template<typename T>
struct Move
{
    constexpr static const T id = 2;
    constexpr static const std::size_t NumberOfArgs = 2;
};

template<typename T>
struct Cubic
{
    constexpr static const T id = 8;
    constexpr static const std::size_t NumberOfArgs = 6;
};

template<typename T>
struct Finish
{
    constexpr static const T id = 0;
    constexpr static const std::size_t NumberOfArgs = 0;
};
} // namespace Tags

template<typename DataType>
struct CoordinateType;

template<>
struct CoordinateType<std::int8_t>
{
    constexpr static const vg_lite_format_t format = VG_LITE_S8;
};

template<>
struct CoordinateType<std::int16_t>
{
    constexpr static const vg_lite_format_t format = VG_LITE_S16;
};

template<>
struct CoordinateType<std::int32_t>
{
    constexpr static const vg_lite_format_t format = VG_LITE_S32;
};

template<>
struct CoordinateType<float>
{
    constexpr static const vg_lite_format_t format = VG_LITE_FP32;
};

template<typename T, std::size_t N = 3>
struct PathBuilder
{
    static_assert(std::is_same<T, std::int16_t>::value || std::is_same<T, std::int32_t>::value
                      || std::is_same<std::int8_t, T>::value || std::is_same<float, T>::value,
                  "T must be int8, int16, int32 or float");
    PathBuilder() {}

    template<typename T1, std::size_t N1>
    friend class PathBuilder;

    template<std::size_t M>
    PathBuilder(const PathBuilder<T, M> &other)
    {
        static_assert(N >= M, "");
        std::copy(std::begin(other._data), std::end(other._data), std::begin(_data));
    }

    template<std::size_t M>
    PathBuilder<T, N + M> &operator=(const PathBuilder<T, M> &other)
    {
        static_assert(N >= M, "");
        std::copy(std::begin(other._data), std::end(other._data), std::begin(_data));
        return *this;
    }

    decltype(auto) moveTo(T x, T y)
    {
        PathBuilder<T, N + Tags::Move<T>::NumberOfArgs + 1> builder{*this};
        builder.template addCommand<Tags::Move<T> >(_data.size(), x, y);
        return builder;
    }

    decltype(auto) lineTo(T x, T y)
    {
        PathBuilder<T, N + Tags::Line<T>::NumberOfArgs + 1> builder{*this};
        builder.template addCommand<Tags::Line<T> >(_data.size(), x, y);
        return builder;
    }

    decltype(auto) cubicTo(T cx, T cy, T cx2, T cy2, T endX, T endY)
    {
        PathBuilder<T, N + Tags::Cubic<T>::NumberOfArgs + 1> builder{*this};
        builder.template addCommand<Tags::Cubic<T> >(_data.size(), cx, cy, cx2, cy2, endX, endY);
        return builder;
    }

    struct ArcBuilderData
    {
        PathBuilder<T, N + Tags::Cubic<T>::NumberOfArgs + 1> builder;
        std::size_t dataSize;
    };

    static void addSegment(void *data,
                           const PlatformInterface::PointF &cPoint1,
                           const PlatformInterface::PointF &cPoint2,
                           const PlatformInterface::PointF &endPoint)
    {
        const T cx1 = static_cast<T>(cPoint1.x());
        const T cy1 = static_cast<T>(cPoint1.y());
        const T cx2 = static_cast<T>(cPoint2.x());
        const T cy2 = static_cast<T>(cPoint2.y());
        const T cx3 = static_cast<T>(endPoint.x());
        const T cy3 = static_cast<T>(endPoint.y());

        ArcBuilderData *builderData = static_cast<ArcBuilderData *>(data);
        builderData->builder.template addCommand<Tags::Cubic<T> >(builderData->dataSize, cx1, cy1, cx2, cy2, cx3, cy3);
    }

    decltype(auto)
        arcTo(T startX, T startY, T hRadius, T vRadius, T xRotation, bool largeArc, bool sweep, T endX, T endY)
    {
        ArcBuilderData builderData{*this, _data.size()};

        PlatformInterface::PointF startPoint(static_cast<float>(startX), static_cast<float>(startY));
        PlatformInterface::PointF endPoint(static_cast<float>(endX), static_cast<float>(endY));

        PlatformInterface::pathArcToBeziers(&addSegment,
                                            &builderData,
                                            static_cast<float>(hRadius),
                                            static_cast<float>(vRadius),
                                            static_cast<float>(xRotation),
                                            largeArc,
                                            sweep,
                                            startPoint,
                                            endPoint);

        return builderData.builder;
    }

    decltype(auto) finish()
    {
        PathBuilder<T, N + Tags::Finish<T>::NumberOfArgs + 1> builder{*this};
        builder.template addCommand<Tags::Finish<T> >(_data.size());
        return builder;
    }

    vg_lite_path_t path(const BoundingBox &bounding_box) const
    {
        vg_lite_path_t path = {{bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3]},
                               VG_LITE_MEDIUM, // quality
                               CoordinateType<T>::format,
                               {0}, // uploaded
                               // number of bytes, not number of elements
                               static_cast<int32_t>(_data.size() * sizeof(T)),
                               const_cast<T *>(_data.data()),
                               1};
        return path;
    }

private:
    template<typename Tag, typename... Data>
    void addCommand(std::size_t previousIndex, Data... data)
    {
        static_assert(Tag::NumberOfArgs == sizeof...(data), "");
        std::array<T, sizeof...(data) + 1> arr;
        int j = 0;
        arr[j++] = Tag::id;
        using unused = int[];
        (void) unused{0, (arr[j++] = data, 0)...};
        (void) j;
        std::copy(std::begin(arr), std::end(arr), std::begin(_data) + previousIndex);
    }
    std::array<T, N> _data;
};
} // namespace Vector
} // namespace Vglite
} // namespace Private
} // namespace Platform
} // namespace Qul
