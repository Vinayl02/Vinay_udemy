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

#include <qul/eventqueue.h>
#include <platform/singlepointtoucheventdispatcher.h>

namespace Qul {
namespace Platform {
namespace Private {

class SinglePointTouchEventQueue : public EventQueue<SinglePointTouchEvent>
{
public:
    virtual void onEvent(const SinglePointTouchEvent &event) { touchEventDispatcher.dispatch(event); }

private:
    SinglePointTouchEventDispatcher touchEventDispatcher;
};

void BOARD_Touch_Init();
void requestTouchData();

} // namespace Private
} // namespace Platform
} // namespace Qul
