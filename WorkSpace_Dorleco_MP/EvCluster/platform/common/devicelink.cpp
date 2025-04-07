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
#include <platforminterface/log.h>
#include <platform/devicelink.h>
#include <platform/platform.h>
#include <qul/private/application.h>
#include <qul/private/layers/layers.h>

#ifndef QUL_PLATFORM_SKIP_PLATFORM_CONFIG
#include "platform_config.h"
#endif

#ifdef QUL_PLATFORM_DEVICELINK_ENABLED
#include <qul/private/devicelink.h>

// ProtoBuf headers
#include <pb_encode.h>
#include <pb_decode.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "deviceinformationexchangeprotocol.pb.h"
#include "minihdlc.h"
#ifdef __cplusplus
}
#endif

#define QOI_IMPLEMENTATION
#define QOI_NO_STDIO
#include <qoi.h>

static_assert(DeviceToHostMessage_size <= MINIHDLC_MAX_FRAME_LENGTH, "HLDC buffer too small to hold entire message");
static_assert(HostToDeviceMessage_size <= MINIHDLC_MAX_FRAME_LENGTH, "HLDC buffer too small to hold entire message");

static bool hdlcEnabled = false;
static TouchEvent touchSequence[TouchSequence_size / sizeof(TouchEvent)];
static uint64_t touchTimestamp = std::numeric_limits<uint64_t>::max();
static uint16_t touchCount = 0;
static uint16_t touchIndex = 0;

static void SendCharWrapper(uint8_t data)
{
    Qul::Platform::getDeviceLinkInterface()->transmitChars(&data, 1);
}

namespace {

/*!
    \brief Called for a message to be transferred to the host

    When a protocol message is ready to be sent this function will use
    the HDLC interface to send frames over the serial port.
    \a message is the message to be sent.

    \internal
*/
void sendMessage(const DeviceToHostMessage *message)
{
    uint8_t buffer[DeviceToHostMessage_size];

    hdlcEnabled = true;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    /* Now we are ready to encode the message! */
    bool status = pb_encode(&stream, DeviceToHostMessage_fields, message);
    size_t message_length = stream.bytes_written;

    /* Check for errors... */
    if (!status) {
        Qul::PlatformInterface::log("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }

    minihdlc_send_frame(buffer, message_length);
}

void handleGetFramebufferData(const GetFramebufferData &message)
{
    auto app = Qul::Private::Application::instance();
    if (!app)
        return;

    DeviceToHostMessage reply = DeviceToHostMessage_init_zero;

    const Qul::PlatformInterface::LayerEngine::ItemLayer *platformLayer = NULL;
    if (Qul::Platform::getPlatformInstance()->layerEngine()) {
        auto layer = Qul::Private::Items::ItemLayer::asItemLayer(app->layerForPlatformId(message.platformId));
        if (layer) {
            platformLayer = layer->node.platformInstance();
            if (!platformLayer)
                Qul::PlatformInterface::log("DeviceLink: The item layer with platform id %d is not instantiated, "
                                            "meaning it's not enabled or it hasn't been initialized yet\n",
                                            message.platformId);
        } else {
            Qul::PlatformInterface::log("DeviceLink: No item layer found with the platform id %d\n", message.platformId);
        }

        if (!platformLayer) {
            reply.which_alternative = DeviceToHostMessage_dataTransferFailed_tag;
            sendMessage(&reply);
            return;
        }
    }

    auto format = Qul::Platform::getDeviceLinkInterface()->framebufferFormat(platformLayer);

    reply.which_alternative = DeviceToHostMessage_dataTransfer_tag;

    struct
    {
        uint32_t index;
        uint32_t remainingBits;
        uint8_t *address;
    } posData = {0, 8, reinterpret_cast<uint8_t *>(format.address)};

    auto fetch = [&posData, &format]() -> uint32_t {
        uint32_t pixelValue = 0;

        uint32_t num = format.bitsPerPixel;
        while (num > 0) {
            if (posData.remainingBits == 0) {
                posData.remainingBits += 8;
                ++posData.index;

                if (posData.index >= format.width * format.bitsPerPixel / 8) {
                    posData.index = 0;
                    posData.address += format.bytesPerLine;
                }
            }

            uint32_t count = std::min(num, posData.remainingBits);
            pixelValue <<= count;

            int swappedIndex = posData.index;
            if (format.swapBytes == 2) {
                swappedIndex ^= 1;
            } else if (format.swapBytes == 4) {
                swappedIndex = (swappedIndex & ~0x3) | (3 - (swappedIndex & 0x3));
            }

            pixelValue |= (posData.address[swappedIndex]) & ((1 << count) - 1);
            posData.remainingBits -= count;
            num -= count;
        };

        auto getColorChannel =
            [](uint32_t pixel, int bitsPerPixel, const Qul::Platform::FramebufferFormat::BitField &field) -> uint32_t {
            uint32_t value = (pixel >> (bitsPerPixel - field.offset - field.length)) & ((1 << field.length) - 1);
            if (field.length < 8) {
                int32_t lowBits = 8 - field.length;
                value <<= lowBits;

                // Repeat the set bits, e.g. ABCXXXXX becomes ABCABCAB
                uint32_t shift = field.length;
                while (lowBits > 0) {
                    value |= (value >> shift);
                    shift *= 2;
                    lowBits -= shift;
                }
            }
            return value;
        };

        uint32_t r = getColorChannel(pixelValue, format.bitsPerPixel, format.redChannel);
        uint32_t g = getColorChannel(pixelValue, format.bitsPerPixel, format.greenChannel);
        uint32_t b = getColorChannel(pixelValue, format.bitsPerPixel, format.blueChannel);

        return 0xff000000 | (r << 16) | (g << 8) | b;
    };

    uint32_t chunkSize = 0;
    auto emit = [&chunkSize, &reply](uint32_t value) {
        if (chunkSize >= sizeof(reply.alternative.dataTransfer.data.bytes)) {
            reply.alternative.dataTransfer.data.size = chunkSize;
            sendMessage(&reply);
            chunkSize = 0;
        }
        *reinterpret_cast<uint32_t *>(&reply.alternative.dataTransfer.data.bytes[chunkSize++]) = value;
    };

    qoi_desc desc = {format.width, format.height, 3, QOI_SRGB};
    qoi_encode(&desc, fetch, emit);

    if (chunkSize) {
        reply.alternative.dataTransfer.data.size = chunkSize;
        sendMessage(&reply);
    }

    reply.which_alternative = DeviceToHostMessage_dataTransferDone_tag;
    sendMessage(&reply);
}

} // namespace

Qul::Platform::DeviceLink *Qul::Platform::DeviceLink::instance()
{
    static Qul::Platform::DeviceLink mInstance;
    return &mInstance;
}

uint64_t Qul::Platform::DeviceLink::injectTouchEvent()
{
    const auto timestamp = Qul::Platform::getPlatformInstance()->currentTimestamp();

    if (touchCount == 0 || touchIndex >= touchCount || timestamp < touchTimestamp)
        return touchTimestamp;

    Qul::PlatformInterface::TouchPoint tp(touchSequence[touchIndex].x,
                                          touchSequence[touchIndex].y,
                                          Qul::PlatformInterface::TouchPoint::Moved);
    if (touchIndex == 0)
        tp.state = Qul::PlatformInterface::TouchPoint::Pressed;
    else if (touchIndex + 1 == touchCount)
        tp.state = Qul::PlatformInterface::TouchPoint::Released;

    handleTouchEvent(NULL, touchTimestamp, &tp, 1);

    touchIndex++;
    if (touchIndex >= touchCount) {
        // End of data
        touchCount = 0;
        touchIndex = 0;
        touchTimestamp = std::numeric_limits<uint64_t>::max();
    } else {
        touchTimestamp += touchSequence[touchIndex].timestamp;
    }

    return touchTimestamp;
}

void Qul::Platform::DeviceLink::frame_handler(const uint8_t *frame_buffer, uint16_t frame_length)
{
    /* Allocate space for the decoded message. */
    HostToDeviceMessage message = HostToDeviceMessage_init_zero;

    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(frame_buffer, frame_length);

    /* Now we are ready to decode the message. */
    bool status = pb_decode(&stream, HostToDeviceMessage_fields, &message);

    /* Check for errors... */
    if (!status) {
        Qul::PlatformInterface::log("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }

    switch (message.which_alternative) {
    case HostToDeviceMessage_touchSequence_tag: {
        if (touchCount != 0)
            Qul::PlatformInterface::log("New touch sequence received while still in progress.\n");

        touchCount = message.alternative.touchSequence.touchEvents_count;
        memcpy(touchSequence, &message.alternative.touchSequence.touchEvents, sizeof(TouchEvent) * touchCount);
        touchIndex = 0;
        touchTimestamp = Qul::Platform::getPlatformInstance()->currentTimestamp() + touchSequence[touchIndex].timestamp;

        Qul::Private::Application *instance = Qul::Private::Application::instance();
        if (instance) {
            instance->requestEventProcessing();
        }
        break;
    }
    case HostToDeviceMessage_getFramebufferData_tag:
        handleGetFramebufferData(message.alternative.getFramebufferData);
        break;
    default:
        QUL_ASSERT(false, QulError_DeviceLink_UnsupportedMessageType, message.which_alternative);
    }
}

namespace {
class DeviceLinkReceiveQueue
    : public Qul::EventQueue<uint8_t, Qul::EventQueueOverrunPolicy_Discard, 100> // Up to: MINIHDLC_MAX_FRAME_LENGTH
{
    void onEvent(const uint8_t &c) override { minihdlc_char_receiver(c); }
    void onQueueOverrun() override
    {
        clearOverrun();
        Qul::PlatformInterface::log("Device link receiver queue overrun. Consider increasing the queue size.\n");
    }
};

DeviceLinkReceiveQueue &deviceLinkReceiveQueue()
{
    static DeviceLinkReceiveQueue instance;
    return instance;
}
} // namespace

void Qul::Platform::DeviceLink::init()
{
    deviceLinkReceiveQueue(); // This initializes the static variable. It should not be initialized when called from an interrupt.
    Qul::Private::setTouchInjectionCallback([]() -> uint64_t {
        Qul::Platform::DeviceLink::instance()->injectTouchEvent();
        return touchTimestamp;
    });
    minihdlc_init(&SendCharWrapper, &frame_handler);
    Qul::Platform::getDeviceLinkInterface()->platformInit();
}

// This function is called from an interrupt
void Qul::Platform::DeviceLink::charsReceived(const uint8_t *data, int size)
{
    hdlcEnabled = true;
    while (size--)
        deviceLinkReceiveQueue().postEventFromInterrupt(*data++);
}

// Buffer for printing messages on terminal
static char
    messageBuffer[80]; // Maximum message size. If changed, do it in deviceinformationexchangeprotocol.options as well.
static unsigned int messageSize = 0;
static char *messagePointer = messageBuffer;

void Qul::Platform::DeviceLink::printMessage(char *message, int length)
{
    for (int i = 0; i < length; i++) {
        *messagePointer = *message;
        messageSize++;

        // There need to be size for \r and \0 left in the message
        if (*message == '\n' || messageSize >= sizeof(messageBuffer) - 2) {
            if (*message == '\n') {
                // Add additional carriage return
                messagePointer++;
                *messagePointer = '\r';
                messageSize++;
            }

            if (hdlcEnabled) {
                // Zero terminate the message
                messagePointer++;
                *messagePointer = '\0';
                messageSize++;

                DeviceToHostMessage messageEnvelope = DeviceToHostMessage_init_zero;
                // Maybe use variable length callbacks as alternative
                messageEnvelope.which_alternative = DeviceToHostMessage_consoleLog_tag;
                memcpy(&messageEnvelope.alternative.consoleLog.data, messageBuffer, messageSize);
                sendMessage(&messageEnvelope);
            } else {
                Qul::Platform::getDeviceLinkInterface()->transmitChars((uint8_t *) messageBuffer, messageSize);
            }
            messagePointer = messageBuffer;
            messageSize = 0;
        } else {
            messagePointer++;
        }

        message++;
    }
}

#else  // QUL_PLATFORM_DEVICELINK_ENABLED

Qul::Platform::DeviceLink *Qul::Platform::DeviceLink::instance()
{
    return NULL;
}

uint64_t Qul::Platform::DeviceLink::injectTouchEvent()
{
    QUL_ASSERT(false, QulError_DeviceLink_ShouldNotBeCalled);
    return std::numeric_limits<uint64_t>::max();
}

void Qul::Platform::DeviceLink::init()
{
    QUL_ASSERT(false, QulError_DeviceLink_ShouldNotBeCalled);
    return;
}

void Qul::Platform::DeviceLink::charsReceived(const uint8_t *data, int size)
{
    QUL_UNUSED(data);
    QUL_UNUSED(size);
    QUL_ASSERT(false, QulError_DeviceLink_ShouldNotBeCalled);
    return;
}

void Qul::Platform::DeviceLink::printMessage(char *message, int length)
{
    QUL_UNUSED(message);
    QUL_UNUSED(length);
    QUL_ASSERT(false, QulError_DeviceLink_ShouldNotBeCalled);
    return;
}
#endif // QUL_PLATFORM_DEVICELINK_ENABLED

/*!
    \ingroup platformapi
    \headerfile <platform/devicelink.h>

    \title Platform device link functionality

    \brief Provides device link functionality for the platform.

    The device link component enables communication between host and device. It
    allows them to exchange touch, screenshot, performance, or log data, for
    debugging or testing purposes.

    \sa{Porting DeviceLink Communication}

    \section1 Classes or structs
    \annotatedlist header-devicelink
*/

/*!
    \class Qul::Platform::DeviceLinkInterface
    \relatedheader devicelink
    \inheaderfile platform/devicelink.h
    \since \QULPlatform 2.3

    \brief This provides the interface to implement by the platform.

    Each platform using the device link has to provide an implementation of this interface.

    \sa Qul::Platform::DeviceLink
    \sa{Porting DeviceLink Communication}
*/

/*!
    \fn void Qul::Platform::DeviceLinkInterface::transmitChars(const uint8_t *data, uint32_t size)

    \brief Send a data stream to the host after it has been packed and framed.

    This function is called when there is message data to be transferred to the host.
    The implementation has to use the serial port write functions to send the \a data with \a size.
*/

/*!
    \fn void Qul::Platform::DeviceLinkInterface::platformInit()

    \brief Perform last minute platform specific setup.

    This function is called after the DeviceLink component is ready to send and receive data.
    The implementation may enable the serial port interrupts or set other flags that
    enable sending and receiving actual data.
*/

/*!
    \fn Qul::Platform::FramebufferFormat Qul::Platform::DeviceLinkInterface::framebufferFormat(const PlatformInterface::LayerEngine::ItemLayer *layer)

    \brief Get information about a framebuffer for the given \a layer.

    This is used by the device link implementation to read the data and transfer
    it to the host when a screenshot is requested.

    \note The \a layer parameter can be ignored if the platform does not support
    multiple layers.
*/

/*!
    \fn uint64_t Qul::Platform::DeviceLink::injectTouchEvent()
    \brief Injects a touch event after a touch sequence message has been received.

    When a message with a touch sequence has been received this function is called
    to inject a single event from the buffer to the event loop and remove it from the queue.
    This function has to be called from the main loop at the given time being returned from
    nextTouchTimestamp.

    After a touch sequence message has been received, this function returns
    the timestamp at which the next touch event has to be injected by calling
    \l injectTouchEvent.
    When there are no more touch events in the current sequence this function
    returns the maximum integer value.
*/

/*!
    \fn void Qul::Platform::DeviceLink::printMessage(char *message, int length)
    \brief Sends a logging message to the host

    The logging facilities of \QUL are using this function to send a log message to the host.
    If the transfer protocol has been enabled it will be wrapped in the protocol and framing.
    If the transfer protocol has not been enabled yet, it will be sent over the serial port as is.
    \a message and \a length specify the string to be sent.
*/

/*!
    \class Qul::Platform::DeviceLink
    \relatedheader devicelink
    \inheaderfile platform/devicelink.h
    \since \QULPlatform 2.3

    \brief This class provides functionality for communication between host and device.

    Methods of this class provide the ability for communition between the host machine and the device
    by sending messages over the serial port of a device.
    It uses ProtoBuf and HDLC for sending and framing the messages.

    The protocol and framing will be disabled until the first byte from the host is received.
    This allows to see any log output in the terminal without having to use special tools.
    Once the first byte from the host is received, the protocol and framing is turned on so that
    in order to read debugging messages, there are tools required to decode them.

    \sa Qul::Platform::DeviceLinkInterface
*/

/*!
    \fn void Qul::Platform::DeviceLink::charsReceived(const uint8_t *data, int size)
    \brief Decodes and processes data received on the serial port.

    This function will be called from \l Qul::PlatformInterface::deviceLinkBytesReceived and handles the data
    specified by \a data and \a size.

    \sa{Porting DeviceLink Communication}
*/

/*!
    \fn void Qul::Platform::DeviceLink::init()
    \brief Initializes the device link capability.

    This function has to be called after the serial communication hardware has been initialized and is
    ready to be used.
*/

/*!
    \fn Qul::Platform::getDeviceLinkInterface *Qul::Platform::getDeviceLinkInterface()

    \brief Provides the device link interface implementation for the platform.

    If there is no such imlementation the function can return \c nullptr. But ideally, when the device link
    is not compiled in, this function should not be called at all.

    \sa QUL_PLATFORM_DEVICELINK_ENABLED
*/

/*!
    \ingroup platformapi
    \headerfile <platform/framebufferformat.h>

    \title Framebuffer format

    \brief Provides information about framebuffer format for use with the device
    link protocol.

    \sa{Porting DeviceLink Communication}

    \section1 Classes or structs
    \annotatedlist header-framebufferformat
*/

/*!
    \class Qul::Platform::FramebufferFormat
    \relatedheader framebufferformat
    \inheaderfile platform/framebufferformat.h
    \since \QULPlatform 2.3

    \brief This struct contains information about a framebuffer.

    The data in this struct is used to interpret the contents of a framebuffer.

    \sa Qul::Platform::DeviceLinkInterface::framebufferFormat()
    \sa{Porting DeviceLink Communication}
*/

/*!
    \variable Qul::Platform::FramebufferFormat::address

    \brief The address of the framebuffer.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::width

    \brief The width in pixels of the framebuffer.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::height

    \brief The height in pixels of the framebuffer.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::bitsPerPixel

    \brief The bits per pixel of the framebuffer.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::bytesPerLine

    \brief The bytes per line (stride) of the framebuffer.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::redChannel

    \brief Offset and length of the bits in the red color channel.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::greenChannel

    \brief Offset and length of the bits in the green color channel.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::blueChannel

    \brief Offset and length of the bits in the blue color channel.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::swapBytes

    \brief Group size of bytes to swap.

    If set to 0, no swapping is done. If set to 2, every pair of bytes are
    swapped. If set to 4, every set of four bytes are reversed, so that BGRA
    becomes ARGB.
*/

/*!
    \class Qul::Platform::FramebufferFormat::BitField

    \brief This struct contains information about the bits of a color channel in a framebuffer.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::BitField::offset

    \brief The offset of the bits.
*/

/*!
    \variable Qul::Platform::FramebufferFormat::BitField::length

    \brief The number of bits.
*/
