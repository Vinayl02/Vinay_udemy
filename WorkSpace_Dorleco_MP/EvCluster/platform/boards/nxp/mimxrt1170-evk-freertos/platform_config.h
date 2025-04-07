#pragma once

#ifndef QUL_PLATFORM_DEVICELINK_ENABLED
// Enable the device link component.
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_DEVICELINK_ENABLED page.
/* #undef QUL_PLATFORM_DEVICELINK_ENABLED */
#endif

#ifndef QUL_ENABLE_PERFORMANCE_LOGGING
// Enable performance logging for QUL rendering, such as render times and FPS.
// For more details, see Qt Quick Ultralite Performance Logging page.
// NOTE: This define will only enable performance logging from platform side. You'll also need to rebuild QUL Core with
//       this define to fully enable/disable this.
/* #undef QUL_ENABLE_PERFORMANCE_LOGGING */
#endif
#ifndef QUL_ENABLE_HARDWARE_PERFORMANCE_LOGGING
// Enable logging of hardware performance statistics, such as CPU usage.
/* #undef QUL_ENABLE_HARDWARE_PERFORMANCE_LOGGING */
#endif

#ifndef QUL_COLOR_DEPTH
// Defines the color depth used by the platform. Supported values depend on the platform but Qt Quick Ultralite
// supports the following bit depths: 8, 16, 24 and 32.
// For more details, see Qt for MCUs CMake manual QUL_COLOR_DEPTH page.
#define QUL_COLOR_DEPTH 32
#endif

#ifndef QUL_PLATFORM_DEFAULT_TEXT_CACHE_ENABLED
// Enable text cache.
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_DEFAULT_TEXT_CACHE_ENABLED page.
#define QUL_PLATFORM_DEFAULT_TEXT_CACHE_ENABLED 1
#endif
#ifndef QUL_PLATFORM_DEFAULT_TEXT_CACHE_SIZE
// The default text cache size for the platform in bytes. The size should not exceed the size of the heap. Default
// value is 24*1024.
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_DEFAULT_TEXT_CACHE_SIZE page.
#define QUL_PLATFORM_DEFAULT_TEXT_CACHE_SIZE 262144
#endif
#ifndef QUL_PLATFORM_DEFAULT_NUM_FRAMES_TO_PRESERVE_ASSETS
// Default number of frames required to preserve assets in text cache. Default value is 0.
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_DEFAULT_NUM_FRAMES_TO_PRESERVE_ASSETS page.
#define QUL_PLATFORM_DEFAULT_NUM_FRAMES_TO_PRESERVE_ASSETS 1
#endif

#ifndef QUL_PLATFORM_REQUIRED_IMAGE_ALIGNMENT
// The minimum alignment required for image data on the given platform. The value should be a power of 2. Default value
// is 1.
// The value should match the value set in BoardDefaults*.qmlprojectconfig.
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_REQUIRED_IMAGE_ALIGNMENT page.
#define QUL_PLATFORM_REQUIRED_IMAGE_ALIGNMENT 8
#endif
#ifndef QUL_PLATFORM_REQUIRED_PIXEL_WIDTH_ALIGNMENT
// The image width will be a multiple of this value on the given platform. Default value is 1.
// The value should match the value set in BoardDefaults*.qmlprojectconfig.
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_REQUIRED_PIXEL_WIDTH_ALIGNMENT page.
#define QUL_PLATFORM_REQUIRED_PIXEL_WIDTH_ALIGNMENT 16
#endif

#ifndef QUL_PLATFORM_RENDER_BATCH_HEIGHT
// The render batch height on the given platform. Default value is 0 (disabled).
// For more details, see Qt for MCUs CMake manual QUL_PLATFORM_RENDER_BATCH_HEIGHT page and
// for QmlProject MCU.Config.platformRenderBatchHeight property.
#define QUL_PLATFORM_RENDER_BATCH_HEIGHT 0
#endif
/********** mimxrt1170-evk-freertos specific properties **********/
// Display panel defines supported in Qt Quick Ultralite
#define MIPI_PANEL_RK055AHD091 0 /* RK055HDMIPI4M display https://www.nxp.com/part/RK055HDMIPI4M#/ */
#define MIPI_PANEL_RK055MHD091 1 /* RK055HDMIPI4MA0 display https://www.nxp.com/part/RK055HDMIPI4MA0#/ */

#ifndef USE_MIPI_PANEL
// Display used in the mimxrt1170-evk board. Defaults to MIPI_PANEL_RK055MHD091 if no panel type was explicitly set.
#define USE_MIPI_PANEL MIPI_PANEL_RK055MHD091
#endif
