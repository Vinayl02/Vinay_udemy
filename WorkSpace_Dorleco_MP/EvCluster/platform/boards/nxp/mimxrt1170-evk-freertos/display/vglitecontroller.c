/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "display/vglitecontroller.h"
#include "fsl_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vg_lite.h"
#include "vg_lite_platform.h"

#define MAX_CONTIGUOUS_SIZE 0x200000
#define VG_LITE_COMMAND_BUFFER_SIZE (256 << 10)
#define MEMORY_ALIGN 32

static uint32_t registerMemBase = 0x41800000;
static uint32_t gpu_mem_base    = 0x0;

/*
 * In case custom VGLite memory parameters are used, the application needs to
 * allocate and publish the VGLite heap base, its size and the size of the
 * command buffer(s) using the following global variables:
 */
extern void *vglite_heap_base;
extern uint32_t vglite_heap_size;
extern uint32_t vglite_cmd_buff_size;

#if (CUSTOM_VGLITE_MEMORY_CONFIG == 0)
/* VGLite driver heap */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t contiguous_mem[MAX_CONTIGUOUS_SIZE], MEMORY_ALIGN);

void *vglite_heap_base        = &contiguous_mem;
uint32_t vglite_heap_size     = MAX_CONTIGUOUS_SIZE;
uint32_t vglite_cmd_buff_size = VG_LITE_COMMAND_BUFFER_SIZE;
#endif /* CUSTOM_VGLITE_MEMORY_CONFIG */

static status_t BOARD_InitVGliteClock(void)
{
    const clock_root_config_t gc355ClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Gc355, &gc355ClockConfig);

    CLOCK_GetRootClockFreq(kCLOCK_Root_Gc355);

    CLOCK_EnableClock(kCLOCK_Gpu2d);

    NVIC_SetPriority(GPU2D_IRQn, 3);

    EnableIRQ(GPU2D_IRQn);

    return kStatus_Success;
}

status_t BOARD_PrepareVGLiteController(void)
{
    status_t status;

    status = BOARD_InitVGliteClock();

    if (kStatus_Success != status)
    {
        return status;
    }

    vg_lite_init_mem(registerMemBase, gpu_mem_base, vglite_heap_base, vglite_heap_size);

    vg_lite_set_command_buffer_size(vglite_cmd_buff_size);

    return kStatus_Success;
}
