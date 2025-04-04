
#include <qul/application.h>
#include <qul/qul.h>
#include <platforminterface/log.h>
#include <FreeRTOS.h>
#include <task.h>
#include <board.h>
#include <hmimain.h>
#include "fsl_debug_console.h"
#include "fsl_iomuxc.h"
#include "cluster_controller_platform.h"

static void Qul_Thread(void *argument);
static void ControllerPlatform_Thread(void *argument);

int main()
{
    Qul::initHardware();
    Qul::initPlatform();
    if (xTaskCreate(Qul_Thread, "Qul_Thread", (63*1024), 0, 3, 0) != pdPASS) {
        Qul::PlatformInterface::log("Task creation failed!.\r\n");
        configASSERT(false);
    }

    if (xTaskCreate(ControllerPlatform_Thread, "ControllerPlfThread",  (63*1024), 0, 3, 0) != pdPASS) {
        Qul::PlatformInterface::log("ControllerPlatform_Thread Task creation failed!.\r\n");
        configASSERT(false);
    }

    vTaskStartScheduler();

    return 1;
}

static void Qul_Thread(void *argument)
{
    (void) argument;
    Qul::Application _qul_app;
    static struct ::hmimain _qul_item;
    _qul_app.setRootItem(&_qul_item);

#ifdef APP_DEFAULT_UILANGUAGE
    _qul_app.settings().uiLanguage.setValue(APP_DEFAULT_UILANGUAGE);
#endif

    _qul_app.exec();
}

static void ControllerPlatform_Thread(void *argument)
{
	ControllerPlatform::Start();
}


extern "C" {
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    (void) xTask;
    (void) pcTaskName;

    Qul::PlatformInterface::log("vApplicationStackOverflowHook");
    configASSERT(false);
}

void vApplicationMallocFailedHook(void)
{
    Qul::PlatformInterface::log("vApplicationMallocFailedHook");
    configASSERT(false);
}
}
