#include <stdio.h>
#include<string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "uart.h"
#include "gpio.h"

osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;

UART_HandleType_Def huart2;

osMutexid mutex1Handle;
semaphore_t sem1Handle;

osTHreadDef(Task1, Task1_init,osPriorityNormal,0,128);
osTHreadDef(Task2, Task2_init,osPriorityNormal,0,128);
osTHreadDef(Task3, Task3_init,osPriorityNormal,0,128);

Task1Handle = osThreadCreate(osThread(Task1),NULL);
Task2Handle = osThreadCreate(osThread(Task2),NULL);
Task3Handle = osThreadCreate(osThread(Task3),NULL);

mutex1Handle = osMutexCreate(osMutex(Mutex1));
sem1Handle = osSemaphoreCreate(osSemaphore(Sema1),1);

void Task1_init(void Const* arg);
void Task2_init(void Const* arg);
void Task3_init(void Const* arg);

void Task1_init(void Const* arg)
{
    str1 = "Test : 1 \r\n";
    printf("Task 1 is running \r\n");
    HAL_Toggle_Pin(GPIOA,GPIO_PIN_10);
    HAL_UART_Transmit(&huart,str1,strlen(str1),HAL_MAX_DELAY);
    osMutexWait(mutex1Handle,port_MAX_DELAY);
    osDelay(1000);
    osMutexRelease(mutex1Handle);    
}

void Task2_init(void Const* arg)
{
    str1 = "Test : 2 \r\n";
    printf("Task 2 is running \r\n");
    HAL_Toggle_Pin(GPIOA,GPIO_PIN_11);
    HAL_UART_Transmit(&huart,str1,strlen(str1),HAL_MAX_DELAY);
    osMutexWait(mutex1Handle,port_MAX_DELAY);
    osDelay(1000);
    osMutexRelease(mutex1Handle); 
}

void Task3_init(void Const* arg)
{

    str1 = "Test : 3 \r\n";
    printf("Task 3 is running \r\n");
    HAL_Toggle_Pin(GPIOA,GPIO_PIN_09);
    HAL_UART_Transmit(&huart,str1,strlen(str1),HAL_MAX_DELAY);
    osMutexWait(mutex1Handle,port_MAX_DELAY);
    osDelay(1000);
    osMutexRelease(mutex1Handle);
}

