#include<stdio.h>
#include<iostream>
#include"stm32f4xx.h"
#include"stm32f4xx_gpio.h"

HAL_UART_HandleTypeDef huart2;

uint8_t data[]  = "Hello World";

int main()
{
    while(1)
    {
        HAL_UART_Transmit(&huart2, data, sizeof(data), 1000);
        HAL_GetInfo();  //Provides the detailed information of the microcontroller
        HAL_Getdata();
        HAL_Delay(1000);
    } 

    return 0;
}
uint32_t HAL_Getdata(void)
{
    uint8_t num = 123;
    uint8_t numarr[5];
    while(1)
    {
        sprintf(numarr,"%d\n",num););
        HAL_UART_Transmit(&huart2, data, sizeof(data), 1000);
        HAL_Delay(1000);
    }
}
///////////////////////////////////////
uint8_t txdata[10240];

int main()
{
    for(uint8_t i = 0;i<10240;i++)
    {
        txdata[i] = i & (0xff);
    }

    while(1)
    HAL_UART_Transmit(&huart,txdata,10240,HAL_MAX_DELAY);
    HAL_GPIO_TooglePin(GPIOA,GPIO_PIN_10);
    HAL_DELAY(1000);
}

int isSent = 1;
int countinterrupt = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    isSent = 1;
    countinterrupt++;
}
int main()
{
    int countloop = 0;
    while(1)
    {
        (isSent == 1)
        {
            HAL_UART_Transmit_IT(&huart2, data, sizeof(data));
            isSent = 0;
        }
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);//Toggle the LED
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);// Set the LED
        HAL_UART_Transmit_IT(&huart2, data, sizeof(data));
        //HAL_UART_Transmit_DMA(&huart2, data, sizeof(data));
        HAL_Delay(1000);
        countloop++;
    }
}


int indx = 49;
void HAL_UART_Tx_HalfCpltCallback(UART_HandleTypeDef *huart)
{
    for(int i=0;i<5024;i++)
    {
        txdata[i] = indx;
    }
    indx++;
}

void HAL_UART_Tx_CpltCallback(UART_HandleTypeDef *huart)
{
    for(int i =0; i<5024;i++)
    {
        txdata[i] = indx;
    }
    indx++;

    if(indx >= 60)
    {
        HAL_UART_Stop_DMA(&huart2);
    }
    isSent = 1;
    countinterrupt++;
}
