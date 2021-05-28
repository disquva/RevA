#include "main.h"
#include "lr1110_hal.h"
#include "configuration.h"
#include "stm32l0xx_hal.h"


lr1110_hal_status_t lr1110_hal_bootstrap( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;
    GPIO_InitTypeDef const GPIO_InitStruct = {0};
    GPIO_InitTypeDef GPIO_config = GPIO_InitStruct;

    // configure BUSY pin as output
    GPIO_config.Pin = BUSY_Pin;
    GPIO_config.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_config.Pull = GPIO_NOPULL;
    GPIO_config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_config);

    // set BUSY low
    HAL_GPIO_WritePin(radio_local->busy.port, radio_local->busy.pin, GPIO_PIN_RESET);

    // set NRESET low
    HAL_GPIO_WritePin(radio_local->reset.port, radio_local->reset.pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    // set NRESET high
    HAL_GPIO_TogglePin(radio_local->reset.port, radio_local->reset.pin);

    HAL_Delay(500);

    HAL_GPIO_TogglePin(radio_local->busy.port, radio_local->busy.pin);

    // set BUSY pin as input
    GPIO_config = GPIO_InitStruct;
    GPIO_config.Pin = BUSY_Pin;
    GPIO_config.Mode = GPIO_MODE_INPUT;
    GPIO_config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_config);

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    return LR1110_HAL_STATUS_OK;
}
