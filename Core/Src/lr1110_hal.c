/*!
 * \file      lr1110_hal.c
 *
 * \brief     HAL implementation for LR1110 radio chip
 */

#include "lr1110_hal.h"
#include "configuration.h"
#include "stm32l0xx_hal.h"


lr1110_hal_status_t lr1110_hal_reset( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;

    HAL_GPIO_WritePin(radio_local->reset.port, radio_local->reset.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_TogglePin(radio_local->reset.port, radio_local->reset.pin);

    return LR1110_HAL_STATUS_OK;
}


lr1110_hal_status_t lr1110_hal_read( const void* radio, const uint8_t* cbuffer,
                                     const uint16_t cbuffer_length,
                                     uint8_t*       rbuffer,
                                     const uint16_t rbuffer_length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    uint8_t  dummy_byte  = 0x00;

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    /* 1st SPI transaction */
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio_local->hspi, cbuffer, cbuffer_length, 10);
    HAL_GPIO_TogglePin(radio_local->nss.port, radio_local->nss.pin);

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    /* 2nd SPI transaction */
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio_local->hspi, &dummy_byte, 1, 10);
    HAL_SPI_Receive(radio_local->hspi, rbuffer, rbuffer_length, 10);
    HAL_GPIO_TogglePin(radio_local->nss.port, radio_local->nss.pin);

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_write_read( const void*    radio,
                                           const uint8_t* cbuffer,
                                           uint8_t*       rbuffer,
                                           const uint16_t length )
{
    radio_t* radio_local = ( radio_t* ) radio;

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(radio_local->hspi, cbuffer, rbuffer, length, 10);
    HAL_GPIO_TogglePin(radio_local->nss.port, radio_local->nss.pin);

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_write( const void* radio, const uint8_t* cbuffer,
                                      const uint16_t cbuffer_length,
                                      const uint8_t* cdata,
                                      const uint16_t cdata_length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    HAL_StatusTypeDef hal_status = HAL_OK;

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_Transmit(radio_local->hspi, cbuffer, cbuffer_length, 10);
    hal_status = HAL_SPI_Transmit(radio_local->hspi, cdata, cdata_length, 10) | hal_status;
    HAL_GPIO_TogglePin(radio_local->nss.port, radio_local->nss.pin);

    return LR1110_HAL_STATUS_OK | hal_status;
}

lr1110_hal_status_t lr1110_hal_wakeup( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;

    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_TogglePin(radio_local->nss.port, radio_local->nss.pin);

    return LR1110_HAL_STATUS_OK;
}


