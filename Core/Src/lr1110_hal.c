/*!
 * \file      lr1110_hal.c
 *
 * \brief     HAL implementation for LR1110 radio chip
 */

#include "lr1110_hal.h"
#include "configuration.h"
#include "stm32l0xx_hal.h"


//lr1110_hal_status_t lr1110_hal_bootstrap( const void* radio )
//{
//	radio_t* radio_local = ( radio_t* ) radio;
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	GPIO_InitStruct.Pin = radio_local->busy.pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(radio_local->busy.port, &GPIO_InitStruct);
//
//	HAL_GPIO_WritePin(radio_local->busy.port, radio_local->busy.pin, GPIO_PIN_RESET);
//
//	lr1110_hal_reset(radio);
//	HAL_Delay(500);
//
//	HAL_GPIO_WritePin(radio_local->busy.port, radio_local->busy.pin, GPIO_PIN_SET);
//	HAL_Delay(100);
//
//
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(radio_local->busy.port, &GPIO_InitStruct);
//
//    return LR1110_HAL_STATUS_OK;
//}


lr1110_hal_status_t lr1110_hal_reset( const void* radio )
{
    radio_t* radio_local = ( radio_t* ) radio;

    HAL_GPIO_WritePin(radio_local->reset.port, radio_local->reset.pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(radio_local->reset.port, radio_local->reset.pin);

    return LR1110_HAL_STATUS_OK;
}


lr1110_hal_status_t lr1110_hal_read( const void* radio, const uint8_t* cbuffer,
                                     const uint16_t cbuffer_length,
                                     uint8_t*       rbuffer,
                                     const uint16_t rbuffer_length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    HAL_StatusTypeDef hal_status = HAL_OK;
    uint8_t  dummy_byte  = 0x00;

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    /* 1st SPI transaction */
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    hal_status = HAL_SPI_Transmit(radio_local->hspi, cbuffer, cbuffer_length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_SET);

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    /* 2nd SPI transaction */
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    hal_status = HAL_SPI_Transmit(radio_local->hspi, &dummy_byte, 1, HAL_MAX_DELAY) | hal_status;
    hal_status = HAL_SPI_Receive(radio_local->hspi, rbuffer, rbuffer_length, HAL_MAX_DELAY) | hal_status;
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_SET);

    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    return LR1110_HAL_STATUS_OK | hal_status;
}

lr1110_hal_status_t lr1110_hal_write_read( const void*    radio,
                                           const uint8_t* cbuffer,
                                           uint8_t*       rbuffer,
                                           const uint16_t length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    HAL_StatusTypeDef hal_status = HAL_OK;

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    hal_status = HAL_SPI_TransmitReceive(radio_local->hspi, cbuffer, rbuffer, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_SET);

    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    return LR1110_HAL_STATUS_OK | hal_status;
}

lr1110_hal_status_t lr1110_hal_write( const void* radio,
									  const uint8_t* cbuffer,
                                      const uint16_t cbuffer_length,
                                      const uint8_t* cdata,
                                      const uint16_t cdata_length )
{
    radio_t* radio_local = ( radio_t* ) radio;
    HAL_StatusTypeDef hal_status = HAL_OK;

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    hal_status = HAL_SPI_Transmit(radio_local->hspi, cbuffer, cbuffer_length, HAL_MAX_DELAY);
    if (cdata_length > 0)
    {
    	hal_status = HAL_SPI_Transmit(radio_local->hspi, cdata, cdata_length, HAL_MAX_DELAY) | hal_status;
    }
    HAL_GPIO_WritePin(radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_SET);

    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

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


