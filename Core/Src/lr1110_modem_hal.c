/*
 * lr1110_modem_hal.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Alexey Novikov
 */

#include "lr1110_modem_hal.h"
#include "configuration.h"
#include "stm32l0xx_hal.h"


lr1110_modem_hal_status_t lr1110_modem_hal_write( const void* context, const uint8_t* command,
                                                  const uint16_t command_length, const uint8_t* data,
                                                  const uint16_t data_length )
{
    radio_t*                  radio_local = ( radio_t* ) context;
    uint8_t                   crc         = 0;
    lr1110_modem_hal_status_t rc;


    if ( HAL_GPIO_ReadPin( radio_local->busy.port, radio_local->busy.pin) == GPIO_PIN_SET ) {
    	HAL_GPIO_WritePin( radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET );
    	HAL_SPI_Transmit( radio_local->hspi, &crc, 1, 10 );
    	HAL_GPIO_TogglePin( radio_local->nss.port, radio_local->nss.pin );
    }

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
    crc = lr1110_modem_compute_crc( crc, data, data_length );

    HAL_GPIO_WritePin( radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET );
    HAL_SPI_Transmit( radio_local->hspi, command, command_length, 10 );
    HAL_SPI_Transmit( radio_local->hspi, &crc, 1, 10 );
    HAL_SPI_Transmit( radio_local->hspi, data, data_length, 10 );
    HAL_GPIO_TogglePin( radio_local->nss.port, radio_local->nss.pin );

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    HAL_GPIO_WritePin( radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET );
    HAL_SPI_Receive(radio_local->hspi, ( uint8_t* ) &rc, 1, 10);
    HAL_SPI_Receive(radio_local->hspi, ( uint8_t* ) &crc, 1, 10);
    HAL_GPIO_TogglePin( radio_local->nss.port, radio_local->nss.pin );

    return LR1110_MODEM_HAL_STATUS_OK;
}



lr1110_modem_hal_status_t lr1110_modem_hal_read( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    radio_t*                  radio_local = ( radio_t* ) context;
    uint8_t                   crc         = 0;
    lr1110_modem_hal_status_t rc          = 0;
    uint8_t  dummy_byte  = 0x00;

    if ( HAL_GPIO_ReadPin( radio_local->busy.port, radio_local->busy.pin) == GPIO_PIN_SET ) {
    	HAL_GPIO_WritePin( radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET );
    	HAL_SPI_Transmit( radio_local->hspi, &crc, 1, 10 );
    	HAL_GPIO_TogglePin( radio_local->nss.port, radio_local->nss.pin );
    }

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    crc = lr1110_modem_compute_crc( 0xFF, command, command_length );

    HAL_GPIO_WritePin( radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET );
    HAL_SPI_Transmit( radio_local->hspi, command, command_length, 10 );
    HAL_SPI_Transmit( radio_local->hspi, &crc, 1, 10 );
    HAL_GPIO_TogglePin( radio_local->nss.port, radio_local->nss.pin );

    // wait for low BUSY
    while( HAL_GPIO_ReadPin(radio_local->busy.port, radio_local->busy.pin) ) {};

    crc = 0;

    HAL_GPIO_WritePin( radio_local->nss.port, radio_local->nss.pin, GPIO_PIN_RESET );
    // HAL_SPI_Transmit(radio_local->hspi, &dummy_byte, 1, 10);
    HAL_SPI_Receive(radio_local->hspi, ( uint8_t* ) &rc, 1, 10);
    HAL_SPI_Receive(radio_local->hspi, ( uint8_t* ) data, data_length, 10);
    HAL_SPI_Receive(radio_local->hspi, ( uint8_t* ) &crc, 1, 10);
    HAL_GPIO_TogglePin( radio_local->nss.port, radio_local->nss.pin );

    return LR1110_MODEM_HAL_STATUS_OK;
}
