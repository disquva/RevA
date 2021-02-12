/*!
 * \file      configuration.h
 *
 * \brief     Configuration definition
 */


#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H


/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "stm32l071xx.h"
#include "stm32l0xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct configuration
{
    GPIO_TypeDef* port;
    uint32_t      pin;
} gpio_t;

typedef struct
{
	SPI_HandleTypeDef*	hspi;
    gpio_t       		nss;
    gpio_t       		reset;
    gpio_t       		irq;
    gpio_t       		busy;
} radio_t;

#endif
