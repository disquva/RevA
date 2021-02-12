/*!
 * \file      lr1110_firmware_update.c
 *
 * \brief     LR1110 firmware update implementation
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>

#include "configuration.h"
#include "lr1110_bootloader.h"
#include "lr1110_modem_system.h"
#include "lr1110_firmware_update.h"
#include "lr1110_modem_lorawan.h"
#include "system.h"
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_fw_update_status_t lr1110_update_firmware( void* radio, const uint32_t* buffer, uint32_t length )
{
	lr1110_status_t status = LR1110_STATUS_ERROR;
	lr1110_bootloader_version_t version_bootloader = { 0 };

    lr1110_bootloader_get_version( radio, &version_bootloader );

    // Start flash erase
	// lr1110_bootloader_erase_flash( radio );
	// Flash erase done

	lr1110_bootloader_pin_t      pin      = { 0x00 };
	lr1110_bootloader_chip_eui_t chip_eui = { 0x00 };
	lr1110_bootloader_join_eui_t join_eui = { 0x00 };

	lr1110_bootloader_read_pin( radio, pin );
	lr1110_bootloader_read_chip_eui( radio, chip_eui );
	//lr1110_bootloader_read_join_eui( radio, join_eui );

	// Start flashing firmware
	// status = lr1110_bootloader_write_flash_encrypted_full( radio, 0, buffer, length );
	status = lr1110_bootloader_write_flash_encrypted_full( radio, 163328, buffer, length );
	// Flashing done
	return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
