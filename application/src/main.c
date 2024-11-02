/*!
 * @file      main.c
 *
 * @brief     LR11XX updater tool application entry point
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#define IMAGE_HEADER_FILE 

#if defined IMAGE_HEADER_FILE
#include "lr1121_modem_05020001.h"
#else
#error IMAGE_HEADER_FILE is not defined, please define it or include firmware image instead of this message
#endif

#include <stdint.h>
#include <stdbool.h>

#include <string.h>
#include <stdlib.h>

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include <lora_lbm_transceiver.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <smtc_modem_hal_init.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, 4);

#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(lr11xx_board, CONFIG_LORA_BASICS_MODEM_DRIVERS_LOG_LEVEL);

#include "ralf_lr11xx.h"
#include "lr11xx_bootloader.h"
#include "lr11xx_system.h"
#include "lr11xx_firmware_update.h"
// #include "lr1110_modem_lorawan.h"
#include "lr1121_modem_modem.h"
#include <stdint.h>
#include "configuration_z.h"

#include <smtc_modem_hal.h>
#include <smtc_modem_hal_init.h>

#include <lora_lbm_transceiver.h>

#include "lr11xx_firmware_update.h"


#include "version.h"


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
extern ralf_t modem_radio;

/* lr11xx radio context and its use in the ralf layer */
static const struct device *transceiver = DEVICE_DT_GET(DT_ALIAS(lora_transceiver));
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
/* Callbacks for HAL implementation */
static struct smtc_modem_hal_cb prv_hal_cb;

/**
 * @brief Callback for modem hal
 */
static int prv_get_battery_level_cb(uint32_t *value)
{
	*value = 98;
	return 0;
}

/**
 * @brief Callback for modem hal
 */
static int prv_get_temperature_cb(int32_t *value)
{
	*value = 25;
	return 0;
}

/**
 * @brief Callback for modem hal
 */
static int prv_get_voltage_cb(uint32_t *value)
{
	*value = 3300;
	return 0;
}
int main( void )
{
    k_msleep(1000);
    bool is_updated = false;
    
    // system_init( );
    // system_time_wait_ms( 2000 );

    // lv_init( );
    // lv_port_disp_init( );
    printf( "LR11XX updater tool %s\n", DEMO_VERSION );
    // LOG_DBG("H3");
    k_msleep(100);
    // printf()
    // gui_init( LR11XX_FIRMWARE_UPDATE_TO, LR11XX_FIRMWARE_VERSION );
    // bool ret = true;

	/* Create callback structure for HAL impl */
	prv_hal_cb = (struct smtc_modem_hal_cb){
		.get_battery_level = prv_get_battery_level_cb,
		.get_temperature = prv_get_temperature_cb,
		.get_voltage = prv_get_voltage_cb,

#ifdef CONFIG_LORA_BASICS_MODEM_FUOTA
		.get_hw_version_for_fuota = prv_get_hw_version_for_fuota,
		.get_fw_version_for_fuota = prv_get_fw_version_for_fuota,
		.get_fw_status_available_for_fuota = prv_get_fw_status_available_for_fuota,
		.get_next_fw_version_for_fuota = prv_get_next_fw_version_for_fuota,
		.get_fw_delete_status_for_fuota = prv_get_fw_delete_status_for_fuota,
#endif /* CONFIG_LORA_BASICS_MODEM_FUOTA */
	};

	modem_radio.ral.context = transceiver;

	smtc_modem_hal_init(transceiver);
	smtc_modem_hal_register_callbacks(&prv_hal_cb);

	/* Disable IRQ to avoid unwanted behaviour during init */
	hal_mcu_disable_irq();

	/* Configure all the µC periph (clock, gpio, timer, ...) */
	hal_mcu_init();

	/* Re-enable IRQ */
	hal_mcu_enable_irq();
    printf("%d %s\n", __LINE__, __FILE__);
	/* Tests */
	// SMTC_HAL_TRACE_MSG("\n\n\nPORTING_TEST example is starting\n\n");

    switch( LR11XX_FIRMWARE_UPDATE_TO )
    {
    case LR1110_FIRMWARE_UPDATE_TO_TRX:
    {
        printf( "Update LR1110 to transceiver firmware 0x%04x\n", LR11XX_FIRMWARE_VERSION );
        break;
    }
    case LR1120_FIRMWARE_UPDATE_TO_TRX:
    {
        printf( "Update LR1120 to transceiver firmware 0x%04x\n", LR11XX_FIRMWARE_VERSION );
        break;
    }
    case LR1121_FIRMWARE_UPDATE_TO_TRX:
    {
        printf( "Update LR1121 to transceiver firmware 0x%04x\n", LR11XX_FIRMWARE_VERSION );
        break;
    }
    case LR1110_FIRMWARE_UPDATE_TO_MODEM_V1:
    {
        printf( "Update LR1110 to modem firmware 0x%06x\n", LR11XX_FIRMWARE_VERSION );
        break;
    }
    case LR1121_FIRMWARE_UPDATE_TO_MODEM_V2:
    {
        printf( "Update LR1121 to modem firmware 0x%06x\n", LR11XX_FIRMWARE_VERSION );
        break;
    }
    }

    while( 1 )
    {
        // lv_task_handler( );

        if( is_updated == false )
        {
            // system_gpio_set_pin_state( lr11xx_led_scan, SYSTEM_GPIO_PIN_STATE_HIGH );

            const lr11xx_fw_update_status_t status = lr11xx_update_firmware(
                (void*)transceiver, LR11XX_FIRMWARE_UPDATE_TO, LR11XX_FIRMWARE_VERSION, lr11xx_firmware_image,
                sizeof( lr11xx_firmware_image ) / sizeof( lr11xx_firmware_image[0] ) );

            // system_gpio_set_pin_state( lr11xx_led_scan, SYSTEM_GPIO_PIN_STATE_LOW );

            switch( status )
            {
            case LR11XX_FW_UPDATE_OK:
                // system_gpio_set_pin_state( lr11xx_led_rx, SYSTEM_GPIO_PIN_STATE_HIGH );
                // gui_update( "UPDATE DONE!\nPlease flash another application\n(like EVK Demo App)" );
                printf( "Expected firmware running!\n" );
                printf( "Please flash another application (like EVK Demo App).\n" );
                break;
            case LR11XX_FW_UPDATE_WRONG_CHIP_TYPE:
                // system_gpio_set_pin_state( lr11xx_led_tx, SYSTEM_GPIO_PIN_STATE_HIGH );
                // gui_update( "WRONG CHIP TYPE" );
                printf( "Wrong chip type!\n" );
                break;
            case LR11XX_FW_UPDATE_ERROR:
                // system_gpio_set_pin_state( lr11xx_led_tx, SYSTEM_GPIO_PIN_STATE_HIGH );
                // gui_update( "ERROR\nWrong firmware version\nPlease retry" );
                printf( "Error! Wrong firmware version - please retry.\n" );
                break;
            }

            is_updated = true;
        }
    };
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
