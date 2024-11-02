/*!
 * @file      configuration.h
 *
 * @brief     Configuration file
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

#ifndef _CONFIGURATION_Z_H
#define _CONFIGURATION_Z_H

#include <string.h>
#include "stdio.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>

/*
 * ----------------------------------------------------------------------------- 
 * --- PUBLIC MACROS ----------------------------------------------------------- 
 */

/*
 * ----------------------------------------------------------------------------- 
 * --- PUBLIC CONSTANTS -------------------------------------------------------- 
 */

// #define LR11XX_NSS_PORT DT_NODELABEL(gpioa)
// #define LR11XX_NSS_PIN 8
// #define LR11XX_RESET_PORT DT_NODELABEL(gpioa)
// #define LR11XX_RESET_PIN 0
// #define LR11XX_IRQ_PORT DT_NODELABEL(gpiob)
// #define LR11XX_IRQ_PIN 4
// #define LR11XX_BUSY_PORT DT_NODELABEL(gpiob)
// #define LR11XX_BUSY_PIN 3

// #define LR11XX_LED_SCAN_PORT DT_NODELABEL(gpiob)
// #define LR11XX_LED_SCAN_PIN 5
// #define LR11XX_LED_TX_PORT DT_NODELABEL(gpioc)
// #define LR11XX_LED_TX_PIN 1
// #define LR11XX_LED_RX_PORT DT_NODELABEL(gpioc)
// #define LR11XX_LED_RX_PIN 0

// #define LR11XX_LNA_PORT DT_NODELABEL(gpiob)
// #define LR11XX_LNA_PIN 0

// #define DISPLAY_NSS_PORT DT_NODELABEL(gpiob)
// #define DISPLAY_NSS_PIN 6
// #define DISPLAY_DC_PORT DT_NODELABEL(gpioc)
// #define DISPLAY_DC_PIN 7

// #define TOUCH_IRQ_PORT DT_NODELABEL(gpioa)
// #define TOUCH_IRQ_PIN 10

// #define FLASH_NSS_PORT DT_NODELABEL(gpiob)
// #define FLASH_NSS_PIN 10
// #define ACCELERATOR_IRQ_PORT DT_NODELABEL(gpioa)
// #define ACCELERATOR_IRQ_PIN 9

// #define ANTENNA_SWITCH_CTRL_PORT DT_NODELABEL(gpioc)
// #define ANTENNA_SWITCH_CTRL_PIN 8
// #define ANTENNA_SWITCH_N_CTRL_PORT DT_NODELABEL(gpioc)
// #define ANTENNA_SWITCH_N_CTRL_PIN 6

// #define BUTTON_BLUE_PORT DT_NODELABEL(gpioc)
// #define BUTTON_BLUE_PIN 13

/*
 * ----------------------------------------------------------------------------- 
 * --- PUBLIC TYPES ------------------------------------------------------------ 
 */

typedef struct configuration
{
    const struct device *port;
    uint32_t pin;
} gpio_t;

typedef struct
{
    const struct device *spi;
    gpio_t nss;
    gpio_t reset;
    gpio_t irq;
    gpio_t busy;
} radio_t;

#endif

/*
 * ----------------------------------------------------------------------------- 
 * --- PUBLIC FUNCTIONS PROTOTYPES --------------------------------------------- 
 */

/* --- EOF ------------------------------------------------------------------ */