/*
 * Copyright (C) 2013 Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Implementation of driver internal functions
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include <mraa/spi.h>
#include "at86rf2xx.h"

#include <unistd.h>
typedef unsigned char byte;
#define LOW 0
#define HIGH 1

void AT86RF2XX::reg_write(const uint8_t addr,
                         const uint8_t value)
{
    byte writeCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_WRITE;
    mraa_gpio_write(cs_pin, LOW);
    mraa_spi_write(spi, writeCommand);
    mraa_spi_write(spi, value);
    mraa_gpio_write(cs_pin, HIGH);
}

uint8_t AT86RF2XX::reg_read(const uint8_t addr)
{
    byte value;
    byte readCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_READ;
    mraa_gpio_write(cs_pin, LOW);
    mraa_spi_write(spi, readCommand);
    value = mraa_spi_write(spi, 0x00);
    mraa_gpio_write(cs_pin, HIGH);

    return (uint8_t)value;
}

void AT86RF2XX::sram_read(const uint8_t offset,
                         uint8_t *data,
                         const size_t len)
{
    byte readCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_READ;
    mraa_gpio_write(cs_pin, LOW);
    mraa_spi_write(spi, readCommand);
    mraa_spi_write(spi, (char)offset);
    for (int b=0; b<len; b++) {
      data[b] = mraa_spi_write(spi, 0x00);
    }
    mraa_gpio_write(cs_pin, HIGH);
}

void AT86RF2XX::sram_write(const uint8_t offset,
                          const uint8_t *data,
                          const size_t len)
{
    byte writeCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_WRITE;
    mraa_gpio_write(cs_pin, LOW);
    mraa_spi_write(spi, writeCommand);
    mraa_spi_write(spi, (char)offset);
    for (int b=0; b<len; b++) {
      mraa_spi_write(spi, data[b]);
    }
    mraa_gpio_write(cs_pin, HIGH);
}

void AT86RF2XX::fb_read(uint8_t *data,
                       const size_t len)
{
    byte readCommand = AT86RF2XX_ACCESS_FB | AT86RF2XX_ACCESS_READ;
    mraa_gpio_write(cs_pin, LOW);
    mraa_spi_write(spi, readCommand);
    for (int b=0; b<len; b++) {
      data[b] = mraa_spi_write(spi, 0x00);
    }
    mraa_gpio_write(cs_pin, HIGH);
}

uint8_t AT86RF2XX::get_status()
{
    /* if sleeping immediately return state */
    if(state == AT86RF2XX_STATE_SLEEP)
        return state;

    return reg_read(AT86RF2XX_REG__TRX_STATUS) & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
}

void AT86RF2XX::assert_awake()
{
    if(get_status() == AT86RF2XX_STATE_SLEEP) {
        /* wake up and wait for transition to TRX_OFF */
        mraa_gpio_write(sleep_pin, LOW);
        usleep(AT86RF2XX_WAKEUP_DELAY);

        /* update state */
        state = reg_read(AT86RF2XX_REG__TRX_STATUS) & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
    }
}

void AT86RF2XX::hardware_reset()
{
    /* wake up from sleep in case radio is sleeping */
    //delayMicroseconds(50); // Arduino seems to hang without some minimum pause here
    assert_awake();

    /* trigger hardware reset */

    mraa_gpio_write(reset_pin, LOW);
    usleep(AT86RF2XX_RESET_PULSE_WIDTH);
    mraa_gpio_write(reset_pin, HIGH);
    usleep(AT86RF2XX_RESET_DELAY);
}

void AT86RF2XX::force_trx_off()
{
    reg_write(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__FORCE_TRX_OFF);
    while (get_status() != AT86RF2XX_STATE_TRX_OFF);
}
