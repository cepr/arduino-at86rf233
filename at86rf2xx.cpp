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
 * @brief       Implementation of public functions for AT86RF2xx drivers
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include <mraa.h>
#include <mraa/spi.h>
#include <mraa/gpio.h>

#include "at86rf2xx.h"

#include <unistd.h>
#define delay(ms) usleep((ms)*1000)
#define LOW 0
#define HIGH 1
typedef unsigned char byte;
#include <assert.h>

/*  Declare radio device as globally scoped struct  */
AT86RF2XX at86rf2xx = AT86RF2XX();

/**
 * @brief   Increments events count by  1.
 */
static void at86rf2xx_irq_handler(void *)
{
    at86rf2xx.events++;
    return;
}

AT86RF2XX::AT86RF2XX() {}

int AT86RF2XX::init(int cs_pin_, int int_pin_, int sleep_pin_, int reset_pin_, int spi_bus)
{
    printf("[at86rf2xx] Booting radio device.\n");

    /* initialize device descriptor */
    cs_pin = mraa_gpio_init_raw(cs_pin_);
    assert(cs_pin != NULL);

    int_pin = mraa_gpio_init_raw(int_pin_);
    assert(int_pin != NULL);

    sleep_pin = mraa_gpio_init_raw(sleep_pin_);
    assert(sleep_pin != NULL);

    reset_pin = mraa_gpio_init_raw(reset_pin_);
    assert(reset_pin != NULL);

    idle_state = AT86RF2XX_STATE_TRX_OFF;
    state = AT86RF2XX_STATE_SLEEP;

    /* setup GPIOs */    
    assert(mraa_gpio_dir(reset_pin, MRAA_GPIO_OUT) == MRAA_SUCCESS);
    assert(mraa_gpio_dir(sleep_pin, MRAA_GPIO_OUT) == MRAA_SUCCESS);
    assert(mraa_gpio_dir(int_pin, MRAA_GPIO_IN) == MRAA_SUCCESS);
    assert(mraa_gpio_dir(cs_pin, MRAA_GPIO_OUT) == MRAA_SUCCESS);

    /* initialise SPI */
    //  Set up SPI
    spi = mraa_spi_init_raw(spi_bus, cs_pin_);
    assert(spi != NULL);
    //  Data is transmitted and received MSB first
    assert(mraa_spi_lsbmode(spi, 0) == MRAA_SUCCESS);
    //  SPI interface will run at 1MHz
    assert(mraa_spi_frequency(spi, 1000000) == MRAA_SUCCESS);
    //  Data is clocked on the rising edge and clock is low when inactive
    assert(mraa_spi_mode(spi, MRAA_SPI_MODE0) == MRAA_SUCCESS);

    /*  wait for SPI to be ready  */
    delay(10);

    /*  initialize GPIOs */
    assert(mraa_gpio_write(sleep_pin, LOW) == MRAA_SUCCESS);
    assert(mraa_gpio_write(reset_pin, HIGH) == MRAA_SUCCESS);
    assert(mraa_gpio_write(cs_pin, HIGH) == MRAA_SUCCESS);
    assert(mraa_gpio_isr(int_pin, MRAA_GPIO_EDGE_RISING, at86rf2xx_irq_handler, this) == MRAA_SUCCESS);

    /* make sure device is not sleeping, so we can query part number */
    assert_awake();

    /* test if the SPI is set up correctly and the device is responding */
    byte part_num = reg_read(AT86RF2XX_REG__PART_NUM);
    if (part_num != AT86RF233_PARTNUM) {
        printf("[at86rf2xx] Error: unable to read correct part number.\n");
        return -1;
    }
    printf("[at86rf2xx] Detected part #: 0x%x [at86rf2xx] Version: 0x%x\n",
           part_num, reg_read(AT86RF2XX_REG__VERSION_NUM));

    /* reset device to default values and put it into RX state */
    reset();

    return 0;
}

void AT86RF2XX::reset()
{
    hardware_reset();

    /* Reset state machine to ensure a known state */
    reset_state_machine();

    /* reset options and sequence number */
    seq_nr = 0;
    options = 0;

    /* set short and long address */
    set_addr_long(AT86RF2XX_DEFAULT_ADDR_LONG);
    set_addr_short(AT86RF2XX_DEFAULT_ADDR_SHORT);

    /* set default PAN id */
    set_pan(AT86RF2XX_DEFAULT_PANID);

    /* set default channel */
    set_chan(AT86RF2XX_DEFAULT_CHANNEL);

    /* set default TX power */
    set_txpower(AT86RF2XX_DEFAULT_TXPOWER);

    /* set default options */
    set_option(AT86RF2XX_OPT_PROMISCUOUS, true);
    set_option(AT86RF2XX_OPT_AUTOACK, true);
    set_option(AT86RF2XX_OPT_CSMA, true);
    set_option(AT86RF2XX_OPT_TELL_RX_START, true);
    set_option(AT86RF2XX_OPT_TELL_RX_END, true);

    /* enable safe mode (protect RX FIFO until reading data starts) */
    reg_write(AT86RF2XX_REG__TRX_CTRL_2, AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE);

//#ifdef MODULE_AT86RF212B
//    at86rf2xx_set_freq(dev, AT86RF2XX_FREQ_915MHZ);
//#endif

    /* don't populate masked interrupt flags to IRQ_STATUS register */
    /*uint8_t tmp = at86rf2xx_reg_read(AT86RF2XX_REG__TRX_CTRL_1);
    tmp &= ~(AT86RF2XX_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_CTRL_1, tmp);*/

    /* disable clock output to save power */
    byte tmp = reg_read(AT86RF2XX_REG__TRX_CTRL_0);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
    tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
    reg_write(AT86RF2XX_REG__TRX_CTRL_0, tmp);

    /* enable interrupts */
    reg_write(AT86RF2XX_REG__IRQ_MASK, AT86RF2XX_IRQ_STATUS_MASK__TRX_END);

    /* clear interrupt flags */
    reg_read(AT86RF2XX_REG__IRQ_STATUS);

    /* go into RX state */
    set_state(AT86RF2XX_STATE_RX_AACK_ON);

    printf("[at86rf2xx] Reset complete.\n");
}

bool AT86RF2XX::cca()
{
    uint8_t tmp;
    uint8_t status;

    assert_awake();

    /* trigger CCA measurment */
    tmp = reg_read(AT86RF2XX_REG__PHY_CC_CCA);
    tmp &= AT86RF2XX_PHY_CC_CCA_MASK__CCA_REQUEST;
    reg_write(AT86RF2XX_REG__PHY_CC_CCA, tmp);

    /* wait for result to be ready */
    do {
        status = reg_read(AT86RF2XX_REG__TRX_STATUS);
    } while (!(status & AT86RF2XX_TRX_STATUS_MASK__CCA_DONE));

    /* return according to measurement */
    if (status & AT86RF2XX_TRX_STATUS_MASK__CCA_STATUS) {
        return true;
    }
    else {
        return false;
    }
}

size_t AT86RF2XX::send(uint8_t *data, size_t len)
{
    /* check data length */
    if (len > AT86RF2XX_MAX_PKT_LENGTH) {
        printf("[at86rf2xx] Error: Data to send exceeds max packet size.\n");
        return 0;
    }
    AT86RF2XX::tx_prepare();
    AT86RF2XX::tx_load(data, len, 0);
    AT86RF2XX::tx_exec();
    return len;
}

void AT86RF2XX::tx_prepare()
{
    uint8_t state;

    /* make sure ongoing transmissions are finished */
    do {
        state = get_status();
    }
    while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

    /* if receiving cancel */
    if(state == AT86RF2XX_STATE_BUSY_RX_AACK) {
        force_trx_off();
        idle_state = AT86RF2XX_STATE_RX_AACK_ON;
    } else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
        idle_state = state;
    }
    set_state(AT86RF2XX_STATE_TX_ARET_ON);
    frame_len = IEEE802154_FCS_LEN;
}

size_t AT86RF2XX::tx_load(uint8_t *data,
                         size_t len, size_t offset)
{
    frame_len += (uint8_t)len;
    sram_write(offset + 1, data, len);
    return offset + len;
}

void AT86RF2XX::tx_exec()
{
    /* write frame length field in FIFO */
    sram_write(0, &(frame_len), 1);
    /* trigger sending of pre-loaded frame */
    reg_write(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__TX_START);
    /*if (at86rf2xx.event_cb && (at86rf2xx.options & AT86RF2XX_OPT_TELL_TX_START)) {
        at86rf2xx.event_cb(NETDEV_EVENT_TX_STARTED, NULL);
    }*/
}

size_t AT86RF2XX::rx_len()
{
    uint8_t phr;
    fb_read(&phr, 1);

    /* ignore MSB (refer p.80) and substract length of FCS field */
    return (size_t)((phr & 0x7f) - 2);
}

void AT86RF2XX::rx_read(uint8_t *data, size_t len, size_t offset)
{
    /* when reading from SRAM, the different chips from the AT86RF2xx family
     * behave differently: the AT86F233, the AT86RF232 and the ATRF86212B return
     * frame length field (PHR) at position 0 and the first data byte at
     * position 1.
     * The AT86RF231 does not return the PHR field and return
     * the first data byte at position 0.
     */
#ifndef MODULE_AT86RF231
    sram_read(offset + 1, data, len);
#else
    sram_read(offset, data, len);
#endif
}
