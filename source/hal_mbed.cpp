// ----------------------------------------------------------------------------
// Copyright 2018 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "mbed.h"
#include "atca_hal.h"

#include "mbed_trace.h"
#define TRACE_GROUP "CRYP"

#ifndef ATAC_HAL_MBED_MAX_I2C
#define ATAC_HAL_MBED_MAX_I2C       1
#endif

// HAL structure, use this to store data
typedef struct {
    bool active;
    uint8_t slave_address;
    uint8_t bus;
    uint32_t baud;
    uint16_t wake_delay;
    int rx_retries;
    I2C *i2c;
} mbed_i2c_hal_data_t;

// Hold all active HAL structures
static mbed_i2c_hal_data_t mbed_i2c_hal_data[ATAC_HAL_MBED_MAX_I2C];
static bool mbed_i2c_hal_first_init = true;

void atca_delay_us(uint32_t delay) {
    // tr_debug("delay_us %lu", delay);
    wait_us(delay);
}

void atca_delay_10us(uint32_t delay) {
    // tr_debug("delay_10us %lu", delay);
    wait_us(delay * 10);
}

void atca_delay_ms(uint32_t delay) {
    // tr_debug("delay_ms %lu", delay);
    wait_ms(delay);
}

/** \brief hal_i2c_init manages requests to initialize a physical interface.  it manages use counts so when an interface
 * has released the physical layer, it will disable the interface for some other use.
 * You can have multiple ATCAIFace instances using the same bus, and you can have multiple ATCAIFace instances on
 * multiple i2c buses, so hal_i2c_init manages these things and ATCAIFace is abstracted from the physical details.
 */

/** \brief initialize an I2C interface using given config
 * \param[in] hal - opaque ptr to HAL data
 * \param[in] cfg - interface configuration
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg) {
    if (mbed_i2c_hal_first_init) {
        for (size_t ix = 0; ix < ATAC_HAL_MBED_MAX_I2C; ix++) {
            mbed_i2c_hal_data[ix].active = false;
        }
        mbed_i2c_hal_first_init = false;
    }

    if (cfg->iface_type != ATCA_I2C_IFACE) {
        return ATCA_BAD_PARAM;
    }

    // OK... Let's find an unused item...
    mbed_i2c_hal_data_t *hal_data = NULL;
    for (size_t ix = 0; ix < ATAC_HAL_MBED_MAX_I2C; ix++) {
        if (!mbed_i2c_hal_data[ix].active) {
            hal_data = &mbed_i2c_hal_data[ix];
            break;
        }
    }

    if (!hal_data) {
        tr_error("Could not find unallocated mbed_i2c_hal_data_t structure");
        return ATCA_ALLOC_FAILURE;
    }

    // tr_debug("hal_i2c_init, slave_address=%u, bus=%u, baud=%lu, wake_delay=%u, rx_retries=%d",
    //     cfg->atcai2c.slave_address, cfg->atcai2c.bus, cfg->atcai2c.baud, cfg->wake_delay, cfg->rx_retries);

    hal_data->active = true;
    hal_data->slave_address = cfg->atcai2c.slave_address;
    hal_data->bus = cfg->atcai2c.bus;
    hal_data->baud = cfg->atcai2c.baud;
    hal_data->wake_delay = cfg->wake_delay;
    hal_data->rx_retries = cfg->rx_retries;

    // @todo, bus is ignored, and we only search primary bus. This needs to be fixed.
    I2C *i2c = new I2C(I2C_SDA, I2C_SCL);
    i2c->frequency(hal_data->baud);

    hal_data->i2c = i2c;

    ((ATCAHAL_t*)hal)->hal_data = hal_data;

    // tr_debug("hal_i2c_init OK");

    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C post init
 * \param[in] iface  instance
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_post_init(ATCAIface iface) {
    // tr_debug("hal_i2c_post_init");
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C send over ASF
 * \param[in] iface     instance
 * \param[in] txdata    pointer to space to bytes to send
 * \param[in] txlength  number of bytes to send
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength) {
    mbed_i2c_hal_data_t *hal_data = (mbed_i2c_hal_data_t*)(iface->hal_data);
    // tr_debug("hal_i2c_send length=%d", txlength);

    // for this implementation of I2C with CryptoAuth chips, txdata is assumed to have ATCAPacket format

    // other device types that don't require i/o tokens on the front end of a command need a different hal_i2c_send and wire it up instead of this one
    // this covers devices such as ATSHA204A and ATECCx08A that require a word address value pre-pended to the packet
    // txdata[0] is using _reserved byte of the ATCAPacket
    txdata[0] = 0x3;    // insert the Word Address Value, Command token
    txlength++;         // account for word address value byte.

    int r = hal_data->i2c->write(hal_data->slave_address, (char*)txdata, txlength);
    // tr_debug("hal_i2c_send returned %d", r);
    if (r != 0) {
        return ATCA_TX_FAIL;
    }

    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C receive function for ASF I2C
 * \param[in]    iface     Device to interact with.
 * \param[out]   rxdata    Data received will be returned here.
 * \param[inout] rxlength  As input, the size of the rxdata buffer.
 *                         As output, the number of bytes received.
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength) {
    mbed_i2c_hal_data_t *hal_data = (mbed_i2c_hal_data_t*)(iface->hal_data);

    // read procedure is:
    // 1. read 1 byte, this will be the length of the package
    // 2. read the rest of the package

    char lengthPackage[1] = { 0 };
    int r = -1;
    int retries = hal_data->rx_retries;
    while (--retries > 0 && r != 0) {
        r = hal_data->i2c->read(hal_data->slave_address, lengthPackage, 1);
    }

    if (r != 0) {
        return ATCA_RX_TIMEOUT;
    }

    uint8_t bytesToRead = lengthPackage[0] - 1;

    if (bytesToRead > *rxlength) {
        tr_warn("hal_i2c_receive buffer too small, requested %u, but have %u", bytesToRead, *rxlength);
        return ATCA_SMALL_BUFFER;
    }

    memset(rxdata, 0, *rxlength);
    rxdata[0] = lengthPackage[0];

    r = -1;
    retries = hal_data->rx_retries;
    while (--retries > 0 && r != 0) {
        r = hal_data->i2c->read(hal_data->slave_address, (char*)rxdata + 1, bytesToRead);
    }

    if (r != 0) {
        return ATCA_RX_TIMEOUT;
    }

    // tr_debug("hal_i2c_receive, returned=%d (retryCount=%d)", r, retries);
    // tr_debug("rx buffer is %02x %02x %02x %02x", rxdata[0], rxdata[1], rxdata[2], rxdata[3]);

    *rxlength = lengthPackage[0];

    return ATCA_SUCCESS;
}

/** \brief wake up CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to wakeup
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_wake(ATCAIface iface) {
    mbed_i2c_hal_data_t *hal_data = (mbed_i2c_hal_data_t*)(iface->hal_data);

    // tr_debug("hal_i2c_wake");

    // steps to wake the chip up...

    // 1. switch to 100KHz
    hal_data->i2c->frequency(100000);

    // 2. Send NULL buffer to address 0x0 (NACK)
    int r = hal_data->i2c->write(0x0, NULL, 0);
    // tr_debug("write returned %d", r);

    // 3. Wait for wake_delay
    wait_us(hal_data->wake_delay);

    char rx_buffer[4] = { 0 };

    // 4. Read from normal slave_address
    int retries = hal_data->rx_retries;
    while (--retries > 0 && r != 0) {
        r = hal_data->i2c->read(hal_data->slave_address, rx_buffer, 4);
        // tr_debug("read returned %d (retryCount=%d)", r, retries);
    }

    // 5. Set frequency back to requested one
    hal_data->i2c->frequency(hal_data->baud);

    const uint8_t expected_response[4] = { 0x04, 0x11, 0x33, 0x43 };
    uint8_t selftest_fail_resp[4] = { 0x04, 0x07, 0xC4, 0x40 };

    if (memcmp(rx_buffer, expected_response, 4) == 0) {
        // tr_debug("wake successful");
        return ATCA_SUCCESS;
    }
    if (memcmp(rx_buffer, selftest_fail_resp, 4) == 0) {
        tr_warn("wake selftest error");
        return ATCA_STATUS_SELFTEST_ERROR;
    }
    tr_warn("wake failed");
    tr_debug("wake response buffer is %02x %02x %02x %02x", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
    return ATCA_WAKE_FAILED;
}

/** \brief idle CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to idle
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_idle(ATCAIface iface) {
    mbed_i2c_hal_data_t *hal_data = (mbed_i2c_hal_data_t*)(iface->hal_data);
    // tr_debug("hal_i2c_idle");

    char buffer[1] = { 0x2 }; // idle word address value
    hal_data->i2c->write(hal_data->slave_address, buffer, 1);

    return ATCA_SUCCESS;
}

/** \brief sleep CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to sleep
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_i2c_sleep(ATCAIface iface) {
    mbed_i2c_hal_data_t *hal_data = (mbed_i2c_hal_data_t*)(iface->hal_data);
    // tr_debug("hal_i2c_sleep");

    char buffer[1] = { 0x1 };  // sleep word address value
    hal_data->i2c->write(hal_data->slave_address, buffer, 1);

    return ATCA_SUCCESS;
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 * return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_release(void *hal_data) {
    mbed_i2c_hal_data_t *data = (mbed_i2c_hal_data_t*)hal_data;

    // tr_debug("hal_i2c_release");

    if (data->i2c) {
        delete data->i2c;
    }
    data->active = false;

    return ATCA_SUCCESS;
}

/** \brief discover i2c buses available for this hardware
 * this maintains a list of logical to physical bus mappings freeing the application
 * of the a-priori knowledge
 * \param[in] i2c_buses - an array of logical bus numbers
 * \param[in] max_buses - maximum number of buses the app wants to attempt to discover
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses) {
    tr_debug("hal_i2c_discover_buses max_buses=%d", max_buses);
    return ATCA_UNIMPLEMENTED;
}


/** \brief discover any CryptoAuth devices on a given logical bus number
 * \param[in]  bus_num  logical bus number on which to look for CryptoAuth devices
 * \param[out] cfg     pointer to head of an array of interface config structures which get filled in by this method
 * \param[out] found   number of devices found on this bus
 * \return ATCA_SUCCESS
 */
ATCA_STATUS hal_i2c_discover_devices(int bus_num, ATCAIfaceCfg *cfg, int *found) {
    tr_debug("hal_i2c_discover_devices bus_num=%d", bus_num);
    return ATCA_UNIMPLEMENTED;
}
