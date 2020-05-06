/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "Semaphores.h"

namespace x86Duino {

class SPIBus;
class SPIDesc;

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &bus, SPIDesc &device_desc);

    virtual ~SPIDevice() { }

    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /* See AP_HAL::SPIDevice::transfer_fullduplex() */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore();

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

private:
    SPIBus &_bus;
    SPIDesc &_desc;
    AP_HAL::DigitalSource *_cs;
    uint32_t _speed;
    static Semaphore spi_semaphore;


    /*
     * Select device if using userspace CS
     */
    void _cs_assert();

    /*
     * Deselect device if using userspace CS
     */
    void _cs_release();
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    SPIDeviceManager();
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) override;
    void init() {};

private:
    static const uint8_t _n_device_desc;
    static SPIDesc _device[];
    SPIBus  *_bus;
};

}
