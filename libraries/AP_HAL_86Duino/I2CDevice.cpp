/*
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
//#include "Scheduler.h"
#include <stdio.h>
#include "I2CDevice.h"
#include "io.h"
#include "i2c.h"

extern const AP_HAL::HAL& hal;

using namespace x86Duino;

Semaphore I2CDevice::i2c_semaphore;

I2CDevice::I2CDevice(uint8_t bus, uint8_t address) :
    _bus(bus),
    _address(address)
{
    set_device_bus(bus);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
        (unsigned)bus, (unsigned)address);
    perf = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, pname);
}

I2CDevice::~I2CDevice()
{
    hal.console->printf("I2C device bus %u address 0x%02x closed\n",
        (unsigned)_bus, (unsigned)_address);
    free(pname);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (send_len == 0 && recv_len == 0) {
        hal.console->printf("the number of bytes to read or write must > 0");
        return false;
    }
    else if (send_len == 0)
        return i2c_Receive(_address, recv, recv_len);
    else if (recv_len == 0)
        return i2c_Send(_address, send, send_len);
    else
        return i2c_SensorReadEX(_address, send, send_len, recv, recv_len);
}


AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &i2c_semaphore;
}


void I2CDeviceManager::init(void)
{
    if(!i2c_Init(I2CMODE_AUTO, 400000L))
        AP_HAL::panic("Failed to int i2c bus");
    _is_initailized = true;
}
