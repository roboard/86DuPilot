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
#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "GPIO.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "io.h"

extern const AP_HAL::HAL& hal ;
namespace x86Duino {
static unsigned int SPI_IOaddr;

#define SPI_MODE0 0x00
#define SPI_MODE1 0x02
#define SPI_MODE2 0x04
#define SPI_MODE3 0x06

#define PCI_CLOCK (100000000L) // 100MHz

#define FULLDUPEX    (0x80) // 0x7F is half-dupex
#define LSBSHIFT     (0x20) // 0xDF is MSB shift
#define SMOD         (0x08) // 0xF7 is disable it
#define CPOL         (0x04) // 0xFB is disable it
#define CPHA         (0x02) // 0xFD is disable it
#define RESET        (0x01) // it will be clear when out of reset state

#define MHZ (1000U*1000U)
#define KHZ (1000U)
#define SPI_CS_KERNEL -1

Semaphore SPIDevice::spi_semaphore;

class SPIDesc {
public:
    SPIDesc(const char *name_, uint16_t bus_, uint16_t subdev_, uint8_t mode_,
            uint8_t bits_per_word_, int16_t cs_pin_, uint32_t lowspeed_,
            uint32_t highspeed_)
        : name(name_), bus(bus_), subdev(subdev_), mode(mode_)
        , bits_per_word(bits_per_word_), cs_pin(cs_pin_), lowspeed(lowspeed_)
        , highspeed(highspeed_)
    {
    }
    
    const char *name;
    uint16_t bus;
    uint16_t subdev;
    uint8_t mode;
    uint8_t bits_per_word;
    int16_t cs_pin;
    uint32_t lowspeed;
    uint32_t highspeed;
};

SPIDesc SPIDeviceManager::_device[] = {
    // different SPI tables per board subtype
    SPIDesc("mpu9250",    0, 0, SPI_MODE3, 8, 9,  1*MHZ, 10*MHZ),
    SPIDesc("bmp280",     0, 0, SPI_MODE3, 8, 8,  10*MHZ, 10*MHZ),
};

const uint8_t SPIDeviceManager::_n_device_desc = ARRAY_SIZE(SPIDeviceManager::_device);

class SPIBus {
public:
    SPIBus() {_initialized = false;}
    bool isInit();
    void init();
    void set_Speed(uint32_t speed);
    void set_Mode(uint8_t mode);
    uint32_t Speed() { return _speed ;}
    uint8_t Mode() { return _mode ;}
    
    
private:
    uint32_t _initialized;
    uint8_t _mode;
    uint32_t _speed;
    void WriteCLKDIVR(uint8_t data);
    void Reset();
    void WriteCTRR(uint8_t data);
    void setClockDivider(uint16_t rate);
    void useFIFO();
};

bool SPIBus::isInit()
{
    return _initialized ;
}

void SPIBus::init()
{
    if (!_initialized) {
        void* pciDev = NULL;

        // Get SPI device base address
        pciDev = pci_Alloc(0x00, 0x10, 0x01); // PCI SPI configuration space
        if (pciDev == NULL) { printf("SPI device don't exist\n"); return; }
        SPI_IOaddr = (unsigned)(pci_In16(pciDev, 0x10) & 0xFFFFFFF0L); // get SPI base address
        pci_Free(pciDev);

        _mode = SPI_MODE3;
        _speed = 4 * MHZ;
        WriteCTRR(FULLDUPEX + _mode + RESET);

        io_outpb(SPI_IOaddr + 7, FULLDUPEX); // full-dupex
        set_Mode(_mode);
        io_outpb(SPI_IOaddr + 0x0b, 0x08); // delay clk between two transfers
        //SOURCE clock/(2 * SPI_CLOCK_DIV)
        set_Speed(_speed);
        useFIFO();

    }
    _initialized = true;
}

void SPIBus::set_Speed(uint32_t speed) {
    // set SPI speed
    uint32_t clockDiv;
    double fdiv = PCI_CLOCK / (2.0 * speed);
    uint32_t div = (uint32_t)fdiv;
    if (fdiv == (double)div)
        clockDiv = div;
    else
        clockDiv = div + 1;
	//hal.console->printf("clockDiv = %lu\n", clockDiv);
    setClockDivider(clockDiv);
    _speed = speed;
}

void SPIBus::set_Mode(uint8_t mode)
{
    if(SPI_IOaddr == 0) return;
    io_outpb(SPI_IOaddr + 7, (io_inpb(SPI_IOaddr + 7) & 0xF1 )| mode); // set mode
    _mode = mode;
}

void SPIBus::WriteCLKDIVR(uint8_t data) {
    if(SPI_IOaddr == 0) return;
    io_outpb(SPI_IOaddr + 6, data);
}

void SPIBus::Reset(void) {
    if(SPI_IOaddr == 0) return;
    io_outpb(SPI_IOaddr + 7, 0x01);
    while((io_inpb(SPI_IOaddr + 7)&0x01) != 0); // wait SPI reset for complete
}

void SPIBus::WriteCTRR(uint8_t data) {
    if(SPI_IOaddr == 0) return;
    io_outpb(SPI_IOaddr + 7, io_inpb(SPI_IOaddr + 7) | data);
}

void SPIBus::setClockDivider(uint16_t rate)
{
    if(rate == 0 || rate > 4095) return;
    if(SPI_IOaddr == 0) return;
    if(rate > 15)
        io_outpb(SPI_IOaddr + 6, (rate&0x0ff0)>>4);
	else
		io_outpb(SPI_IOaddr + 6, 0x00);
    
    io_outpb(SPI_IOaddr + 2, (io_inpb(SPI_IOaddr + 2) & 0xF0) | (rate%16));
}

void SPIBus::useFIFO(void) {
    if(SPI_IOaddr == 0) return;
    io_outpb(SPI_IOaddr + 2, io_inpb(SPI_IOaddr + 2) | 0x10);
}

SPIDevice::SPIDevice(SPIBus &bus, SPIDesc &device_desc)
    : _bus(bus)
    , _desc(device_desc)
{
    set_device_bus(0);
    set_device_address(_desc.subdev);
    _speed = _desc.highspeed;
    
    if (_desc.cs_pin != SPI_CS_KERNEL) {
        _cs = hal.gpio->channel(_desc.cs_pin);
        if (!_cs) {
            AP_HAL::panic("Unable to instantiate cs pin");
        }
        
        _cs->mode(HAL_GPIO_OUTPUT);
        
        // do not hold the SPI bus initially
        _cs_release();
    }
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        _speed = _desc.highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
        _speed = _desc.lowspeed;
        break;
    }
	//hal.console->printf("%s speed: %lu\n", _desc.name, _speed);
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    if(SPI_IOaddr == 0) return 0;

    // check bus setting
    _cs_assert();
    if (_bus.Speed() != _speed) {
        _bus.set_Speed(_speed);
    }
    if (_bus.Mode() != _desc.mode) {
        _bus.set_Mode(_desc.mode); 
    }

    if( send_len )
    {
        for( uint32_t i = 0 ; i < send_len ; i++ )
        {
            io_outpb(SPI_IOaddr, send[i]);
            while((io_inpb(SPI_IOaddr + 3) & 0x08) == 0);
            while((io_inpb(SPI_IOaddr + 3) & 0x20) == 0);
            io_inpb(SPI_IOaddr + 1);
        }
    }
    
    if( recv_len )
    {
        for( uint32_t i = 0 ; i < recv_len ; i++ )
        {
            io_outpb(SPI_IOaddr, 0);
            while((io_inpb(SPI_IOaddr + 3) & 0x08) == 0);
            while((io_inpb(SPI_IOaddr + 3) & 0x20) == 0);
            recv[i] = io_inpb(SPI_IOaddr + 1);
        }
    }

    _cs_release();
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    if(SPI_IOaddr == 0) return 0;
    // check bus setting
    if( _bus.Speed() != _speed) _bus.set_Speed(_speed);
    if( _bus.Mode() != _desc.mode ) _bus.set_Mode(_desc.mode);
    
    _cs_assert();
    if( len )
    {
        for( uint32_t i = 0 ; i < len ; i++ )
        {
            io_outpb(SPI_IOaddr, send[i]);
            while((io_inpb(SPI_IOaddr + 3) & 0x08) == 0);
            while((io_inpb(SPI_IOaddr + 3) & 0x20) == 0);
            recv[i] = io_inpb(SPI_IOaddr + 1);
        }
    }
    _cs_release();
    return true;
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &spi_semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return nullptr ;
}

void SPIDevice::_cs_assert()
{
    if (_desc.cs_pin == SPI_CS_KERNEL) {
        return;
    }
    
    _cs->write(0);
}

void SPIDevice::_cs_release()
{
    if (_desc.cs_pin == SPI_CS_KERNEL) {
        return;
    }
    
    _cs->write(1);
}

// SPI BUS relate
SPIDeviceManager::SPIDeviceManager()
{
    _bus = new SPIBus();
    if (!_bus) {
        printf("fail to create SPIDevice bus\n");
        return;
    }
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice> SPIDeviceManager::get_device(const char *name)
{
    SPIDesc *desc = nullptr;
    
	if (!_bus->isInit()) {
		_bus->init();
	}
    /* Find the bus description in the table */
    for (uint8_t i = 0; i < _n_device_desc; i++) {
        if (!strcmp(_device[i].name, name)) {
            desc = &_device[i];
            break;
        }
    }
    
    if (!desc) {
        printf("SPI: Invalid device name: %s\n", name);
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }
    
    auto dev = AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*_bus, *desc));
    if (!dev) {
        printf("fail to new SPIDevice desciption name: %s\n", name);
        return nullptr;
    }
    
    return dev;
}

}
