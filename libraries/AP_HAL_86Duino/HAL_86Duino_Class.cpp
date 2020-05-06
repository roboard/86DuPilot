
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO

#include <assert.h>
#include <signal.h>

#include "HAL_86Duino_Class.h"
#include "AP_HAL_86Duino_Private.h"
#include "io.h"
#include "irq.h"
#include "com.h"
#include "pins_arduino.h"
using namespace x86Duino;

static UARTDriver Serial1(COM1, 115200L, BYTESIZE8 | NOPARITY | STOPBIT1, 0L, 500L);
static UARTDriver Serial2(COM2, 115200L, BYTESIZE8 | NOPARITY | STOPBIT1, 0L, 500L);
static UARTDriver Serial3(COM3, 115200L, BYTESIZE8 | NOPARITY | STOPBIT1, 0L, 500L);
static USBCDC usbUart;

static I2CDeviceManager i2cDeviceManager;
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;

static Util utilInstance;
static Scheduler schedulerInstance;

unsigned long CLOCKS_PER_MICROSEC = 300L; // The default value is 300Mhz for 86Duino, you should call init() to set it automatically.
unsigned long VORTEX86EX_CLOCKS_PER_MS = 300000L; // The default value is 300000 for 86Duino, you should call init() to set it automatically.
bool Global_irq_Init = false;

// Error process
#define LONG_TIME		(1000L)
#define SHORT_TIME		(50L)
#define ATIMESIZE 		(39)

static unsigned int _segledtime[ATIMESIZE + 1] = { SHORT_TIME, LONG_TIME, SHORT_TIME, LONG_TIME, SHORT_TIME, LONG_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, LONG_TIME
};

static unsigned int _fpledtime[ATIMESIZE + 1] = { SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME, SHORT_TIME,
                                             SHORT_TIME, LONG_TIME
};

void error_led_blink(int ledpin, unsigned int _ledtime[]) {
    static int led_stat = 0;
    static bool first = true;
    static unsigned long nowledtime = 0L;
    if (first == true)
    {
        nowledtime = AP_HAL::millis();
        if ((led_stat % 2) == 0) gpioDriver.write(ledpin, HIGH); else gpioDriver.write(ledpin, LOW);
        first = false;
    }

    if ((AP_HAL::millis() - nowledtime) > _ledtime[led_stat])
    {
        if (led_stat == ATIMESIZE) led_stat = 0; else led_stat++;
        first = true;
    }
}

void _86Duino_error_process(int num) {
    int ledpin = 13;
    // disable all irq except usb irq (5)
    i8259_DisableIRQ(0);
    i8259_DisableIRQ(1);
    i8259_DisableIRQ(3);
    i8259_DisableIRQ(4);
    i8259_DisableIRQ(6);
    i8259_DisableIRQ(7);
    i8259_DisableIRQ(8);
    i8259_DisableIRQ(9);
    i8259_DisableIRQ(10);
    i8259_DisableIRQ(11);
    i8259_DisableIRQ(12);
    i8259_DisableIRQ(13);
    i8259_DisableIRQ(14);

    // print error message
    printf("\nOop, this program is crash :(\n");
    printf("You may write a bug in your sketch, check and upload it again.\n");

    // led blink pattern	
    gpioDriver.pinMode(ledpin, OUTPUT);
    while (1)
    {
        if (num == SIGSEGV)
            error_led_blink(ledpin, _segledtime);
        else if(num == SIGFPE)
            error_led_blink(ledpin, _fpledtime);

    }
}


HAL_86Duino::HAL_86Duino() :
    AP_HAL::HAL(
        &usbUart,           /* usb cdc A */
        &Serial1,           /* uartB */
        &Serial2,           /* uartC */
        &Serial3,           /* uartD */
        nullptr,            /* no uartE */
        nullptr,            /* no uartF */
        &i2cDeviceManager,  /* i2c mgr*/
        &spiDeviceManager,  /* spi_mgr */
        &analogIn,          /* analogin */
        &storageDriver,     /* storage */
        &usbUart,           /* console */
        &gpioDriver,        /* GPIO */
        &rcinDriver,        /* rc input */
        &rcoutDriver,       /* rc output */
        &schedulerInstance, /* scheduler */
        &utilInstance,      /* utility */
        nullptr,
		nullptr)
{}


DPMI_MEMORY_ALL_LOCK(0)
void HAL_86Duino::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    int i, crossbarBase;
    if (io_Init() == false) return;
    timer_NowTime(); // initialize timer
    CLOCKS_PER_MICROSEC = vx86_CpuCLK();
    VORTEX86EX_CLOCKS_PER_MS = CLOCKS_PER_MICROSEC * 1000UL;

    // Set IRQ4 as level-trigger
    io_outpb(0x4D0, io_inpb(0x4D0) | 0x10);

    //set corssbar Base Address
    crossbarBase = sb_Read16(SB_CROSSBASE) & 0xfffe;
    if (crossbarBase == 0 || crossbarBase == 0xfffe)
    {
        sb_Write16(SB_CROSSBASE, CROSSBARBASE | 0x01);
        crossbarBase = CROSSBARBASE;
    }

#if defined CRB_DEBUGMODE
    for (i = 0; i < CRBTABLE_SIZE; i++) io_outpb(crossbarBase + i, CROSSBARTABLE[i]);
#endif

     //Force set HIGH speed ISA on SB
    sb_Write(SB_FCREG, sb_Read(SB_FCREG) | 0x8000C000L);

    // gpio init
    gpioDriver.init();

    // analogin  init
    analogIn.init();
    // set MCM Base Address
    set_MMIO();
    mcmInit();
    for (i = 0; i < 4; i++)
        mc_SetMode(i, MCMODE_PWM_SIFB);

    if (Global_irq_Init == false)
    {
        // set MCM IRQ
        if (irq_Init() == false)
        {
            printf("MCM IRQ init fail\n"); 
            return;
        }

        if (irq_Setting(GetMCIRQ(), IRQ_LEVEL_TRIGGER | IRQ_DISABLE_INTR | IRQ_USE_FPU) == false)
        {
            printf("MCM IRQ Setting fail\n"); 
            return;
        }
        Set_MCIRQ(GetMCIRQ());
        Global_irq_Init = true;
    }

	
	// init wdt 1Hz
	//util->init_wdt(1000000UL, false);

    usbUart.begin(115200);

    signal(SIGSEGV, _86Duino_error_process);
    signal(SIGFPE, _86Duino_error_process);

    scheduler->init();
    
    rcin->init();
    rcout->init();

    storage->init();

    callbacks->setup();
    scheduler->system_initialized();
	util->init_perf();
    console->printf("system_initialized\n");
    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_86Duino hal_86duino;
    return hal_86duino;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
