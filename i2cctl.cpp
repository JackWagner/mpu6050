// implementation of the i2c controller
//
// basically stolen/guided from sparkfun (great company)
// https://learn.sparkfun.com/tutorials/programming-the-pcduino/i2c-communications
// http://www.frank-buss.de/io-expander/linux-i2c-test.c
//
// Modified to be not stolen
//
// by Mark Hill

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>

#include "i2cctl.h"


#ifdef RELEASE
static const int debug = 0;
#else
static const int debug = 1;
#endif



// sorry for the global. Forgive me for I know not what I do
// defaults to 1 because lets face it, thats normal
static uint8_t _bus = 1;
static int _i2cFile = -1000;

// this mutex is used to protect the i2c device from being used to do a read or write operations
//   simulateously on another thread
// if this weren't used, programs running on a separate thread trying to write to i2c devices
//   could potentially interfere with and corrupt other concurrent operations
// the other _mutex_created variable allows the mutex to be initialized without a discrete initialize function
// 1 = created, 0 = not created
static pthread_mutex_t _lock;
static int _mutex_created = 0;

// this is a wait function and returns once the lock is removed to allow the i2c function to access the device
static void getLock() {
    if (!_mutex_created) {
        pthread_mutex_init(&_lock, NULL);
        _mutex_created = 1;
    }
    pthread_mutex_lock(&_lock);
}

// this frees the lock to allow another function to use the i2c device
static void releaseLock() {
    if (_mutex_created) {
        pthread_mutex_unlock(&_lock);
    }
}


// initializes the _i2cFile variable which makes the i2c bus avaliable for reading and writing
// returns -1 on failure and 0 on success
int i2cInit() {
    if (_i2cFile < 0) {
        char i2cBusName[12];
        sprintf(i2cBusName, "/dev/i2c-%d", _bus);

        _i2cFile = open(i2cBusName, O_RDWR);
        if (_i2cFile < 0)
            goto init_error;
    }

    return 0;

init_error:
    printf("Error opening i2c file for bus %i\n", _bus);
    return -1;
}




// closes the i2c file
void i2cClose() {
    getLock();

    close(_i2cFile);
    _i2cFile = -1000;

    releaseLock();
    pthread_mutex_destroy(&_lock);
    _mutex_created = 0;
}



static uint8_t _tenbit_enabled = 0;
// sets the i2c device address and also configures the i2c device to take 10 bit or 8 bit addresses
// returns 0 for success and something else for error
static int i2cSetAddress(uint16_t address) {
    // in case the bus was never set, this ensures the i2c device is always initialized
    i2cInit();
    
    // set ten bit address mode
    uint8_t isTenBit = (address - 127 > 0) ? 1 : 0;
    if (_tenbit_enabled != isTenBit && ioctl(_i2cFile, I2C_TENBIT, isTenBit))
        goto ioctl_error;

    if(ioctl(_i2cFile, I2C_SLAVE, address))
        goto ioctl_error;

    return 0;

ioctl_error:
    printf("Failed to set i2c slave address to %x with tenbit set to %i\n", address, isTenBit);
    return -1;
}




// set the bus used for i2c communication
void i2cSetBus(uint8_t bus) {
    i2cClose();

    getLock();
    _bus = bus;
    i2cInit();
    releaseLock();
}

// performs a read operation
int i2c_read(uint16_t address, uint8_t reg, uint8_t *data, uint8_t count) {
    getLock();

    if (i2cSetAddress(address))
        goto read_error;
    if (write(_i2cFile, &reg, 1) < 1)
        goto read_error;
    if (read(_i2cFile, data, count) < count)
        goto read_error;
    releaseLock();
    return 0;

read_error:
    releaseLock();
    printf("failed to read %i bytes from device %x\n", count, address);
    return -1;
}

// performs a write operation
int i2c_write(uint16_t address, const uint8_t *data, uint8_t count) {
    getLock();

    if (i2cSetAddress(address))
        goto write_error;
    if (write(_i2cFile, data, count) < count)
        goto write_error;
    releaseLock();
    return 0;

write_error:
    releaseLock();
    printf("failed to write %i bytes to device %x\n", count, address);
    return -1;
}



    







