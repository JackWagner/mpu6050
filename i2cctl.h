// It became obvious that i2c is integral to more than just the sensor code, so it should be a separate file
//   with its own header
//
//   by Mark Hill

#ifndef _i2cctl
#define _i2cctl   

#include<stdint.h>


// sets the i2c bus number to be used
void i2cSetBus(uint8_t bus);

// reads <count> items from the i2c device at <address>
//   starting from <reg>
// returns -1 on failure and 0 on success
int i2c_read(uint16_t address, uint8_t reg, uint8_t *data, uint8_t count);

// writes the bytes stored in <data> to the i2c device at <address>
// supports 10 bit addressing
// returns -1 on failure and 0 on success
int i2c_write(uint16_t address, const uint8_t *data, uint8_t count);

// closes out the i2c file
// I honestly can't anticipate a valid use for this since it's not like having the
//   file open is that big a strain, but someone else may have better use, and its
//   good practice to have this ability
void i2cClose();



#endif
