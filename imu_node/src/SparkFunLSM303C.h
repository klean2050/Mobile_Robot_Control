// Test derrived class for base class SparkFunIMU
#ifndef __SPARKFUN_LSM303C_H__
#define __SPARKFUN_LSM303C_H__

#include "SparkFunIMU.h"
#include "LSM303CTypes.h"

#define SENSITIVITY_ACC   0.06103515625   // LSB/mg
#define SENSITIVITY_MAG   0.48828125   // LSB/mGa

#define DEBUG 0 // Change to 1 (nonzero) to enable debug messages
#include <math.h>

// Define a few error messages to save on space
static const char AERROR[] = "\nAccel Error";
static const char MERROR[] = "\nMag Error";


class LSM303C : public SparkFunIMU
{
  public:
    // These are the only methods are the only methods the user can use w/o mods
    ~LSM303C();
    status_t begin(void);
    // Begin contains hardware specific code (Pro Mini)
    status_t begin(MAG_DO_t, MAG_FS_t, MAG_BDU_t, MAG_OMXY_t,
        MAG_OMZ_t, MAG_MD_t, ACC_FS_t, ACC_BDU_t, uint8_t, ACC_ODR_t);
    float readAccelX(void);
    float readAccelY(void);
    float readAccelZ(void);
    float readMagX(void);
    float readMagY(void);
    float readMagZ(void);

  protected:
  	int I2C_A_fd,I2C_M_fd;
    // Variables to store the most recently read raw data from sensor
    AxesRaw_t accelData = {NAN, NAN, NAN};
    AxesRaw_t magData = {NAN, NAN, NAN};

    // Hardware abstraction functions (Pro Mini)
    uint8_t  I2C_ByteWrite(int, uint8_t, uint8_t);  
    status_t I2C_ByteRead(int, uint8_t, uint8_t&);

    // Methods required to get device up and running
    status_t MAG_SetODR(MAG_DO_t);
    status_t MAG_SetFullScale(MAG_FS_t);
    status_t MAG_BlockDataUpdate(MAG_BDU_t);
    status_t MAG_XY_AxOperativeMode(MAG_OMXY_t);
    status_t MAG_Z_AxOperativeMode(MAG_OMZ_t);
    status_t MAG_SetMode(MAG_MD_t);
    status_t ACC_SetFullScale(ACC_FS_t);
    status_t ACC_BlockDataUpdate(ACC_BDU_t);
    status_t ACC_EnableAxis(uint8_t);
    status_t ACC_SetODR(ACC_ODR_t);

    status_t ACC_Status_Flags(uint8_t&);
    status_t ACC_GetAccRaw(AxesRaw_t&);
    float    readAccel(AXIS_t); // Reads the accelerometer data from IC

    status_t MAG_GetMagRaw(AxesRaw_t&);
    status_t MAG_TemperatureEN(MAG_TEMP_EN_t);    
    status_t MAG_XYZ_AxDataAvailable(MAG_XYZDA_t&);
    float readMag(AXIS_t); // Reads the magnetometer data from IC

    status_t MAG_ReadReg(MAG_REG_t, uint8_t&);
    uint8_t MAG_WriteReg(MAG_REG_t, uint8_t);
    status_t ACC_ReadReg(ACC_REG_t, uint8_t&);
    uint8_t  ACC_WriteReg(ACC_REG_t, uint8_t);
};

#endif
