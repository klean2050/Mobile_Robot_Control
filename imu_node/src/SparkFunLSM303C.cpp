#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <stdlib.h>
#include "SparkFunLSM303C.h"

// Public methods

LSM303C::~LSM303C()
{
	close(I2C_A_fd);
        close(I2C_M_fd);
}

status_t LSM303C::begin()
{
	//return
	begin(
        // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
        MAG_DO_80_Hz,
        // Initialize magnetic field full scale to +/-16 gauss
        MAG_FS_16_Ga,
        // Enabling block data updating
        MAG_BDU_ENABLE,
        // Initialize magnetometer X/Y axes ouput data rate to high-perf mode
        MAG_OMXY_ULTRA_HIGH_PERFORMANCE,
        // Initialize magnetometer Z axis performance mode
        MAG_OMZ_ULTRA_HIGH_PERFORMANCE,
        // Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
        MAG_MD_CONTINUOUS,
        // Initialize acceleration full scale to +/-2g
	ACC_FS_2g,
	// Enable block data updating
	ACC_BDU_ENABLE,
	// Enable X, Y, and Z accelerometer axes
	ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
	// Initialize accelerometer output data rate to 100 Hz (turn on device)
	ACC_ODR_100_Hz
	);
}

status_t LSM303C::begin(MAG_DO_t modr, MAG_FS_t mfs,
    MAG_BDU_t mbu, MAG_OMXY_t mxyodr, MAG_OMZ_t mzodr, MAG_MD_t mm,
    ACC_FS_t afs, ACC_BDU_t abu, uint8_t aea, ACC_ODR_t aodr)
{
	uint8_t successes = 0;
        uint8_t value;
	int adapter_nr = 1;
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	if((I2C_A_fd = open(filename, O_RDWR)) < 0) {
		printf("Failed to open the i2c bus1\n");
		exit(1);
	}

	if(ioctl(I2C_A_fd, I2C_SLAVE, ACC_I2C_ADDR) < 0) {
		printf("Failed to acquire bus access and/or talk to slave\n");
		exit(1);
	}
        
	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	if((I2C_M_fd = open(filename, O_RDWR)) < 0) {
		printf("Failed to open the i2c bus2\n");
		exit(1);
	}

	if(ioctl(I2C_M_fd, I2C_SLAVE, MAG_I2C_ADDR) < 0) {
		printf("Failed to acquire bus access and/or talk to slave\n");
		exit(1);
	}
        
    ////////// Initialize Magnetometer //////////
    // Initialize magnetometer output data rate
    successes += MAG_SetODR(modr);
    // Initialize magnetic field full scale
    successes += MAG_SetFullScale(mfs);
    // Enabling block data updating
    successes += MAG_BlockDataUpdate(mbu);
    // Initialize magnetometer X/Y axes ouput data rate
    successes += MAG_XY_AxOperativeMode(mxyodr);
    // Initialize magnetometer Z axis performance mode
    successes += MAG_Z_AxOperativeMode(mzodr);
    // Initialize magnetometer run mode.
    successes += MAG_SetMode(mm);

  	////////// Initialize Accelerometer //////////
  	// Initialize acceleration full scale
	successes += ACC_SetFullScale(afs);
  	// Enable block data updating
	successes += ACC_BlockDataUpdate(abu);
  	// Enable X, Y, and Z accelerometer axes
	successes += ACC_EnableAxis(aea);
  	// Initialize accelerometer output data rate
	successes += ACC_SetODR(aodr);

	MAG_ReadReg(MAG_CTRL_REG1, value);	
	printf("M1:%u\n",value);
	MAG_ReadReg(MAG_CTRL_REG2, value);	
	printf("M2:%u\n",value);
	MAG_ReadReg(MAG_CTRL_REG3, value);	
	printf("M3:%u\n",value);
	MAG_ReadReg(MAG_CTRL_REG4, value);	
	printf("M4:%u\n",value);
	MAG_ReadReg(MAG_CTRL_REG5, value);	
	printf("M5:%u\n",value);
	ACC_ReadReg(ACC_CTRL1, value);	
	printf("A1:%u\n",value);
	ACC_ReadReg(ACC_CTRL2, value);	
	printf("A2:%u\n",value);
	ACC_ReadReg(ACC_CTRL3, value);	
	printf("A3:%u\n",value);
	ACC_ReadReg(ACC_CTRL4, value);	
	printf("A4:%u\n",value);
	ACC_ReadReg(ACC_CTRL5, value);	
	printf("A5:%u\n",value);	
	ACC_ReadReg(ACC_CTRL6, value);	
	printf("A6:%u\n",value);	
	ACC_ReadReg(ACC_CTRL7, value);	
	printf("A7:%u\n",value);
	printf("successes:%u\n",successes);
	
	return (successes == IMU_SUCCESS) ? IMU_SUCCESS : IMU_HW_ERROR;
}

float LSM303C::readMagX()
{
  return readMag(xAxis);
}

float LSM303C::readMagY()
{
  return readMag(yAxis);
}

float LSM303C::readMagZ()
{
  return readMag(zAxis);
}

float LSM303C::readAccelX()
{
 	return readAccel(xAxis);
	uint8_t flag_ACC_STATUS_FLAGS;
	status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
	
	if (response != IMU_SUCCESS)
	{
		printf("Error on readAccelX: %s\n", AERROR);
		return NAN;
	}
	
  	// Check for new data in the status flags with a mask
  	// If there isn't new data use the last data read.
  	// There are valid cases for this, like reading faster than refresh rate.
	if (flag_ACC_STATUS_FLAGS & ACC_X_NEW_DATA_AVAILABLE)
	{
		uint8_t valueL;
		uint8_t valueH;

		if ( ACC_ReadReg(ACC_OUT_X_H, valueH) )
		{
			return IMU_HW_ERROR;
		}
		
		if ( ACC_ReadReg(ACC_OUT_X_L, valueL) )
		{
			return IMU_HW_ERROR;
		}
		
		//printf("Fresh raw data\n");

		//convert from LSB to mg
		return int16_t(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
	}

  	// Should never get here
	printf("Returning NAN1\n");
	return NAN;
}

float LSM303C::readAccelY()
{
 	return readAccel(yAxis);
	uint8_t flag_ACC_STATUS_FLAGS;
	status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
	
	if (response != IMU_SUCCESS)
	{
		printf("Error on readAccelY: %s\n", AERROR);
		return NAN;
	}
	
  	// Check for new data in the status flags with a mask
  	// If there isn't new data use the last data read.
  	// There are valid cases for this, like reading faster than refresh rate.
	if (flag_ACC_STATUS_FLAGS & ACC_Y_NEW_DATA_AVAILABLE)
	{
		uint8_t valueL;
		uint8_t valueH;

		if ( ACC_ReadReg(ACC_OUT_Y_H, valueH) )
		{
			return IMU_HW_ERROR;
		}
		
		if ( ACC_ReadReg(ACC_OUT_Y_L, valueL) )
		{
			return IMU_HW_ERROR;
		}

		//printf("Fresh raw data\n");

		//convert from LSB to mg
		return int16_t(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
	}

  	// Should never get here
	printf("Returning NAN2\n");
	return NAN;
}

float LSM303C::readAccelZ()
{
 	return readAccel(zAxis);
	uint8_t flag_ACC_STATUS_FLAGS;
	status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
	
	if (response != IMU_SUCCESS)
	{
		printf("Error on readAccelZ: %s\n", AERROR);
		return NAN;
	}
	
  	// Check for new data in the status flags with a mask
  	// If there isn't new data use the last data read.
  	// There are valid cases for this, like reading faster than refresh rate.
	if (flag_ACC_STATUS_FLAGS & ACC_Z_NEW_DATA_AVAILABLE)
	{
		uint8_t valueL;
		uint8_t valueH;

		if ( ACC_ReadReg(ACC_OUT_Z_H, valueH) )
		{
			return IMU_HW_ERROR;
		}
		
		if ( ACC_ReadReg(ACC_OUT_Z_L, valueL) )
		{
			return IMU_HW_ERROR;
		}
		
		//printf("Fresh raw data\n");

		//convert from LSB to mg
		return(int16_t(( (valueH << 8) | valueL )) * SENSITIVITY_ACC);
	}

  	// Should never get here
	printf("Returning NAN3\n");
	return NAN;
}


////////////////////////////////////////////////////////////////////////////////
////// Protected methods

float LSM303C::readAccel(AXIS_t dir)
{
	uint8_t flag_ACC_STATUS_FLAGS;
	status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);
	
	if (response != IMU_SUCCESS)
	{
		printf("Error on readAccel: %s\n", AERROR);
		return NAN;
	}
	
  	// Check for new data in the status flags with a mask
  	// If there isn't new data use the last data read.
  	// There are valid cases for this, like reading faster than refresh rate.
	if (flag_ACC_STATUS_FLAGS & ACC_ZYX_NEW_DATA_AVAILABLE)
	{
		response = ACC_GetAccRaw(accelData);
		//printf("Fresh raw data\n");
	}
  	//convert from LSB to mg
	switch (dir)
	{
		case xAxis:
		return accelData.xAxis * SENSITIVITY_ACC;
		break;
		case yAxis:
		return accelData.yAxis * SENSITIVITY_ACC;
		break;
		case zAxis:
		return accelData.zAxis * SENSITIVITY_ACC;
		break;
		default:
		return NAN;
	}

  	// Should never get here
	printf("Returning NAN4\n");
	return NAN;
}

float LSM303C::readMag(AXIS_t dir)
{
  MAG_XYZDA_t flag_MAG_XYZDA;
  status_t response = MAG_XYZ_AxDataAvailable(flag_MAG_XYZDA);
  
  if (response != IMU_SUCCESS)
  {
    printf("MERROR\n");
    return NAN;
  }
  
  // Check for new data in the status flags with a mask
  if (flag_MAG_XYZDA & MAG_XYZDA_YES)
  {
    response = MAG_GetMagRaw(magData);
    //printf("Fresh raw data\n");
  }
  //convert from LSB to mGauss
  switch (dir)
  {
  case xAxis:
    return magData.xAxis * SENSITIVITY_MAG;
    break;
  case yAxis:
    return magData.yAxis * SENSITIVITY_MAG;
    break;
  case zAxis:
    return magData.zAxis * SENSITIVITY_MAG;
    break;
  default:
    return NAN;
  }

  // Should never get here
  printf("Returning NAN5\n");
  return NAN;
}

status_t LSM303C::MAG_GetMagRaw(AxesRaw_t& buff)
{
  uint8_t valueL;
  uint8_t valueH;
  
  if( MAG_ReadReg(MAG_OUTX_L, valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTX_H, valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff.xAxis = (int16_t)( (valueH << 8) | valueL );
  
  if( MAG_ReadReg(MAG_OUTY_L, valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTY_H, valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff.yAxis = (int16_t)( (valueH << 8) | valueL );
  
  if( MAG_ReadReg(MAG_OUTZ_L, valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( MAG_ReadReg(MAG_OUTZ_H, valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff.zAxis = (int16_t)( (valueH << 8) | valueL );

  return IMU_SUCCESS;
}

// Methods required to get device up and running
status_t LSM303C::MAG_SetODR(MAG_DO_t val)
{
  uint8_t value;

  if(MAG_ReadReg(MAG_CTRL_REG1, value))
  {
    printf("Failed Read from MAG_CTRL_REG1\n");
    return IMU_HW_ERROR;
  }

  // Mask and only change DO0 bits (4:2) of MAG_CTRL_REG1
  value &= ~MAG_DO_80_Hz;
  value |= val;

  if(MAG_WriteReg(MAG_CTRL_REG1, value))
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_SetFullScale(MAG_FS_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG2, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_FS_16_Ga; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG2, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_BlockDataUpdate(MAG_BDU_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG5, value) )
  {
    return IMU_HW_ERROR;
  }


  value &= ~MAG_BDU_ENABLE; //mask
  value |= val;		

  if ( MAG_WriteReg(MAG_CTRL_REG5, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_XYZ_AxDataAvailable(MAG_XYZDA_t& value)
{
  if ( MAG_ReadReg(MAG_STATUS_REG, (uint8_t&)value) )
  {
    return IMU_HW_ERROR;
  }

  value = (MAG_XYZDA_t)((int8_t)value & (int8_t)MAG_XYZDA_YES);

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_XY_AxOperativeMode(MAG_OMXY_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }
	
  value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_Z_AxOperativeMode(MAG_OMZ_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG4, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;	

  if ( MAG_WriteReg(MAG_CTRL_REG4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_SetMode(MAG_MD_t val)
{
  uint8_t value;

  if ( MAG_ReadReg(MAG_CTRL_REG3, value) )
  {
    printf("Failed to read MAG_CTRL_REG3.\n");
    return IMU_HW_ERROR;
  }

  value &= ~MAG_MD_POWER_DOWN_2;
  value |= val;		

  if ( MAG_WriteReg(MAG_CTRL_REG3, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}


status_t LSM303C::ACC_SetFullScale(ACC_FS_t val)
{
	uint8_t value;

	if ( ACC_ReadReg(ACC_CTRL4, value) )
	{
		printf("Failed ACC read\n");
		return IMU_HW_ERROR;
	}
	
	value &= ~ACC_FS_8g;
	value |= val;

	if ( ACC_WriteReg(ACC_CTRL4, value) )
	{
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;
}

status_t LSM303C::ACC_BlockDataUpdate(ACC_BDU_t val)
{
	uint8_t value;

	if ( ACC_ReadReg(ACC_CTRL1, value) )
	{
		printf("Failed on ACC_BlockDataUpdate: %s\n", AERROR);
		return IMU_HW_ERROR;
	}

	value &= ~ACC_BDU_ENABLE;
	value |= val;	

	if ( ACC_WriteReg(ACC_CTRL1, value) )
	{
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;
}

status_t LSM303C::ACC_EnableAxis(uint8_t val)
{
	uint8_t value;

	if ( ACC_ReadReg(ACC_CTRL1, value) )
	{
		printf("Failed on ACC_EnableAxis: %s\n", AERROR);
		return IMU_HW_ERROR;
	}

	value &= ~0x07;
	value |= val;	

	if ( ACC_WriteReg(ACC_CTRL1, value) )
	{
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;
}

status_t LSM303C::ACC_SetODR(ACC_ODR_t val)
{
	uint8_t value;

	if ( ACC_ReadReg(ACC_CTRL1, value) )
	{
		printf("Failed on ACC_SetODR: %s\n", AERROR);
		return IMU_HW_ERROR;
	}

	value &= ~ACC_ODR_MASK;
	value |= val;	
	
	if ( ACC_WriteReg(ACC_CTRL1, value) )
	{
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;
}

status_t LSM303C::MAG_TemperatureEN(MAG_TEMP_EN_t val){
  uint8_t value;

  if( MAG_ReadReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_TEMP_EN_ENABLE; //mask
  value |= val;	

  if( MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C::MAG_ReadReg(MAG_REG_t reg, uint8_t& data)
{
  status_t ret = IMU_GENERIC_ERROR;

  ret = I2C_ByteRead(I2C_M_fd, reg, data);

  return ret;
}

uint8_t  LSM303C::MAG_WriteReg(MAG_REG_t reg, uint8_t data)
{
  uint8_t ret;

  ret = I2C_ByteWrite(I2C_M_fd, reg, data);

  return ret;
}

status_t LSM303C::ACC_ReadReg(ACC_REG_t reg, uint8_t& data)
{
	//printf("Reading address %x\n", reg);
	status_t ret;
	
	ret = I2C_ByteRead(I2C_A_fd, reg, data);

	return ret;
}

uint8_t  LSM303C::ACC_WriteReg(ACC_REG_t reg, uint8_t data)
{
	uint8_t ret;

	ret = I2C_ByteWrite(I2C_A_fd, reg, data);
	
	return ret;
}





uint8_t  LSM303C::I2C_ByteWrite(int slaveAddress, uint8_t reg,
	uint8_t data)
{
	uint8_t ret = IMU_GENERIC_ERROR;
  	//Wire.beginTransmission(slaveAddress);  // Initialize the Tx buffer
  	// returns num bytes written
  	ret = i2c_smbus_write_byte_data(slaveAddress, reg, data);
  	if (ret)
  	{	
		printf("Errorn on I2C_ByteWrite: %d\n", ret); 
  		ret = IMU_HW_ERROR;
	}
	else
	{
		//printf("Wrote: %x\n", data);
	}
	return ret;
}

status_t LSM303C::I2C_ByteRead(int slaveAddress, uint8_t reg,
	uint8_t& data)
{
	status_t ret = IMU_GENERIC_ERROR;
	//printf("Reading from I2C address: %x, register: %x\n", slaveAddress, reg);
  	//Wire.beginTransmission(slaveAddress); // Initialize the Tx buffer
  	__s32 value = i2c_smbus_read_byte_data(slaveAddress, reg);
  	if (value < 0)  // Put slave register address in Tx buff
  	{
  		printf("Error on I2C_ByteRead\n");
  		return IMU_HW_ERROR;	
	}
	else
	{
		data = value;
		//printf("Read: %x\n", data);
		ret = IMU_SUCCESS;
	}
	return ret;
}

status_t LSM303C::ACC_Status_Flags(uint8_t& val)
{
	//printf("Getting accel status\n");
	if( ACC_ReadReg(ACC_STATUS, val) )
	{
		printf("Error on ACC_Status_Flags: %s\n", AERROR);
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;
}

status_t LSM303C::ACC_GetAccRaw(AxesRaw_t& buff)
{
	uint8_t valueL;
	uint8_t valueH;

	if ( ACC_ReadReg(ACC_OUT_X_H, valueH) )
	{
		return IMU_HW_ERROR;
	}

	if ( ACC_ReadReg(ACC_OUT_X_L, valueL) )
	{
		return IMU_HW_ERROR;
	}

	buff.xAxis = (int16_t)( (valueH << 8) | valueL );

	if ( ACC_ReadReg(ACC_OUT_Y_H, valueH) )
	{
		return IMU_HW_ERROR;
	}

	if ( ACC_ReadReg(ACC_OUT_Y_L, valueL) )
	{
		return IMU_HW_ERROR;
	}

	buff.yAxis = (int16_t)( (valueH << 8) | valueL );

	if ( ACC_ReadReg(ACC_OUT_Z_H, valueH) )
	{
		return IMU_HW_ERROR;
	}

	if ( ACC_ReadReg(ACC_OUT_Z_L, valueL) )
	{
		return IMU_HW_ERROR;
	}

	buff.zAxis = (int16_t)( (valueH << 8) | valueL ); 

	return IMU_SUCCESS;
}
