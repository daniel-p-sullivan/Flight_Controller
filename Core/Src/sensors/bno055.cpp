/*
 * bno055.cpp
 *
 *  Created on: Aug 21, 2022
 *      Author: danie
 */

#include "bno055.hpp"

BNO055::BNO055(I2C_HandleTypeDef hi2c1){
	this->i2c = hi2c1;
}

bool BNO055::Config_Sensor(void){

	static uint8_t op_mode = IMU_OP_MODE;

	hal_status = HAL_I2C_Mem_Write(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_CONFIG_R, I2C_MEMADD_SIZE_8BIT, &op_mode, I2C_MEMADD_SIZE_8BIT, (uint32_t)1000);


	if(hal_status == HAL_BUSY){

		//i2c dead lock, BNO055 is holding the bus
		this->I2C1_ClearBusyFlagErratum(&(this->i2c));
		return false;

	}

	//add in a delay to allow for the sensor to properly switch operating modes
	HAL_Delay(100);

	return true;
}

bool BNO055::Read_IMU_Calib_Status(void){

	static uint16_t calib_stat_address = 0x35;
	static uint8_t IMU_full_calib = 0x3C;
	static uint8_t calib;

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_CALIB_STAT_R, I2C_MEMADD_SIZE_8BIT, &calib, I2C_MEMADD_SIZE_8BIT, (uint32_t)1000000);

	if(hal_status != HAL_OK){

		return false;

	}
	else{

		//value of 3C means full calibration
		if(calib == IMU_FULL_CALIB){

			return true;

		}
		else{

			return false;

		}
	}

}

bool BNO055::Write_IMU_Calib_Params(void){
	//no calib needed in 9DOF mode?
	static uint8_t accel_calib_params[6] = {0, 0, 0, 0, 0, 0};
	static uint8_t gyro_calib_params[6] = {0, 0, 0, 0, 0, 0};

	hal_status = HAL_I2C_Mem_Write(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_CALIB_R, I2C_MEMADD_SIZE_8BIT * 6, accel_calib_params, I2C_MEMADD_SIZE_8BIT * 6, (uint32_t)1000);
	hal_status = HAL_I2C_Mem_Write(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_CALIB_R, I2C_MEMADD_SIZE_8BIT * 6, gyro_calib_params, I2C_MEMADD_SIZE_8BIT * 6, (uint32_t)1000);

	if(hal_status != HAL_OK){
		return false;
	}
	else{
		return true;
	}

}


bool BNO055::Read_Calib_Params(void){

	static uint16_t calib_params_address = 0x00;

	//offsets
	static uint8_t axo_lsb, axo_msb, ayo_lsb, ayo_msb, azo_lsb, azo_msb;
	static uint8_t gxo_lsb, gxo_msb, gyo_lsb, gyo_msb, gzo_lsb, gzo_msb;

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_OFFSET_X_LSB_R, I2C_MEMADD_SIZE_8BIT, &axo_lsb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_OFFSET_X_MSB_R, I2C_MEMADD_SIZE_8BIT, &axo_msb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_OFFSET_Y_LSB_R, I2C_MEMADD_SIZE_8BIT, &ayo_lsb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_OFFSET_Y_MSB_R, I2C_MEMADD_SIZE_8BIT, &ayo_msb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_OFFSET_Z_LSB_R, I2C_MEMADD_SIZE_8BIT, &azo_lsb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_OFFSET_Z_MSB_R, I2C_MEMADD_SIZE_8BIT, &azo_msb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_OFFSET_X_LSB_R, I2C_MEMADD_SIZE_8BIT, &gxo_lsb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_OFFSET_X_MSB_R, I2C_MEMADD_SIZE_8BIT, &gxo_msb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_OFFSET_Y_LSB_R, I2C_MEMADD_SIZE_8BIT, &gyo_lsb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_OFFSET_Y_MSB_R, I2C_MEMADD_SIZE_8BIT, &gyo_msb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_OFFSET_Z_LSB_R, I2C_MEMADD_SIZE_8BIT, &gzo_lsb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_OFFSET_Z_MSB_R, I2C_MEMADD_SIZE_8BIT, &gzo_msb, I2C_MEMADD_SIZE_8BIT, (uint32_t)25);

	uint16_t axo = (axo_msb << 8) | axo_lsb;
	uint16_t ayo = (ayo_msb << 8) | ayo_lsb;
	uint16_t azo = (azo_msb << 8) | azo_lsb;

	uint16_t gxo = (gxo_msb << 8) | gxo_lsb;
	uint16_t gyo = (gyo_msb << 8) | gyo_lsb;
	uint16_t gzo = (gzo_msb << 8) | gzo_lsb;

	uint16_t offsets[6];
	offsets[0] = axo;
	offsets[1] = ayo;
	offsets[2] = azo;
	offsets[3] = gxo;
	offsets[4] = gyo;
	offsets[5] = gzo;


	if(hal_status != HAL_OK){
		return false;
	}
	else{
		return true;
	}

}

IMU_Sample* BNO055::Read_IMU(void){

	//addresses for accel
	static uint8_t ax_lsb, ax_msb, ay_lsb, ay_msb, az_lsb, az_msb;
	static uint8_t gx_lsb, gx_msb, gy_lsb, gy_msb, gz_lsb, gz_msb;

	static IMU_Sample* sample_i;
	static HAL_StatusTypeDef hal_status;

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_X_LSB_R, 1, &ax_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_X_MSB_R, 1, &ax_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Y_LSB_R, 1, &ay_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Y_MSB_R, 1, &ay_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Z_LSB_R, 1, &az_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Z_MSB_R, 1, &az_msb, 1, 25);

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_X_LSB_R, 1, &gx_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_X_MSB_R, 1, &gx_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Y_LSB_R, 1, &gy_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Y_MSB_R, 1, &gy_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Z_LSB_R, 1, &gz_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Z_MSB_R, 1, &gz_msb, 1, 25);

	sample_i->a_x = ((ax_msb << 8) | ax_lsb);
	sample_i->a_y = ((ay_msb << 8) | ay_lsb);
	sample_i->a_z = ((az_msb << 8) | az_lsb);

	sample_i->g_x = ((gx_msb << 8) | gx_lsb);
	sample_i->g_y = ((gy_msb << 8) | gy_lsb);
	sample_i->g_z = ((gz_msb << 8) | gz_lsb);

	return sample_i;

}

void BNO055::Update_IMU_Sample(IMU_Sample* sample){

	//addresses for accel
	static uint8_t ax_lsb, ax_msb, ay_lsb, ay_msb, az_lsb, az_msb;
	static uint8_t gx_lsb, gx_msb, gy_lsb, gy_msb, gz_lsb, gz_msb;

	static HAL_StatusTypeDef hal_status;

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_X_LSB_R, 1, &ax_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_X_MSB_R, 1, &ax_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Y_LSB_R, 1, &ay_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Y_MSB_R, 1, &ay_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Z_LSB_R, 1, &az_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_ACC_DATA_Z_MSB_R, 1, &az_msb, 1, 25);

	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_X_LSB_R, 1, &gx_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_X_MSB_R, 1, &gx_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Y_LSB_R, 1, &gy_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Y_MSB_R, 1, &gy_msb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Z_LSB_R, 1, &gz_lsb, 1, 25);
	hal_status = HAL_I2C_Mem_Read(&(this->i2c), (BNO055_ADDRESS<<1), BNO055_GYR_DATA_Z_MSB_R, 1, &gz_msb, 1, 25);

	sample->a_x = ((ax_msb << 8) | ax_lsb);
	sample->a_y = ((ay_msb << 8) | ay_lsb);
	sample->a_z = ((az_msb << 8) | az_lsb);

	sample->g_x = ((gx_msb << 8) | gx_lsb);
	sample->g_y = ((gy_msb << 8) | gy_lsb);
	sample->g_z = ((gz_msb << 8) | gz_lsb);

}


//FROM https://electronics.stackexchange.com/questions/351972/hal-i2c-hangs-cannot-be-solved-with-standard-routine-use-to-unlock-i2c/351977
void BNO055::I2C1_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int timeout =100;
    int timeout_cnt=0;

    // 1. Clear PE bit.
    instance->Instance->CR1 &= ~(0x0001);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStruct.Mode         = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Alternate    = GPIO_AF4_I2C1;
    GPIO_InitStruct.Pull         = GPIO_PULLUP;
    GPIO_InitStruct.Speed        = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin          = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin          = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);


    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        //Move clock to release I2C
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
        asm("nop");
        HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);

    //  5. Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);

    //  7. Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN))
    {
        timeout_cnt++;
        if(timeout_cnt>timeout)
            return;
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 |= 0x8000;

    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    instance->Instance->CR1 &= ~0x8000;

    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    instance->Instance->CR1 |= 0x0001;

    // Call initialization function.
    HAL_I2C_Init(instance);
}

