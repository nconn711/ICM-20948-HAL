
#include "icm_20948_controller.h"

void IMU_Reset(void) {

	IMU_Bank(0);
	I2C1_Write(IMU, 0x06, 0x80); // IMU Reset
	LL_mDelay(50);
}


void Mag_Reset(void) {

	I2C1_Write(MAG, 0x32, 0x01); // Mag Reset
	LL_mDelay(50);
}


void Get_Acc_Gyr(int16_t* output) {

	uint8_t data[12] = {0};
	I2C1_Read(IMU, 0x2D, data, 12); // read acc/gry data
	for (int i = 0; i < 6; i++) {
		output[i] = (data[i*2] << 8) | data[i*2+1]; // little-endian
	}

}


void Get_Mag(int16_t* output) {

	uint8_t data[6] = {0};
	I2C1_Write(MAG, 0x31, 0x01); // single measurement
	while ((I2C1_Read(MAG, 0x10, NULL, 1) & 0x01) != 0x01); // wait till data ready
	I2C1_Read(MAG, 0x11, data, 6); // read mag data
	uint8_t sr2 = I2C1_Read(MAG, 0x18, NULL, 1); // required
	for (int i = 0; i < 3; i++) {
		output[i] = data[i*2] | (data[i*2+1] << 8); // big-endian
	}
}

void Get_Sensor(int16_t* output) {
	Get_Acc_Gyr(output);
	Get_Mag(output + 6);
}


uint8_t I2C1_Read(uint8_t device, uint8_t RA, uint8_t* output, int registers) {

	I2C1_Start(); // start signal
	I2C1_Address(device << 1); // 7-bit address + write
	I2C1_Transmit(RA); // register address
	I2C1_Start(); // start signal
	I2C1_Address((device << 1) + 1); // 7-bit address + read
	//I2C1_NACK();
	if (output == NULL) {
		uint8_t ret = I2C1_Receive(); // receive byte
		I2C1_Stop(); // stop signal
		return ret;
	}
	I2C1_ACK();
	for (int i = 0; i < registers; i++) {
		if (i == registers - 1) {
			I2C1_NACK();
		}
		output[i] = I2C1_Receive(); // receive byte
	}
	I2C1_Stop(); // stop signal
	return output[0];
}

void I2C1_Write(uint8_t device, uint8_t RA, uint8_t data) {
	I2C1_Start(); // start signal
	I2C1_Address(device << 1); // 7-bit address + write
	I2C1_Transmit(RA); // register address
	I2C1_Transmit(data); // write data
	I2C1_Stop();
}

void IMU_Wake(void) {
	I2C1_Write(0x69, 0x06, 0x01);
	LL_mDelay(50);
}

void IMU_Bank(int bank) {
	I2C1_Write(0x69, 0x7F, bank);
}

void IMU_Bypass(void) {
	I2C1_Write(IMU, 0x0F, 0x02);
}

void I2C1_Start(void) {

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));
}

void I2C1_Stop(void) {

	LL_I2C_GenerateStopCondition(I2C1);
}

void I2C1_Address(uint8_t address) {

	LL_I2C_TransmitData8(I2C1, address);
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);
}

void I2C1_Transmit(uint8_t data) {

	while (!LL_I2C_IsActiveFlag_TXE(I2C1));
	LL_I2C_TransmitData8(I2C1, data);
	while (!LL_I2C_IsActiveFlag_BTF(I2C1));
}

uint8_t I2C1_Receive(void) {

	while (!LL_I2C_IsActiveFlag_RXNE(I2C1));
	return LL_I2C_ReceiveData8(I2C1);
}

void I2C1_ACK(void) {

	SET_BIT(I2C1->CR1, LL_I2C_ACK);
}

void I2C1_NACK(void) {

	CLEAR_BIT(I2C1->CR1, LL_I2C_ACK);
}
