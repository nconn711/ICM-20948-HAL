
#include "main.h"

#define IMU		0x69
#define MAG		0x0C

void IMU_Reset(void);
void Mag_Reset(void);
void Get_Acc_Gyr(int16_t* output);
void Get_Mag(int16_t* output);
void Get_Sensor(int16_t* output);

void IMU_Wake(void);
void IMU_Bank(int bank);
void IMU_Bypass(void);
uint8_t I2C1_Read(uint8_t device, uint8_t RA, uint8_t* output, int registers);
void I2C1_Write(uint8_t device, uint8_t RA, uint8_t data);


void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Address(uint8_t address);
void I2C1_Transmit(uint8_t data);
uint8_t I2C1_Receive(void);
void I2C1_ACK(void);
void I2C1_NACK(void);

