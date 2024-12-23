#ifndef IMU_H__
#define IMU_H__


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"


#define WIT_HAL_OK      (0)     
#define WIT_HAL_BUSY    (-1)    
#define WIT_HAL_TIMEOUT (-2)    
#define WIT_HAL_ERROR   (-3)   
#define WIT_HAL_NOMEM   (-4)    
#define WIT_HAL_EMPTY   (-5)    
#define WIT_HAL_INVAL   (-6)    

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3



typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
int32_t WitSerialWriteRegister(SerialWrite write_func);
void WitSerialDataIn(uint8_t ucData);
void IMU_INIT(void);
void IMU_GetData(void);
void CopeCmdData(unsigned char ucData);
extern int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
extern int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
extern void ShowHelp(void);
extern void CmdProcess(void);
extern void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
extern void AutoScanSensor(void);
extern float fAcc[3], fGyro[3], fAngle[3];

typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);


typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
int32_t WitCanWriteRegister(CanWrite write_func);

typedef void (*DelaymsCb)(uint16_t ucMs);
int32_t WitDelayMsRegister(DelaymsCb delayms_func);


void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
int32_t WitRegisterCallBack(RegUpdateCb update_func);
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);
void WitDeInit(void);



/**
  ******************************************************************************
  * @file    wit_c_sdk.h
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
