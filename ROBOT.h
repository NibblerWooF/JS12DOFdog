#ifndef ROBOT_H__
#define ROBOT_H__

#include <math.h>
#include <Arduino.h>
#include "FKIK.h"
#include <Adafruit_PWMServoDriver.h>
#include <SCServo.h>  

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))




#define KServoMax 180         // 髋关节
#define KServoMin 0
#define TServoMax 180         // 大腿
#define TServoMin 0
#define SServoMax 300          // 小腿
#define SServoMin 0



//------------舵机校准-----------------
extern float Servo0_Init;         // 左前腿舵机
extern float Servo1_Init;
extern float Servo2_Init;     


extern float Servo3_Init;         // 右前腿舵机
extern float Servo4_Init;
extern float Servo5_Init;        


extern float Servo6_Init;         // 左后腿舵机
extern float Servo7_Init;
extern float Servo8_Init;        


extern float Servo9_Init;         // 右后腿舵机
extern float Servo10_Init;
extern float Servo11_Init;   




// 摆线
#define XS      -10.0    //前进
#define XS2      -5.0    //后退
#define YS      -24.6    // Y原点    
#define ZS      -110.0   // Z原点 

#define XMOVE   25      // X距离
#define ZH       10     // 抬腿高度  


// 摆线
#define XS_TURN      -5.0   // X原点    
#define YS_TURN      -24.6//25.4    // Y原点     
#define ZS_TURN      -110.0    // Z原点  

#define XMOVE_TURN   25      // X距离
#define ZH_TURN       15      // 抬腿高度  

// 摆线
#define XS_RL      -10.0   // X原点    
#define XS_RL2      -10.0   // X原点    
#define YS_RL      -24.6//25.4    // Y原点    
#define ZS_RL      -110.0    // Z原点  
#define XS_RL_ATTITUDE      7    //左右横移

#define XMOVE_RL   20      // X距离
#define ZH_RL       10      // 抬腿高度   

//前后旋转矫正
#define XMOVE_TURN_QH   5      // 前进后退
#define XMOVE_TURN_ZY   5      // 左右横移


extern SCSCL sc;  // 


void angle_set(int i, float angle);  
float mapTo(float val, float I_Min, float I_Max, float O_Min, float O_Max);
int ts(int angle);
int ts2(int angle);
void servo_init();
void robot_servo_init();



void Robot_init(void);
void Robot_PowerOFF(void);
void Robot_LegUnload(uint8_t legNum);

void Robot_FK_ResetPose(void);
void Robot_IK_Stand(void);
void Robot_IK_Fall(void);


/* ------------------ 单腿控制 ------------------ */
void FK_LUMove(float posk,float posx,float posh,uint16_t Time);   //左前腿
void FK_LBMove(float posk,float posx,float posh,uint16_t Time);   //左后腿
void FK_RUMove(float posk,float posx,float posh,uint16_t Time);   //右前腿
void FK_RBMove(float posk,float posx,float posh,uint16_t Time);   //右后腿

uint8_t FK_LegServoDebug(float Angle[],uint8_t cmd,uint8_t legNum);     



void IK_LUMove(float x,float y,float z,uint16_t Time);             //左前腿
void IK_LBMove(float x,float y,float z,uint16_t Time);             //左后腿
void IK_RUMove(float x,float y,float z,uint16_t Time);             //右前腿
void IK_RBMove(float x,float y,float z,uint16_t Time);             //右后腿

void IK_LegMove(float Point[],uint8_t LEGNum,uint16_t Time,uint8_t Print);
void LegPointDebug(float point[],uint8_t cmd,uint8_t PrintPoint,uint8_t PrintAngle);
void PosToPoint(float x,float y,float z,uint16_t Time);   
void Trot(uint8_t step,uint8_t dir);
void TrotRL(uint8_t step,uint8_t dir);
void TrotTurn(uint8_t step,uint8_t dir);
void TrotMark(uint8_t step,uint8_t dir);
float Yaw_error(float Target, float Now);
float fabs1(float value);


void action_stand();
void action_fall();
void action_whril();
void action_stand_fall();
void action_sitdown();
void action_pee();
void action_wave();
void action_handshake();
void action_extend();

#endif