#ifndef SATATE_H__
#define STATE_H__

#include <math.h>
#include <Arduino.h>
#include "FKIK.h"





typedef enum
{
    GaitMode_Trot_Forward,	   //前进
    GaitMode_Trot_backward,    //后退
    GaitMode_Trot_Left,        //左横移
    GaitMode_Trot_Right,       //右横移
    GaitMode_Trot_LeftTurn,    //左转
	  GaitMode_Trot_RightTurn    //右转
}GaitMode_t;





//主状态-持续
typedef enum
{
  	MainState_Trot,                  //步态
	  MainState_Stand,                 //站立
		MainState_Fall,                  //趴下
    MainState_Mark,                  //踏步
    MainState_Attitude,              //姿态模式
    MainState_Self_stabilization,    //自稳定模式
	  MainState_Follow,                //跟随模式
	  MainState_Avoid_Obstacle         //避障模式
}MainState_t;








GaitMode_t GetGaitMode(void);
MainState_t GetMainState(void);


#endif