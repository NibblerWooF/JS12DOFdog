#include "ROBOT.h"
#include <Wire.h>
#include <SCServo.h>
#include "FKIK.h"
#include "IMU.h"
#include "Attitude.h"


SCSCL sc;

#define S_RXD 16
#define S_TXD 17


int cal_flag=1;
int offset_flag=0;


 //映射函数
float mapTo(float val, float I_Min, float I_Max, float O_Min, float O_Max){
  return(((val-I_Min)*((O_Max-O_Min)/(I_Max-I_Min)))+O_Min);
}
int ts(int angle){
  float an = mapTo((float)angle,0,300,40,983);
  return an;
}

int ts2(int angle){
  float an = mapTo((float)angle,40,983,0,300);
  return an;
}



void servo_init()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  delay(20);
  Serial.print("\r\n********************** The servo initialization is complete  ************************\r\n");
  
}


void angle_set(int i, float angle)
{
  if(angle>180) angle = 180;
  if(angle<0) angle = 0;
  sc.WritePosEx(i, ts(angle), 1000, 0); 
}

//舵机函数
void robot_servo_init()
{
    IK_LUMove(L2,L0,-L1,0);
    IK_LBMove(L2,L0,-L1,0);
    IK_RUMove(L2,-L0,-L1,0);
    IK_RBMove(L2,-L0,-L1,0);
}

//舵机初始化
void Robot_FK_ResetPose(void){
  FK_RUMove(90,90,90,0);
  FK_RBMove(90,90,90,0);
  FK_LUMove(90,90,90,0);
  FK_LBMove(90,90,90,0);
}




//左前腿
void FK_LUMove(float posk,float posx,float posh,uint16_t Time){
    sc.WritePosEx(0 ,ts(constrain(posk,KServoMin,KServoMax)),1000,0);       delay(Time*100);   //K
    sc.WritePosEx(1 ,ts(constrain(posx,TServoMin,TServoMax)),1000,0);       delay(Time*100);   //X
    sc.WritePosEx(2 ,ts(constrain(posh,SServoMin,SServoMax)),1000,0);       delay(Time*100);   //H
}
//右后腿
void FK_RBMove(float posk,float posx,float posh,uint16_t Time){
    sc.WritePosEx(9 ,ts(constrain(posk,KServoMin,KServoMax)),1000,0);       delay(Time*100);   //K  肩部
    sc.WritePosEx(10 ,ts(constrain(posx,TServoMin,TServoMax)),1000,0);       delay(Time*100);   //X  大腿
    sc.WritePosEx(11,ts(constrain(posh,SServoMin,SServoMax)),1000,0);       delay(Time*100);   //H  小腿
}

//左后腿
void FK_LBMove(float posk,float posx,float posh,uint16_t Time){
    sc.WritePosEx(6 ,ts(constrain(posk,KServoMin,KServoMax)),1000,0);       delay(Time*100);   //K
    sc.WritePosEx(7 ,ts(constrain(posx,TServoMin,TServoMax)),1000,0);       delay(Time*100);   //X
    sc.WritePosEx(8 ,ts(constrain(posh,SServoMin,SServoMax)),1000,0);       delay(Time*100);   //H
}



*uint8_t FK_LegServoDebug(float Angle[],uint8_t cmd,uint8_t legNum){

    uint8_t offset      = 1;

    switch(cmd){
        //K
        case '4': Angle[0] += offset;break;
        case '1': Angle[0] -= offset;break;
        //T
        case '5': Angle[1] += offset;break;
        case '2': Angle[1] -= offset;break;
        //S
        case '6': Angle[2] += offset;break;
        case '3': Angle[2] -= offset;break;
        //Leg
        case '0':
            Robot_LegUnload(legNum);

            legNum++;
            if(legNum == 5){
                legNum = 1;
            }

            Angle[0]    = 90;
            Angle[1]    = 135;
            Angle[2]    = 45;

            printf("Now LegNum:%d\n",legNum);
            break;
    }

    printf("Angle:%5.2f -- %5.2f -- %5.2f\n",Angle[0],Angle[1],Angle[2]);

    switch(legNum){
        case 1: FK_LUMove(Angle[0],Angle[1],Angle[2],0);  break;   //1--->左前
        case 2: FK_LBMove(Angle[0],Angle[1],Angle[2],0);  break;   //2--->左后
        case 3: FK_RBMove(Angle[0],Angle[1],Angle[2],0);  break;   //3--->右后
        case 4: FK_RUMove(Angle[0],Angle[1],Angle[2],0);  break;   //4--->右前
    }

    return legNum;
}


void IK_LegMove(float Point[],uint8_t LEGNum,uint16_t Time,uint8_t Print){
    float angle[3] = {};

    IK(Point[0],Point[1],Point[2],angle);

    angle[0] = 90.0 + angle[0];
    angle[1] = 90.0 + angle[1];
    //angle[2] = 00.0 + angle[1] - angle[2];
    angle[2] = 90.0 + angle[2];

    if(Print){
        printf("Angle:%5.2f -- %5.2f -- %5.2f\n",angle[0],angle[1],angle[2]);
    }

    switch(LEGNum){
        case 1: FK_LUMove(angle[0],angle[1],angle[2],Time);  break;   //1--->左前
        case 2: FK_LBMove(angle[0],angle[1],angle[2],Time);  break;   //2--->左后
        case 3: FK_RBMove(angle[0],angle[1],angle[2],Time);  break;   //3--->右后
        case 4: FK_RUMove(angle[0],angle[1],angle[2],Time);  break;   //4--->右前
    }
}

void LegPointDebug(float point[],uint8_t cmd,uint8_t PrintPoint,uint8_t PrintAngle){

    uint8_t offset     = 10;
    uint8_t legNum     = 1;

    switch(cmd){
        //X
        case '4': point[0] += offset;break;
        case '1': point[0] -= offset;break;
        //Y
        case '5': point[1] += offset;break;
        case '2': point[1] -= offset;break;
        //Z
        case '6': point[2] += offset;break;
        case '3': point[2] -= offset;break;
        //Leg
        case '0':
            legNum++;
            if(legNum == 5){
                legNum = 1;
            }
            printf("LegNum:%d\n",legNum);
            break;
    }

    if(PrintPoint)
        printf("Point:%5.3f -- %5.3f -- %5.3f\n",point[0],point[1],point[2]);

    IK_LegMove(point,legNum,0,PrintAngle);
}

// 姿态坐标
void PosToPoint(float x,float y,float z,uint16_t Time){
    IK_RUMove(XS+x, YS+y, ZS+z,Time);
    IK_LUMove(XS+x, YS-y, ZS+z,Time);
    IK_RBMove(XS+x, YS+y, ZS+z,Time);
    IK_LBMove(XS+x, YS-y, ZS+z,Time);
}


float fabs1(float value) {
    if (value < 0) 
	{
        return -value;
    } 
	else 
	{
        return value;
    }
}


/* ------------------ 偏航角误差&左右偏判断 ------------------ */
float Yaw_error(float Target, float Now)
 {
    static float error;
    if (Target > 0 ) 
	{
        if (Now <= 0) 
		{
            if (fabs(Now) < (180 - Target))
			{
                error = fabs(fabs(Now) + Target);
                if(cal_flag == 1)
                {
                offset_flag=2;//右偏
                //Serial.print("右偏1111111111111111111111\n");
                }
            } 
			else 
			{
                error = fabs(-(180 - Target) - (180 - fabs(Now)));
                if(cal_flag == 1)
                {
                offset_flag=1;//左偏
                //Serial.print("左偏1111111111111111111111\n");
                }
            }
        } 
		else 
		{
            if (Now > 0) 
			{
                error = fabs(Target - Now);
                if((Target < Now) && (cal_flag == 1))
                {
                  offset_flag=1;//左偏
                  //Serial.print("左偏1111111111111111111111\n");
                }
                if((Target > Now) && (cal_flag == 1))
                {
                  offset_flag=2;//右偏
                  //Serial.print("右偏1111111111111111111111\n");
                }
            }
        }
    } 
	else if (Target < 0)
	 {
        if (Now > 0) 
		{
            if (Now > Target + 180)
			{
                error = fabs((180 - Now) + (180 - fabs(Target)));
                if(cal_flag == 1)
                {
                offset_flag=2;//右偏
                //Serial.print("右偏1111111111111111111111\n");
                }
            }
			 else if (Now < Target + 180) 
			{
                error = fabs(-(fabs(Target) + Now));
                if(cal_flag == 1)
                {
                offset_flag=1;//左偏
                //Serial.print("左偏1111111111111111111111\n");
                }
            }
        } 
		else if (Now < 0) 
		{
            error = fabs(-(fabs(Target) - fabs(Now)));
                if(Target < Now && (cal_flag == 1))
                {
                  offset_flag=1;//左偏
                  //Serial.print("左偏1111111111111111111111\n");
                }
                if(Target > Now && (cal_flag == 1))
                {
                  offset_flag=2;//右偏
                  //Serial.print("右偏1111111111111111111111\n");
                }
        }
    }

    
    return error;
}




void Trot(uint8_t step,uint8_t dir){

    uint8_t DSD = 0;   

    float sigma,xep_b,xep_z,yep_b,yep_z,zep;
    float t = 0;  

    float speed = 0.02;                         

    float Ts        = 1;                        
    float fai       = 0.5;                      
    float offset    = XMOVE;                       
    float offset_turn    = XMOVE_TURN_QH;                      


    float xs,xf; 
    if(dir==0)
    {
       xs = XS2 - offset;
       xf = XS2 + offset;  
    }
    else
    {
      xs = XS - offset;
      xf = XS + offset;   
    }
    float zs = ZS,          zh = ZH;            
    float ys = YS_TURN - offset_turn, yf = YS_TURN + offset_turn;   

    //足端
    float x1 = XS, x2 = XS, x3 = XS, x4 = XS;
    float y1 = YS, y2 = YS, y3 = YS, y4 = YS;
    float z1 = ZS, z2 = ZS, z3 = ZS, z4 = ZS;
    IMU_GetData();
    float yaw_first = fAngle[2];
    float yaw_new;
    float err;
    int cal_flag=0;

    // 单个步态
    for(uint8_t i = 0;i < step;i ++){
        while(t < 1){

          IMU_GetData();
          yaw_new = fAngle[2];
          err = Yaw_error(yaw_first,yaw_new);

            if(err > 5.0)
            {
              cal_flag=1;
            }
            if(cal_flag==1 && err < 1.0)
            {
              cal_flag=0;
              offset_flag=0;

            if(t <= Ts*fai){
                sigma  = 2*PI*t/(fai*Ts);
                zep    = zh*(1-cos(sigma))/2+zs;                    

                xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI) + xs;  
                xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI) + xf;  


                yep_b  = (yf - ys)*(sigma-sin(sigma))/(2*PI) + ys;  
                yep_z  = (ys - yf)*(sigma-sin(sigma))/(2*PI) + yf;  
                if(dir != 2){
                    z1 = zep;
                    z2 = zs;
                    z3 = zep;
                    z4 = zs;
                }

                if(dir == 1){
                    x1 = xep_b;
                    x2 = xep_z;
                    x3 = xep_b;
                    x4 = xep_z;

                    if(cal_flag==1 && offset_flag==1)
                    {
                    y1 = yep_z;
                    y2 = yep_z;
                    y3 = yep_z;
                    y4 = yep_z;
                    }

                    if(cal_flag==1 && offset_flag==2)
                    {
                      y1 = yep_b;
                      y2 = yep_b;
                      y3 = yep_b;
                      y4 = yep_b;
                    }

                    if(cal_flag==0)
                    {
                    y1 = YS;
                    y2 = YS;
                    y3 = YS;
                    y4 = YS;
                    }
                }else if(dir == 0){
                    x1 = xep_z;
                    x2 = xep_b;
                    x3 = xep_z;
                    x4 = xep_b;



                    if(cal_flag==1 && offset_flag==1)
                    {
                    y1 = yep_z;
                    y2 = yep_z;
                    y3 = yep_z;
                    y4 = yep_z;
                    }

                    if(cal_flag==1 && offset_flag==2)
                    {
                      y1 = yep_b;
                      y2 = yep_b;
                      y3 = yep_b;
                      y4 = yep_b;
                    }

                    if(cal_flag==0)
                    {
                    y1 = YS;
                    y2 = YS;
                    y3 = YS;
                    y4 = YS;
                    }

                }else if(dir == 2){
                    x1 = XS;
                    x2 = XS;
                    x3 = XS;
                    x4 = XS;
                }
            }

            
            if(Ts*fai < t && t < Ts){
                sigma  = 2*PI*(t - Ts*fai)/(fai*Ts);
                zep    = zh*(1-cos(sigma))/2 + zs;                  

                xep_b  = (xf - xs)*(sigma-sin(sigma))/(2*PI) + xs;  
                xep_z  = (xs - xf)*(sigma-sin(sigma))/(2*PI) + xf;  

                yep_b  = (yf - ys)*(sigma-sin(sigma))/(2*PI) + ys;  
                yep_z  = (ys - yf)*(sigma-sin(sigma))/(2*PI) + yf;  



                if(dir != 2){
                    z1 = zs;
                    z2 = zep;
                    z3 = zs;
                    z4 = zep;
                }

                if(dir == 1){
                    x1 = xep_z;
                    x2 = xep_b;
                    x3 = xep_z;
                    x4 = xep_b;
                    if(cal_flag==1 && offset_flag==1)//左
                    {
                      y1 = yep_b;
                      y2 = yep_b;
                      y3 = yep_b;
                      y4 = yep_b;
                    }
                    if(cal_flag==1 && offset_flag==2)//右
                    {
                      y1 = yep_z;
                      y2 = yep_z;
                      y3 = yep_z;
                      y4 = yep_z;
                    }
                    if(cal_flag==0)
                    {
                    y1 = YS;
                    y2 = YS;
                    y3 = YS;
                    y4 = YS;
                    }
                }else if(dir == 0){
                    x1 = xep_b;
                    x2 = xep_z;
                    x3 = xep_b;
                    x4 = xep_z;

                    if(cal_flag==1 && offset_flag==1)//左
                    {
                      y1 = yep_b;
                      y2 = yep_b;
                      y3 = yep_b;
                      y4 = yep_b;
                    }
                    if(cal_flag==1 && offset_flag==2)//右
                    {
                      y1 = yep_z;
                      y2 = yep_z;
                      y3 = yep_z;
                      y4 = yep_z;
                    }
                    if(cal_flag==0)
                    {
                    y1 = YS;
                    y2 = YS;
                    y3 = YS;
                    y4 = YS;
                    }
                }else if(dir == 3){
                    x1 = XS;
                    x2 = XS;
                    x3 = XS;
                    x4 = XS;
                }
            }

            t = t + speed;
            IK_RUMove(x1,y1,z1,DSD);
            IK_LUMove(x2,-y2,z2,DSD);

            IK_LBMove(x3,-y3,z3,DSD);
            IK_RBMove(x4,y4,z4,DSD);
        }
        t = 0;
    }

    Robot_IK_Stand();
}


void TrotRL(uint8_t step,uint8_t dir){

    uint8_t DSD = 0;   

    float sigma,yep_b,yep_z,yep_b_turn,yep_z_turn,zep;
    float t = 0;

    float speed = 0.015;                        

    float Ts        = 1;
    float fai       = 0.5;                      
    float offset    = XMOVE_RL;                       
    float offset_turn    = XMOVE_TURN_ZY;                      

    float ys = YS_RL - offset, yf = YS_RL + offset;   
    float ys_turn = YS_TURN - offset_turn, yf_turn = YS_TURN + offset_turn;   
    float zs = ZS_RL,          zh = ZH_RL;            

    float x1,x2,x3,x4;
    if(dir == 0)
    {
      x1=XS_RL;
      x2=XS_RL;
      x3=XS_RL;
      x4=XS_RL;
    }
    else
    {
      x1=XS_RL2;
      x2=XS_RL2;
      x3=XS_RL2;
      x4=XS_RL2;
    }
    float y1 = YS_RL, y2 = YS_RL, y3 = YS_RL, y4 = YS_RL;
    float z1 = ZS_RL, z2 = ZS_RL, z3 = ZS_RL, z4 = ZS_RL;

    IMU_GetData();
    float yaw_first = fAngle[2];
    float yaw_new;
    float err;
    int cal_flag=0;

   
    for(uint8_t i = 0;i < step;i ++){
        while(t < 1){
          IMU_GetData();
          yaw_new = fAngle[2];
          err = abs(yaw_first - yaw_new);


            if(Yaw_error(yaw_first, yaw_new)>5.0)
            {
              cal_flag=1;
            }
            if(cal_flag==1 && Yaw_error(yaw_first, yaw_new)<0.5)
            {
              cal_flag=0;
              offset_flag=0;
            }
            
            if(t <= Ts*fai){
                sigma  = 2*PI*t/(fai*Ts);
                zep    = zh*(1-cos(sigma))/2+zs;                    
                
                yep_b  = (yf - ys)*(sigma-sin(sigma))/(2*PI) + ys;  
                yep_z  = (ys - yf)*(sigma-sin(sigma))/(2*PI) + yf;  

                
                yep_b_turn  = (yf_turn - ys_turn)*(sigma-sin(sigma))/(2*PI) + ys_turn;  
                yep_z_turn  = (ys_turn - yf_turn)*(sigma-sin(sigma))/(2*PI) + yf_turn;  

                if(dir != 2){
                    z1 = zep;
                    z2 = zs;
                    z3 = zep;
                    z4 = zs;
                }

                if(dir == 1){   

                if(offset_flag==0)
                {
                    y1 = yep_z;
                    y2 = yep_z;
                    y3 = yep_b;
                    y4 = yep_b;
                    x1=XS_RL;
                    x2=XS_RL;
                    x3=XS_RL;
                    x4=XS_RL;
                }

                if(offset_flag==1)
                {
                    y1 = yep_z;
                    y2 = yep_z;
                    y3 = yep_b;
                    y4 = yep_b;
                  x1=XS_RL+XS_RL_ATTITUDE;
                  x2=XS_RL+XS_RL_ATTITUDE;
                  x3=XS_RL+XS_RL_ATTITUDE;
                  x4=XS_RL+XS_RL_ATTITUDE;
                }

                if(offset_flag==2)
                {
                    y1 = yep_z;
                    y2 = yep_z;
                    y3 = yep_b;
                    y4 = yep_b;
                  x1=XS_RL-XS_RL_ATTITUDE;
                  x2=XS_RL-XS_RL_ATTITUDE;
                  x3=XS_RL-XS_RL_ATTITUDE;
                  x4=XS_RL-XS_RL_ATTITUDE;
                }





                }
                else if(dir == 0){
                    if(offset_flag==0)
                    {
                    y1 = yep_b;
                    y2 = yep_b;
                    y3 = yep_z;
                    y4 = yep_z;
                    x1=XS_RL2;
                    x2=XS_RL2;
                    x3=XS_RL2;
                    x4=XS_RL2;
                    }

                    if(offset_flag==1)
                   {
                      y1 = yep_b;
                      y2 = yep_b;
                      y3 = yep_z;
                      y4 = yep_z;
                      x1=XS_RL2-XS_RL_ATTITUDE;
                      x2=XS_RL2-XS_RL_ATTITUDE;
                      x3=XS_RL2-XS_RL_ATTITUDE;
                      x4=XS_RL2-XS_RL_ATTITUDE;
                    }

                    if(offset_flag==2)
                    {
                      y1 = yep_b;
                      y2 = yep_b;
                      y3 = yep_z;
                      y4 = yep_z;
                      x1=XS_RL2+XS_RL_ATTITUDE;
                      x2=XS_RL2+XS_RL_ATTITUDE;
                      x3=XS_RL2+XS_RL_ATTITUDE;
                      x4=XS_RL2+XS_RL_ATTITUDE;
                    }




                }
                else if(dir == 2){
                    y1 = YS;
                    y2 = YS;
                    y3 = YS;
                    y4 = YS;
                }
            }

            
            if(Ts*fai < t && t < Ts){
                sigma  = 2*PI*(t - Ts*fai)/(fai*Ts);
                zep    = zh*(1-cos(sigma))/2 + zs;                  

                yep_b  = (yf - ys)*(sigma-sin(sigma))/(2*PI) + ys; 
                yep_z  = (ys - yf)*(sigma-sin(sigma))/(2*PI) + yf;  

                yep_b_turn  = (yf_turn - ys_turn)*(sigma-sin(sigma))/(2*PI) + ys_turn; 
                yep_z_turn  = (ys_turn - yf_turn)*(sigma-sin(sigma))/(2*PI) + yf_turn;  
                if(dir != 2){
                    z1 = zs;
                    z2 = zep;
                    z3 = zs;
                    z4 = zep;
                }

                if(dir == 1){   

                if(offset_flag==0
                {
                    y1 = yep_b;
                    y2 = yep_b;
                    y3 = yep_z;
                    y4 = yep_z;
                    x1=XS_RL;
                    x2=XS_RL;
                    x3=XS_RL;
                    x4=XS_RL;
                }

                if(offset_flag==1)
                {
                  y1 = yep_b;
                  y2 = yep_b;
                  y3 = yep_z;
                  y4 = yep_z;
                  x1=XS_RL+XS_RL_ATTITUDE;
                  x2=XS_RL+XS_RL_ATTITUDE;
                  x3=XS_RL+XS_RL_ATTITUDE;
                  x4=XS_RL+XS_RL_ATTITUDE;
                }

                if(offset_flag==2
                {
                  y1 = yep_b;
                  y2 = yep_b;
                  y3 = yep_z;
                  y4 = yep_z;
                  x1=XS_RL-XS_RL_ATTITUDE;
                  x2=XS_RL-XS_RL_ATTITUDE;
                  x3=XS_RL-XS_RL_ATTITUDE;
                  x4=XS_RL-XS_RL_ATTITUDE;
                }





                }
                else if(dir == 0){
                    if(offset_flag==0)
                    {
                    y1 = yep_z;
                    y2 = yep_z;
                    y3 = yep_b;
                    y4 = yep_b;
                      x1=XS_RL2;
                      x2=XS_RL2;
                      x3=XS_RL2;
                      x4=XS_RL2;
                    }

                    if(offset_flag==1)
                   {
                      y1 = yep_z;
                      y2 = yep_z;
                      y3 = yep_b;
                      y4 = yep_b;                    
                      x1=XS_RL2-XS_RL_ATTITUDE;
                      x2=XS_RL2-XS_RL_ATTITUDE;
                      x3=XS_RL2-XS_RL_ATTITUDE;
                      x4=XS_RL2-XS_RL_ATTITUDE;
                    }

                    if(offset_flag==2)
                    {
                      y1 = yep_z;
                      y2 = yep_z;
                      y3 = yep_b;
                      y4 = yep_b;
                      x1=XS_RL2+XS_RL_ATTITUDE;
                      x2=XS_RL2+XS_RL_ATTITUDE;
                      x3=XS_RL2+XS_RL_ATTITUDE;
                      x4=XS_RL2+XS_RL_ATTITUDE;
                    }




                }
                else if(dir == 2){
                    y1 = YS;
                    y2 = YS;
                    y3 = YS;
                    y4 = YS;
                }
            }

            t = t + speed;

            IK_RUMove(x1,y1,z1,DSD);
            IK_LUMove(x2,-y2,z2,DSD);

            IK_LBMove(x3,-y3,z3,DSD);
            IK_RBMove(x4,y4,z4,DSD);
        }
        t = 0;
    }

    Robot_IK_Stand();
}




void action_stand()
{
  for(int i=0;i<30;i++)
  {
    attitude_control(0,0,i,0,0,0);
    delay(10);

  }
  delay(1000);
    for(int i=30;i>0;i--)
  {
    attitude_control(0,0,i,0,0,0);
    delay(10);

  }
}

