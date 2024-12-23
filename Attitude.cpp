#include <math.h>
#include "Attitude.h"
#include "ROBOT.h"
#include "FKIK.h"
#include "IMU.h"


int attitude_x=0;
int attitude_y=0;
int attitude_z=0;
int attitude_roll=0;
int attitude_pitch=0;
int attitude_yaw=0;

Matrix4x4 Rx(float roll) {
      float R = roll * PI / 180; // 度转弧度
    Matrix4x4 mat = {{{1, 0, 0, 0},
                      {0, cos(R), -sin(R), 0},
                      {0, sin(R), cos(R), 0},
                      {0, 0, 0, 1}}};
    return mat;
}

Matrix4x4 Ry(float pitch) {
  float P = pitch * PI / 180; // 度转弧度
    Matrix4x4 mat = {{{cos(P), 0, sin(P), 0},
                      {0, 1, 0, 0},
                      {-sin(P), 0, cos(P), 0},
                      {0, 0, 0, 1}}};
    return mat;
}

Matrix4x4 Rz(float yaw) {
  float Y = yaw * PI / 180; // 度转弧度
    Matrix4x4 mat = {{{cos(Y), -sin(Y), 0, 0},
                      {sin(Y), cos(Y), 0, 0},
                      {0, 0, 1, 0},
                      {0, 0, 0, 1}}};
    return mat;
}

Matrix4x4 multiply(Matrix4x4 a, Matrix4x4 b) {
    Matrix4x4 result = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                result.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }
    return result;
}

Matrix4x4 Rxyz(float roll, float pitch, float yaw) {
    Matrix4x4 R = Rx(roll);
    Matrix4x4 Ryaw = Ry(pitch);
    Matrix4x4 R_combined = multiply(R, Ryaw);
    return multiply(R_combined, Rz(yaw));
}

Matrix4x4 RTmatrix(float orientation[3], float position[3]) {
    float roll = orientation[0];
    float pitch = orientation[1];
    float yaw = orientation[2];
    float x0 = position[0];
    float y0 = position[1];
    float z0 = position[2];
    
    Matrix4x4 translation = {{{1, 0, 0, x0},
                              {0, 1, 0, y0},
                              {0, 0, 1, z0},
                              {0, 0, 0, 1}}};
    
    Matrix4x4 rotation = Rxyz(roll, pitch, yaw);
    return multiply(rotation, translation);
}

void transform(float coord[3], float rotation[3], float translation[3], float result[3]) {
    Matrix4x4 vector = {{{coord[0]}, {coord[1]}, {coord[2]}, {1}}};
    Matrix4x4 RT = RTmatrix(rotation, translation);
    
    Matrix4x4 tranformVector;
    tranformVector.m[0][0] = RT.m[0][0] * vector.m[0][0] + RT.m[0][1] * vector.m[1][0] + RT.m[0][2] * vector.m[2][0] + RT.m[0][3] * vector.m[3][0];
    tranformVector.m[1][0] = RT.m[1][0] * vector.m[0][0] + RT.m[1][1] * vector.m[1][0] + RT.m[1][2] * vector.m[2][0] + RT.m[1][3] * vector.m[3][0];
    tranformVector.m[2][0] = RT.m[2][0] * vector.m[0][0] + RT.m[2][1] * vector.m[1][0] + RT.m[2][2] * vector.m[2][0] + RT.m[2][3] * vector.m[3][0];
    tranformVector.m[3][0] = RT.m[3][0] * vector.m[0][0] + RT.m[3][1] * vector.m[1][0] + RT.m[3][2] * vector.m[2][0] + RT.m[3][3] * vector.m[3][0];
    
    result[0] = Round(tranformVector.m[0][0],2);
    result[1] = Round(tranformVector.m[1][0],2);
    result[2] = Round(tranformVector.m[2][0],2);
    
}

void q_to_euler(float q[4], float euler[3]) {
    euler[0] = atan2(2 * (q[2] * q[3] + q[0] * q[1]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    euler[1] = asin(2 * (q[1] * q[3] - q[0] * q[2]));
    euler[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}

void attitude_control(float x,float y,float z,float roll,float pitch,float yaw) {
    float orientation[3] = {roll, pitch, yaw};
    float position[3] = {x, y, z};
    


    
    float L = 141.0;  
    float W = 133.5;  
    
    float b = 80.0;  
    float Xdist = L;
    float height = 120.0;  

    float bodytoFR0[3] = {L / 2, -b / 2, 0};
    float bodytoFL0[3] = {L / 2, b / 2, 0};
    float bodytoBR0[3] = {-L / 2, -b / 2, 0};
    float bodytoBL0[3] = {-L / 2, b / 2, 0};

    float bodytoFR4[3] = {Xdist / 2, -W / 2, -height};
    float bodytoFL4[3] = {Xdist / 2, W / 2, -height};
    float bodytoBR4[3] = {-Xdist / 2, -W / 2, -height};
    float bodytoBL4[3] = {-Xdist / 2, W / 2, -height};
    

        float _bodytoFR0[3], _bodytoFL0[3], _bodytoBR0[3], _bodytoBL0[3];
        transform(bodytoFR0, orientation, position, _bodytoFR0);
        transform(bodytoFL0, orientation, position, _bodytoFL0);
        transform(bodytoBR0, orientation, position, _bodytoBR0);
        transform(bodytoBL0, orientation, position, _bodytoBL0);
        

        float FRcoord[3], FLcoord[3], BRcoord[3], BLcoord[3];
        FRcoord[0] = bodytoFR4[0] - _bodytoFR0[0];
        FRcoord[1] = bodytoFR4[1] - _bodytoFR0[1];
        FRcoord[2] = bodytoFR4[2] - _bodytoFR0[2];
        
        FLcoord[0] = bodytoFL4[0] - _bodytoFL0[0];
        FLcoord[1] = bodytoFL4[1] - _bodytoFL0[1];
        FLcoord[2] = bodytoFL4[2] - _bodytoFL0[2];
        
        BRcoord[0] = bodytoBR4[0] - _bodytoBR0[0];
        BRcoord[1] = bodytoBR4[1] - _bodytoBR0[1];
        BRcoord[2] = bodytoBR4[2] - _bodytoBR0[2];
        
        BLcoord[0] = bodytoBL4[0] - _bodytoBL0[0];
        BLcoord[1] = bodytoBL4[1] - _bodytoBL0[1];
        BLcoord[2] = bodytoBL4[2] - _bodytoBL0[2];

        float _FRcoord[3], _FLcoord[3], _BRcoord[3], _BLcoord[3];
        transform(FRcoord, orientation, position, _FRcoord);
        transform(FLcoord, orientation, position, _FLcoord);
        transform(BRcoord, orientation, position, _BRcoord);
        transform(BLcoord, orientation, position, _BLcoord);
        
            IK_RUMove(FRcoord[0],FRcoord[1],FRcoord[2],0);
            IK_LUMove(FLcoord[0],FLcoord[1],FLcoord[2],0);
            IK_RBMove(BRcoord[0],BRcoord[1],BRcoord[2],0);
            IK_LBMove(BLcoord[0],BLcoord[1],BLcoord[2],0);


    
}






//姿态动作组
//yaw扭动
void x_play(int time)
{
  for(int i=0;i>-30;i--)
  {
    attitude_control(i,0,0,0,0,0);
    delay(time);

  }
    for(int i=-30;i<30;i++)
  {
    attitude_control(i,0,0,0,0,0);
    delay(time);

  }
    for(int i=30;i>0;i--)
  {
    attitude_control(i,0,0,0,0,0);
    delay(time);

  }
  attitude_control(0,0,0,0,0,0);

}

void y_play(int time)
{
  for(int i=0;i>-30;i--)
  {
    attitude_control(0,i,0,0,0,0);
    delay(time);

  }
    for(int i=-30;i<30;i++)
  {
    attitude_control(0,i,0,0,0,0);
    delay(time);

  }
    for(int i=30;i>0;i--)
  {
    attitude_control(0,i,0,0,0,0);
    delay(time);

  }
  attitude_control(0,0,0,0,0,0);

}

void z_play(int time)
{
  for(int i=0;i>-30;i--)
  {
    attitude_control(0,0,i,0,0,0);
    delay(time);

  }
    for(int i=-30;i<30;i++)
  {
    attitude_control(0,0,i,0,0,0);
    delay(time);

  }
    for(int i=30;i>0;i--)
  {
    attitude_control(0,0,i,0,0,0);
    delay(time);

  }
  attitude_control(0,0,0,0,0,0);

}
void yaw_play(int time)
{
  for(int i=0;i>-30;i--)
  {
    attitude_control(0,0,0,0,0,i);
    delay(time);

  }
    for(int i=-30;i<30;i++)
  {
    attitude_control(0,0,0,0,0,i);
    delay(time);

  }
    for(int i=30;i>0;i--)
  {
    attitude_control(0,0,0,0,0,i);
    delay(time);

  }
  attitude_control(0,0,0,0,0,0);

}
//pitch扭动
void pitch_play(int time)
{
    for(int i=0;i>-20;i--)
  {
    attitude_control(0,0,0,0,i,0);
    delay(time);

  }
    for(int i=-20;i<20;i++)
  {
    attitude_control(0,0,0,0,i,0);
    delay(time);

  }
    for(int i=20;i>0;i--)
  {
    attitude_control(0,0,0,0,i,0);
    delay(time);

  }
  attitude_control(0,0,0,0,0,0);

}
//roll扭动
void roll_play(int time)
{
      for(int i=0;i>-30;i--)
  {
    attitude_control(0,0,0,i,0,0);
    delay(time);

  }
    for(int i=-30;i<30;i++)
  {
    attitude_control(0,0,0,i,0,0);
    delay(time);

  }
    for(int i=30;i>0;i--)
  {
    attitude_control(0,0,0,i,0,0);
    delay(time);

  }
  attitude_control(0,0,0,0,0,0);

}


void attitude_play()
{
    x_play(10);
  delay(1000);
  y_play(10);
  delay(1000);
  z_play(10);
  delay(1000);
  roll_play(10);
  delay(1000);
  pitch_play(10);
  delay(1000);
  yaw_play(10);
  delay(1000);
}



//第三方状态控制
//yaw扭动


void x_control(int angle)
{
  attitude_x=angle;
  attitude_control(attitude_x,attitude_y,attitude_z,attitude_roll,attitude_pitch,attitude_yaw);
}
//pitch扭动
void y_control(int angle)
{
  attitude_y=angle;
  attitude_control(attitude_x,attitude_y,attitude_z,attitude_roll,attitude_pitch,attitude_yaw);
}
//roll扭动
void z_control(int angle)
{
  attitude_z=angle;
  attitude_control(attitude_x,attitude_y,attitude_z,attitude_roll,attitude_pitch,attitude_yaw);
}



void yaw_control(int angle)
{
  attitude_yaw=angle;
  attitude_control(attitude_x,attitude_y,attitude_z,attitude_roll,attitude_pitch,attitude_yaw);
}
//pitch扭动
void pitch_control(int angle)
{
  attitude_pitch=angle;
  attitude_control(attitude_x,attitude_y,attitude_z,attitude_roll,attitude_pitch,attitude_yaw);
}
//roll扭动
void roll_control(int angle)
{
  attitude_roll=angle;
  attitude_control(attitude_x,attitude_y,attitude_z,attitude_roll,attitude_pitch,attitude_yaw);
}




















