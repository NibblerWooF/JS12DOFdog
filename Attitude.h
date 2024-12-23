#ifndef ATTITUDE_H__
#define ATTITUDE_H__

#include <math.h>

#define PI 3.14159265358979323846


typedef struct {
    float m[4][4];
} Matrix4x4;


Matrix4x4 Rx(float roll);
Matrix4x4 Ry(float pitch);
Matrix4x4 Rz(float yaw);
Matrix4x4 multiply(Matrix4x4 a, Matrix4x4 b);
Matrix4x4 Rxyz(float roll, float pitch, float yaw);
Matrix4x4 RTmatrix(float orientation[3], float position[3]);
void transform(float coord[3], float rotation[3], float translation[3], float result[3]);
void q_to_euler(float q[4], float euler[3]);
void attitude_control(float x,float y,float z,float roll,float pitch,float yaw);
void IMU_Attitude_Control(void);


void x_play(int time);
void y_play(int time);
void z_play(int time);

void yaw_play(int time);
void pitch_play(int time);
void roll_play(int time);

void attitude_play();


void x_control(int angle);
void y_control(int angle);
void z_control(int angle);

void yaw_control(int angle);
void pitch_control(int angle);
void roll_control(int angle);





#endif