#ifndef FKIK_H__
#define FKIK_H__

#include <math.h>

#define L0 24.6  //肩
#define L1 75.0    //大腿
#define L2 72.0    //小腿

#define DEG 57.295779513082  
#define RAD 0.0174532925199  

#define PI 3.141592653589793238
float Round(float in, int num);  制

void FK(float j1, float j2, float j3, float point[]);    
void IK(float x, float y, float z, float Angle[]);       
void IK_LEFT(float x, float y, float z, float Angle[]);  

#endif
