#include "FKIK.h"

//四舍五入+控制小数点
float Round(float in,int num){
    return round(in*pow(10,num))/pow(10,num);
}

//运动学正解函数
void FK(float j0,float j1,float j2,float point[]){

    float x,y,z;                

    j0 = j0*RAD;               
    j1 = j1*RAD;
    j2 = j2*RAD;

    float t = 90*RAD + j2 - j1;
    float l = L1 * cos(j1) + L2 * sin(t);

    
    x =   L1*sin(j1) + L2 * cos(t);
    y =    l*sin(j0) + L0 * cos(j0);
    z =    l*cos(j0) - L0 * sin(j0);


    point[0] = Round(x,3);
    point[1] = Round(y,3);
    point[2] = Round(z,3);
}

//运动学逆解
//jo  肩部    j1  大腿   j2  小腿
//L0  肩部    L1  大腿   L2  小腿
void IK(float x,float y,float z,float Angle[]) {
    float j0, j1, j2;                   
    float link = 1.0;                  


    float l = sqrt(y * y + z * z - L0 * L0);

    j0 = -atan(y/z) + atan(L0/l);


    // 小腿
    float s = sqrt(l * l + x * x);
    float n = (s * s - L1 * L1 - L2 * L2) / (2*L1);

    j2 = -acos(n / L2);


    // 大腿
    j1 = acos((L1 + n) / s) - atan(x/l)  ;  

    //角度结果
    Angle[0] = Round(j0 * DEG, 2);
    Angle[1] = Round(j1 * DEG, 2);
    Angle[2] = Round(j2 * DEG*link, 2);
    Angle[2] = 180-Round(j2 * DEG, 2);
}

void IK_LEFT(float x,float y,float z,float Angle[]) {
    float j0, j1, j2;                   
    float link = 1.4;                  

    //逆运动公式
    float l = sqrt(y * y + z * z - L0 * L0);

  
        j0 = atan(y/z) + atan(L0/l);


    //小腿
    float s = sqrt(l * l + x * x);
    float n = (s * s - L1 * L1 - L2 * L2) / (2*L1);

    j2 = -acos(n / L2);


    //大腿
    j1 = acos((L1 + n) / s) - atan(x/l)  ;  
    Angle[0] = Round(j0 * DEG, 2);
    Angle[1] = Round(j1 * DEG, 2);
    Angle[2] = Round(j2 * DEG*link, 2);
    Angle[2] = 180-Round(j2 * DEG, 2);
}



