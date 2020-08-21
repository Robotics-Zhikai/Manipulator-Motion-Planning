#ifndef FIKINEMATIC_H
#define FIKINEMATIC_H

#include "main.h"

const double ZERO = 1e-6;
#define M_PI 3.141592653589793
#define M_PI_2 1.570796326794897

//position1为铲斗旋转中心点坐标，position2为铲齿最末端点坐标
void ForwardKinematics(double *jointangle, double *position1, double *position2);

//int InverseKinematics(double position[16], double *jointAngle);
Vector4d InverseKinematics(Matrix4d position);
//输入为铲尺尖的齐次变换矩阵 T50 输出为四个角度 度数

int InverseKinematicsPos2Angle(double position[3], double * jointAngle);

#endif
