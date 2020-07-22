#ifndef FIKINEMATIC_H
#define FIKINEMATIC_H
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

const double ZERO = 1e-6;
#define M_PI 3.14159 
#define M_PI_2 1.5708

//position1为铲斗旋转中心点坐标，position2为铲齿最末端点坐标
void ForwardKinematics(double *jointangle, double *position1, double *position2);

int InverseKinematics(double position[16], double *jointAngle);

int InverseKinematicsPos2Angle(double position[3], double * jointAngle);

#endif
