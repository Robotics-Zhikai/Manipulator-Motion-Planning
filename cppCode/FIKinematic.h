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

//position1Ϊ������ת���ĵ����꣬position2Ϊ������ĩ�˵�����
void ForwardKinematics(double *jointangle, double *position1, double *position2);

int InverseKinematics(double position[16], double *jointAngle);

int InverseKinematicsPos2Angle(double position[3], double * jointAngle);

#endif
