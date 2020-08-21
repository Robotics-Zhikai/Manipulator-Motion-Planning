#ifndef FIKINEMATIC_H
#define FIKINEMATIC_H

#include "main.h"

const double ZERO = 1e-6;
#define M_PI 3.141592653589793
#define M_PI_2 1.570796326794897

//position1Ϊ������ת���ĵ����꣬position2Ϊ������ĩ�˵�����
void ForwardKinematics(double *jointangle, double *position1, double *position2);

//int InverseKinematics(double position[16], double *jointAngle);
Vector4d InverseKinematics(Matrix4d position);
//����Ϊ���߼����α任���� T50 ���Ϊ�ĸ��Ƕ� ����

int InverseKinematicsPos2Angle(double position[3], double * jointAngle);

#endif
