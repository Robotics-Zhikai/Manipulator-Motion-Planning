#ifndef FIKINEMATIC_H
#define FIKINEMATIC_H

#include "main.h"

const double ZERO = 1e-6;
#define M_PI 3.141592653589793
#define M_PI_2 1.570796326794897

//position1Ϊ������ת���ĵ����꣬position2Ϊ������ĩ�˵�����
void ForwardKinematics(const double *jointangle, double *position1, double *position2);
void ForwardKinematics(const double *jointangle, Matrix4d & position1, Matrix4d & position2);

int InverseKinematics(const double position[16], double *jointAngle);
Vector4d InverseKinematics(Matrix4d position);
//����Ϊ���߼����α任���� T50 ���Ϊ�ĸ��Ƕ� ����

//����ΪP40 ���Ϊǰ�����ǵĽǶ�
Vector3d InverseKinematicsPos2Angle(Vector3d position);
#endif
