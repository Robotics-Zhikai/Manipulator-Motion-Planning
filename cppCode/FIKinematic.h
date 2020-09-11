#ifndef FIKINEMATIC_H
#define FIKINEMATIC_H

#include "main.h"
#include "EigenMatrixOperate.h"

const double ZERO = 1e-6;
#define M_PI 3.141592653589793
#define M_PI_2 1.570796326794897

int IsAnglesInLimitRange(Vector4d jointAngles);

//position1Ϊ������ת���ĵ����꣬position2Ϊ������ĩ�˵�����
void ForwardKinematics(const double *jointangle, double *position1, double *position2);
void ForwardKinematics(const Vector4d jointangle, Matrix4d & position1, Matrix4d & position2);

int InverseKinematics(const double position[16], double *jointAngle);
Vector4d InverseKinematics(Matrix4d position);
//����Ϊ���߼����α任���� T50 ���Ϊ�ĸ��Ƕ� ����
Vector4d InverseKinematics(Matrix4d position, int & Valid);//Valid��ʾ��ʱ�Ľ���Ƿ���� ����Ϊ1 ����Ϊ0

Vector4d InverseKinematicsT40(Matrix4d position, int & Valid); //%�����ǲ�����ת���ĵ�ת������ T40

//����ΪP40 ���Ϊǰ�����ǵĽǶ�
Vector3d InverseKinematicsPos2Angle(Vector3d position);
Vector3d InverseKinematicsPos2Angle(Vector3d position, int & reliable);

double GetAngleOfBucketWithGround(double theta1, double theta2, double theta3, double theta4);
vector <MatrixXd> groundAngleRangeTOtheta4Range(double theta1, double  theta2, double  theta3, MatrixXd WithGroundAngleRange, int & YES);

#endif
