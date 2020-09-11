#ifndef FIKINEMATIC_H
#define FIKINEMATIC_H

#include "main.h"
#include "EigenMatrixOperate.h"

const double ZERO = 1e-6;
#define M_PI 3.141592653589793
#define M_PI_2 1.570796326794897

int IsAnglesInLimitRange(Vector4d jointAngles);

//position1为铲斗旋转中心点坐标，position2为铲齿最末端点坐标
void ForwardKinematics(const double *jointangle, double *position1, double *position2);
void ForwardKinematics(const Vector4d jointangle, Matrix4d & position1, Matrix4d & position2);

int InverseKinematics(const double position[16], double *jointAngle);
Vector4d InverseKinematics(Matrix4d position);
//输入为铲尺尖的齐次变换矩阵 T50 输出为四个角度 度数
Vector4d InverseKinematics(Matrix4d position, int & Valid);//Valid表示此时的结果是否可信 可信为1 否则为0

Vector4d InverseKinematicsT40(Matrix4d position, int & Valid); //%输入是铲斗旋转中心的转换矩阵 T40

//输入为P40 输出为前三个角的角度
Vector3d InverseKinematicsPos2Angle(Vector3d position);
Vector3d InverseKinematicsPos2Angle(Vector3d position, int & reliable);

double GetAngleOfBucketWithGround(double theta1, double theta2, double theta3, double theta4);
vector <MatrixXd> groundAngleRangeTOtheta4Range(double theta1, double  theta2, double  theta3, MatrixXd WithGroundAngleRange, int & YES);

#endif
