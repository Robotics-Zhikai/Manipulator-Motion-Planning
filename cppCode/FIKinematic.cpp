#include "FIKinematic.h"

int IsAnglesInLimitRange(Vector4d jointAngles)
{
	if ((jointAngles(0) < theta1Range(0) || jointAngles(0) > theta1Range(1)) || (jointAngles(1) < theta2Range(0) || jointAngles(1) > theta2Range(1)) \
		|| (jointAngles(2) < theta3Range(0) || jointAngles(2) > theta3Range(1)) || (jointAngles(3) < theta4Range(0) || jointAngles(3) > theta4Range(1)))
		return 0;
	else
		return 1;
}

double mathAtan2(double y, double x)
{
	if (fabs(y) < ZERO)
		y = 0.0;
	if (fabs(x) < ZERO)
		x = 0.0;

	return atan2(y, x);

}

void mathMatxMult(const double *matrix1, const double *matrix2, double *matrix3, int row, int xxx, int col)
{
	int ii, jj, kk, l;

	for (ii = 0; ii < row; ii++)
	{
		for (jj = 0; jj < col; jj++)
		{
			l = ii * col + jj;
			matrix3[l] = 0.0;
			for (kk = 0; kk < xxx; kk++)
				matrix3[l] += matrix1[ii*xxx + kk] * matrix2[kk*col + jj];
		}
	}
}

void MatrixT(double &alpha, double &a, double &theta, double &d, double TransMatrix[16])
{
	TransMatrix[0] = cos(theta);
	TransMatrix[1] = -sin(theta);
	TransMatrix[2] = 0.0;
	TransMatrix[3] = a;

	TransMatrix[4] = sin(theta)*cos(alpha);
	TransMatrix[5] = cos(theta)*cos(alpha);
	TransMatrix[6] = -sin(alpha);
	TransMatrix[7] = -sin(alpha)*d;

	TransMatrix[8] = sin(theta)*sin(alpha);
	TransMatrix[9] = cos(theta)*sin(alpha);
	TransMatrix[10] = cos(alpha);
	TransMatrix[11] = cos(alpha)*d;

	TransMatrix[12] = 0.0;
	TransMatrix[13] = 0.0;
	TransMatrix[14] = 0.0;
	TransMatrix[15] = 1.0;
}

void MatrixTool(double &tool, double matrixtool[16])
{
	matrixtool[0] = 1.0;
	matrixtool[1] = 0.0;
	matrixtool[2] = 0.0;
	matrixtool[3] = tool;

	matrixtool[4] = 0.0;
	matrixtool[5] = 1.0;
	matrixtool[6] = 0.0;
	matrixtool[7] = 0.0;

	matrixtool[8] = 0.0;
	matrixtool[9] = 0.0;
	matrixtool[10] = 1.0;
	matrixtool[11] = 0.0;

	matrixtool[12] = 0.0;
	matrixtool[13] = 0.0;
	matrixtool[14] = 0.0;
	matrixtool[15] = 1.0;
}


void ForwardKinematics(const double *jointangle, double *position1, double *position2)
{
	double m[4], a[4], theta[4], d[4];
	double m_matrix1[16], m_matrix2[16], m_matrix3[16], m_matrix4[16], m_matrix5[16];
	double m_matrixtemp1[16], m_matrixtemp2[16], m_matrixtemp3[16], m_matrixtemp4[16];
	double m_dRPY[3];

	//m[0] = 0.0*M_PI / 180.0;
	//m[1] = 90.0*M_PI / 180.0;
	//m[2] = 0.0*M_PI / 180.0;
	//m[3] = 0.0*M_PI / 180.0;

	//a[0] = 0.0;
	//a[1] = 12.0;
	//a[2] = 460.0;
	//a[3] = 210.9;

	//d[0] = 57.9;
	//d[1] = 13.7;//13.7
	//d[2] = 0.0;
	//d[3] = 0.0;
	//tool = 123.5;

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;

	theta[0] = jointangle[0] * M_PI / 180.0;
	theta[1] = jointangle[1] * M_PI / 180.0;
	theta[2] = jointangle[2] * M_PI / 180.0;
	theta[3] = jointangle[3] * M_PI / 180.0;

	MatrixT(m[0], a[0], theta[0], d[0], m_matrix1);
	MatrixT(m[1], a[1], theta[1], d[1], m_matrix2);
	MatrixT(m[2], a[2], theta[2], d[2], m_matrix3);
	MatrixT(m[3], a[3], theta[3], d[3], m_matrix4);
	MatrixTool(tool, m_matrix5);

	mathMatxMult(m_matrix1, m_matrix2, m_matrixtemp1, 4, 4, 4);
	mathMatxMult(m_matrixtemp1, m_matrix3, m_matrixtemp2, 4, 4, 4);
	mathMatxMult(m_matrixtemp2, m_matrix4, m_matrixtemp3, 4, 4, 4);
	mathMatxMult(m_matrixtemp3, m_matrix5, m_matrixtemp4, 4, 4, 4);

	//欧拉角A、B、C begin
	m_dRPY[1] = mathAtan2(-m_matrixtemp4[8], sqrt(m_matrixtemp4[0] * m_matrixtemp4[0] + m_matrixtemp4[4] * m_matrixtemp4[4]));//欧拉角B

	if ((m_dRPY[1]<M_PI_2 + ZERO) && (m_dRPY[1]>M_PI_2 - ZERO))
	{
		m_dRPY[2] = mathAtan2(m_matrixtemp4[1], m_matrixtemp4[5]);
		m_dRPY[0] = 0.0;
	}
	else if ((m_dRPY[1]<-M_PI_2 + ZERO) && (m_dRPY[1]>-M_PI_2 - ZERO))
	{
		m_dRPY[2] = -mathAtan2(m_matrixtemp4[1], m_matrixtemp4[5]);//欧拉角A
		m_dRPY[0] = 0.0;//欧拉角C
	}
	else
	{
		m_dRPY[2] = mathAtan2(m_matrixtemp4[4] / cos(m_dRPY[1]), m_matrixtemp4[0] / cos(m_dRPY[1]));//欧拉角A
		m_dRPY[0] = mathAtan2(m_matrixtemp4[9] / cos(m_dRPY[1]), m_matrixtemp4[10] / cos(m_dRPY[1]));//欧拉角C
	}
	//欧拉角A、B、C end

	//printf("\nEuler%f %f %f\n", m_dRPY[0] * 180.0 / M_PI, m_dRPY[1] * 180.0 / M_PI, m_dRPY[2] * 180.0 / M_PI);

	for (int i = 0; i < 16; i++)
	{
		position1[i] = m_matrixtemp3[i];//铲斗旋转中心坐标
		position2[i] = m_matrixtemp4[i];//末端铲齿坐标
	}
}
//20200824 转化这个函数


//position1为铲斗旋转中心点坐标，position2为铲齿最末端点坐标
void ForwardKinematics(const Vector4d jointangle, Matrix4d & position1, Matrix4d & position2)
{
	double m[4], a[4], theta[4], d[4];
	double m_matrix1[16], m_matrix2[16], m_matrix3[16], m_matrix4[16], m_matrix5[16];
	double m_matrixtemp1[16], m_matrixtemp2[16], m_matrixtemp3[16], m_matrixtemp4[16];
	double m_dRPY[3];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;

	theta[0] = jointangle[0] * M_PI / 180.0;
	theta[1] = jointangle[1] * M_PI / 180.0;
	theta[2] = jointangle[2] * M_PI / 180.0;
	theta[3] = jointangle[3] * M_PI / 180.0;

	MatrixT(m[0], a[0], theta[0], d[0], m_matrix1);
	MatrixT(m[1], a[1], theta[1], d[1], m_matrix2);
	MatrixT(m[2], a[2], theta[2], d[2], m_matrix3);
	MatrixT(m[3], a[3], theta[3], d[3], m_matrix4);
	MatrixTool(tool, m_matrix5);

	mathMatxMult(m_matrix1, m_matrix2, m_matrixtemp1, 4, 4, 4);
	mathMatxMult(m_matrixtemp1, m_matrix3, m_matrixtemp2, 4, 4, 4);
	mathMatxMult(m_matrixtemp2, m_matrix4, m_matrixtemp3, 4, 4, 4);
	mathMatxMult(m_matrixtemp3, m_matrix5, m_matrixtemp4, 4, 4, 4);


	////欧拉角A、B、C begin
	//m_dRPY[1] = mathAtan2(-m_matrixtemp4[8], sqrt(m_matrixtemp4[0] * m_matrixtemp4[0] + m_matrixtemp4[4] * m_matrixtemp4[4]));//欧拉角B

	//if ((m_dRPY[1]<M_PI_2 + ZERO) && (m_dRPY[1]>M_PI_2 - ZERO))
	//{
	//	m_dRPY[2] = mathAtan2(m_matrixtemp4[1], m_matrixtemp4[5]);
	//	m_dRPY[0] = 0.0;
	//}
	//else if ((m_dRPY[1]<-M_PI_2 + ZERO) && (m_dRPY[1]>-M_PI_2 - ZERO))
	//{
	//	m_dRPY[2] = -mathAtan2(m_matrixtemp4[1], m_matrixtemp4[5]);//欧拉角A
	//	m_dRPY[0] = 0.0;//欧拉角C
	//}
	//else
	//{
	//	m_dRPY[2] = mathAtan2(m_matrixtemp4[4] / cos(m_dRPY[1]), m_matrixtemp4[0] / cos(m_dRPY[1]));//欧拉角A
	//	m_dRPY[0] = mathAtan2(m_matrixtemp4[9] / cos(m_dRPY[1]), m_matrixtemp4[10] / cos(m_dRPY[1]));//欧拉角C
	//}
	////欧拉角A、B、C end

	//printf("\nEuler%f %f %f\n", m_dRPY[0] * 180.0 / M_PI, m_dRPY[1] * 180.0 / M_PI, m_dRPY[2] * 180.0 / M_PI);

	int k_count = 0;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			position1(i, j) = m_matrixtemp3[k_count];
			position2(i, j) = m_matrixtemp4[k_count];
			k_count++;
		}
	}
	//for (int i = 0; i < 16; i++)
	//{
	//	position1[i] = m_matrixtemp3[i];//铲斗旋转中心坐标
	//	position2[i] = m_matrixtemp4[i];//末端铲齿坐标
	//}
}

//输入为铲尺尖的齐次变换矩阵 T50 输出为四个角度 度数
int InverseKinematics(const double position[16], double *jointAngle)
{
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4];

	double mTempAngleOne[2];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;

	double transForm[16] = { 1,0,0,-tool,0,1,0,0,0,0,1,0,0,0,0,1 };
	double positionout[16];

	mathMatxMult(position, transForm, positionout, 4, 4, 4);


	nx = positionout[0];
	ny = positionout[4];
	nz = positionout[8];

	ox = positionout[1];
	oy = positionout[5];
	oz = positionout[9];

	ax = positionout[2];
	ay = positionout[6];
	az = positionout[10];

	px = positionout[3];
	py = positionout[7];
	pz = positionout[11];

	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle[0] = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

	if (abs(Mtemp1)>1)
		error("似乎超出了工作空间，结果不可信");

	//    printf("\nMtemp1=%f\n",Mtemp1);

	jointAngle[2] = -fabs(acos(Mtemp1));
	//    printf("\njointAngle[2]=%f\n",jointAngle[2]*180.0/M_PI);


	double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle[0]) + py*sin(jointAngle[0]) - a[1];
	tempn = pz - d[0];
	if (fabs(tempm*tempm + tempn*tempn) <= ZERO)
	{
		printf("error!");
		return 1;
	}
	else
	{
		tempTwo1 = ((a[2] + a[3] * cos(jointAngle[2]))*tempn - a[3] * sin(jointAngle[2])*tempm) / (tempm*tempm + tempn*tempn);
	}
	if (abs(tempTwo1)>1)
		error("似乎超出了工作空间，结果不可信");

	jointAngle[1] = asin(tempTwo1);

	double tempfour1, tempfour2;
	tempfour1 = -(cos(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*ox + sin(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*oy +
		sin(jointAngle[1] + jointAngle[2])*oz);
	tempfour2 = -cos(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*ox - sin(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*oy +
		cos(jointAngle[1] + jointAngle[2])*oz;
	jointAngle[3] = mathAtan2(tempfour1, tempfour2);

	for (int i = 0; i < 4; i++)
	{
		jointAngle[i] *= 180.0 / M_PI;
	}

	return 0;
}

//输入为铲尺尖的齐次变换矩阵 T50 输出为四个角度 度数
Vector4d InverseKinematics(Matrix4d position)
{
	Vector4d jointAngle = Vector4d::Zero();
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4];

	double mTempAngleOne[2];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;

	Matrix4d transForm;
	transForm.row(0) << 1, 0, 0, -tool;
	transForm.row(1) << 0, 1, 0, 0;
	transForm.row(2) << 0, 0, 1, 0;
	transForm.row(3) << 0, 0, 0, 1;
	position = position*transForm;

	nx = position(0, 0);
	ny = position(1, 0);
	nz = position(2, 0);

	ox = position(0, 1);
	oy = position(1, 1);
	oz = position(2, 1);

	ax = position(0, 2);
	ay = position(1, 2);
	az = position(2, 2);

	px = position(0, 3);
	py = position(1, 3);
	pz = position(2, 3);

	

	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle(0) = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

	if (abs(Mtemp1)>1)
		error("似乎超出了工作空间，结果不可信");



	//    printf("\nMtemp1=%f\n",Mtemp1);

	jointAngle(2) = -fabs(acos(Mtemp1));
	//    printf("\njointAngle[2]=%f\n",jointAngle[2]*180.0/M_PI);


	double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle[0]) + py*sin(jointAngle[0]) - a[1];
	tempn = pz - d[0];
	if (fabs(tempm*tempm + tempn*tempn) <= ZERO)
	{
		error("没有逆解，出现错误");
		//printf("error!");
		//system("pause");
		//return 1;
	}
	else
	{
		tempTwo1 = ((a[2] + a[3] * cos(jointAngle[2]))*tempn - a[3] * sin(jointAngle[2])*tempm) / (tempm*tempm + tempn*tempn);
	}
	if (abs(tempTwo1)>1)
		error("似乎超出了工作空间，结果不可信");
	
	jointAngle[1] = asin(tempTwo1);

	double tempfour1, tempfour2;
	tempfour1 = -(cos(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*ox + sin(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*oy +
		sin(jointAngle[1] + jointAngle[2])*oz);
	tempfour2 = -cos(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*ox - sin(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*oy +
		cos(jointAngle[1] + jointAngle[2])*oz;
	jointAngle[3] = mathAtan2(tempfour1, tempfour2);

	for (int i = 0; i < 4; i++)
	{
		jointAngle[i] *= 180.0 / M_PI;
	}
	return jointAngle;
	//return 0;
}

Vector4d InverseKinematics(Matrix4d position,int & Valid)
{
	Valid = 1;
	Vector4d jointAngle = Vector4d::Zero();
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4];

	double mTempAngleOne[2];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;

	Matrix4d transForm;
	transForm.row(0) << 1, 0, 0, -tool;
	transForm.row(1) << 0, 1, 0, 0;
	transForm.row(2) << 0, 0, 1, 0;
	transForm.row(3) << 0, 0, 0, 1;
	position = position*transForm;

	nx = position(0, 0);
	ny = position(1, 0);
	nz = position(2, 0);

	ox = position(0, 1);
	oy = position(1, 1);
	oz = position(2, 1);

	ax = position(0, 2);
	ay = position(1, 2);
	az = position(2, 2);

	px = position(0, 3);
	py = position(1, 3);
	pz = position(2, 3);



	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle(0) = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

	if (abs(Mtemp1) > 1)
	{
		Valid = 0;
		jointAngle = Vector4d::Zero();
		return jointAngle;
		error("似乎超出了工作空间，结果不可信");
	}
		



	//    printf("\nMtemp1=%f\n",Mtemp1);

	jointAngle(2) = -fabs(acos(Mtemp1));
	//    printf("\njointAngle[2]=%f\n",jointAngle[2]*180.0/M_PI);


	double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle[0]) + py*sin(jointAngle[0]) - a[1];
	tempn = pz - d[0];
	if (fabs(tempm*tempm + tempn*tempn) <= ZERO)
	{
		Valid = 0;
		jointAngle = Vector4d::Zero();
		return jointAngle;
		error("没有逆解，出现错误");
		//printf("error!");
		//system("pause");
		//return 1;
	}
	else
	{
		tempTwo1 = ((a[2] + a[3] * cos(jointAngle[2]))*tempn - a[3] * sin(jointAngle[2])*tempm) / (tempm*tempm + tempn*tempn);
	}
	if (abs(tempTwo1) > 1)
	{
		Valid = 0;
		jointAngle = Vector4d::Zero();
		return jointAngle;
		error("似乎超出了工作空间，结果不可信");
	}
		

	jointAngle[1] = asin(tempTwo1);

	double tempfour1, tempfour2;
	tempfour1 = -(cos(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*ox + sin(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*oy +
		sin(jointAngle[1] + jointAngle[2])*oz);
	tempfour2 = -cos(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*ox - sin(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*oy +
		cos(jointAngle[1] + jointAngle[2])*oz;
	jointAngle[3] = mathAtan2(tempfour1, tempfour2);

	for (int i = 0; i < 4; i++)
	{
		jointAngle[i] *= 180.0 / M_PI;
	}

	if (Valid == 1)
	{
		if (IsAnglesInLimitRange(jointAngle) == 0)
		{
			Valid = 0;
			jointAngle = Vector4d::Zero();
		}
	}
	return jointAngle;
	//return 0;
	
}

Vector4d InverseKinematicsT40(Matrix4d position, int & Valid)
{
	Valid = 1;
	Vector4d jointAngle = Vector4d::Zero();
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4];

	double mTempAngleOne[2];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;

	//Matrix4d transForm;
	//transForm.row(0) << 1, 0, 0, -tool;
	//transForm.row(1) << 0, 1, 0, 0;
	//transForm.row(2) << 0, 0, 1, 0;
	//transForm.row(3) << 0, 0, 0, 1;
	//position = position*transForm;

	nx = position(0, 0);
	ny = position(1, 0);
	nz = position(2, 0);

	ox = position(0, 1);
	oy = position(1, 1);
	oz = position(2, 1);

	ax = position(0, 2);
	ay = position(1, 2);
	az = position(2, 2);

	px = position(0, 3);
	py = position(1, 3);
	pz = position(2, 3);



	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle(0) = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

	if (abs(Mtemp1) > 1)
	{
		Valid = 0;
		jointAngle = Vector4d::Zero();
		return jointAngle;
		error("似乎超出了工作空间，结果不可信");
	}




	//    printf("\nMtemp1=%f\n",Mtemp1);

	jointAngle(2) = -fabs(acos(Mtemp1));
	//    printf("\njointAngle[2]=%f\n",jointAngle[2]*180.0/M_PI);


	double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle[0]) + py*sin(jointAngle[0]) - a[1];
	tempn = pz - d[0];
	if (fabs(tempm*tempm + tempn*tempn) <= ZERO)
	{
		Valid = 0;
		jointAngle = Vector4d::Zero();
		return jointAngle;
		error("没有逆解，出现错误");
		//printf("error!");
		//system("pause");
		//return 1;
	}
	else
	{
		tempTwo1 = ((a[2] + a[3] * cos(jointAngle[2]))*tempn - a[3] * sin(jointAngle[2])*tempm) / (tempm*tempm + tempn*tempn);
	}
	if (abs(tempTwo1) > 1)
	{
		Valid = 0;
		jointAngle = Vector4d::Zero();
		return jointAngle;
		error("似乎超出了工作空间，结果不可信");
	}


	jointAngle[1] = asin(tempTwo1);

	double tempfour1, tempfour2;
	tempfour1 = -(cos(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*ox + sin(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*oy +
		sin(jointAngle[1] + jointAngle[2])*oz);
	tempfour2 = -cos(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*ox - sin(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*oy +
		cos(jointAngle[1] + jointAngle[2])*oz;
	jointAngle[3] = mathAtan2(tempfour1, tempfour2);

	for (int i = 0; i < 4; i++)
	{
		jointAngle[i] *= 180.0 / M_PI;
	}

	if (Valid == 1)
	{
		if (IsAnglesInLimitRange(jointAngle) == 0)
		{
			Valid = 0;
			jointAngle = Vector4d::Zero();
		}
	}
	return jointAngle;
	//return 0;
}

//输入为P40 输出为前三个角的角度
Vector3d InverseKinematicsPos2Angle(Vector3d position)
{
	Vector3d jointAngle = Vector3d::Zero();
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4];

	double mTempAngleOne[2];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;


	px = position[0];
	py = position[1];
	pz = position[2];

	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle[0] = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

	if (abs(Mtemp1)>1)
		error("似乎超出了工作空间，结果不可信");

	//    printf("\nMtemp1=%f\n",Mtemp1);

	jointAngle[2] = -fabs(acos(Mtemp1));
	//    printf("\njointAngle[2]=%f\n",jointAngle[2]*180.0/M_PI);


	double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle[0]) + py*sin(jointAngle[0]) - a[1];
	tempn = pz - d[0];
	if (fabs(tempm*tempm + tempn*tempn) <= ZERO)
	{
		error("error!");
		/*printf("error!");
		return 1;*/
	}
	else
	{
		tempTwo1 = ((a[2] + a[3] * cos(jointAngle[2]))*tempn - a[3] * sin(jointAngle[2])*tempm) / (tempm*tempm + tempn*tempn);
	}
	if (abs(tempTwo1)>1)
		error("似乎超出了工作空间，结果不可信");

	jointAngle[1] = asin(tempTwo1);

	//double tempfour1, tempfour2;
	//tempfour1 = -(cos(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*ox + sin(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*oy +
	//	sin(jointAngle[1] + jointAngle[2])*oz);
	//tempfour2 = -cos(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*ox - sin(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*oy +
	//	cos(jointAngle[1] + jointAngle[2])*oz;
	//jointAngle[3] = mathAtan2(tempfour1, tempfour2);

	for (int i = 0; i < 4; i++)
	{
		jointAngle[i] *= 180.0 / M_PI;
	}

	return jointAngle;
}

Vector3d InverseKinematicsPos2Angle(Vector3d position,int & reliable)
// 输入为铲斗旋转中心位置的列向量，输出为前三个角的角度 为横向量 p40
{
	reliable = 1;
	Vector3d jointAngle = Vector3d::Zero();
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4];

	double mTempAngleOne[2];

	m[0] = m0;
	m[1] = m1;
	m[2] = m2;
	m[3] = m3;

	a[0] = a0;
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	//tool = tool;

	d[0] = d1;
	d[1] = d2;//13.7
	d[2] = d3;
	d[3] = d4;


	px = position[0];
	py = position[1];
	pz = position[2];

	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle[0] = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

	if (abs(Mtemp1) > 1)
	{
		reliable = 0;
		jointAngle = Vector3d::Zero();
		return jointAngle;
		error("似乎超出了工作空间，结果不可信");
	}
		
	//    printf("\nMtemp1=%f\n",Mtemp1);

	jointAngle[2] = -fabs(acos(Mtemp1));
	//    printf("\njointAngle[2]=%f\n",jointAngle[2]*180.0/M_PI);


	double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle[0]) + py*sin(jointAngle[0]) - a[1];
	tempn = pz - d[0];
	if (fabs(tempm*tempm + tempn*tempn) <= ZERO)
	{
		reliable = 0;
		jointAngle = Vector3d::Zero();
		return jointAngle;
		error("error!");
		/*printf("error!");
		return 1;*/
	}
	else
	{
		tempTwo1 = ((a[2] + a[3] * cos(jointAngle[2]))*tempn - a[3] * sin(jointAngle[2])*tempm) / (tempm*tempm + tempn*tempn);
	}
	if (abs(tempTwo1) > 1)
	{
		reliable = 0;
		jointAngle = Vector3d::Zero();
		return jointAngle;
		error("似乎超出了工作空间，结果不可信");
	}
		

	jointAngle[1] = asin(tempTwo1);

	//double tempfour1, tempfour2;
	//tempfour1 = -(cos(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*ox + sin(jointAngle[0])*cos(jointAngle[1] + jointAngle[2])*oy +
	//	sin(jointAngle[1] + jointAngle[2])*oz);
	//tempfour2 = -cos(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*ox - sin(jointAngle[0])*sin(jointAngle[1] + jointAngle[2])*oy +
	//	cos(jointAngle[1] + jointAngle[2])*oz;
	//jointAngle[3] = mathAtan2(tempfour1, tempfour2);

	for (int i = 0; i < 4; i++)
	{
		jointAngle[i] *= 180.0 / M_PI;
	}

	return jointAngle;
}

double GetAngleOfBucketWithGround(double theta1, double theta2, double theta3, double theta4) // 得到在四个角的情况下铲斗与地面的夹角 theta4属于给定的范围
{
	double k1 = theta1*pi / 180;
	double k2 = theta2*pi / 180;
	Matrix4d position1, position2;
	Vector4d Angles;
	Angles(0) = theta1;
	Angles(1) = theta2;
	Angles(2) = theta3;
	Angles(3) = theta4;
	ForwardKinematics(Angles,position1,position2);
	MatrixXd P3minusP2(1, 3);
	P3minusP2(0) = a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1);
	P3minusP2(1) = a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1));
	P3minusP2(2) = a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1));

	Vector3d XPositive;//以这个向量为x的正方向 当铲斗与其呈180度附近时，土不会掉下来
	XPositive(0) = P3minusP2(0);
	XPositive(1) = P3minusP2(1);
	XPositive(2) = 0;
	XPositive.normalize();

	Vector3d BucketVector = position2.block(0, 3, 3, 1) - position1.block(0, 3, 3, 1);
	BucketVector.normalize();
	//对于 X 在区间 [-1, 1] 内的实数值，acos(X) 返回区间 [0, π] 内的值。
	double abstheta4 = acos(BucketVector.dot(XPositive));
	double Angle;
	if (BucketVector(2)<0)
		Angle = -abstheta4;
	else
		Angle = abstheta4;
	Angle = Angle * 180 / pi;
	Angle = legalizAnger(Angle);

	return Angle;
}

Vector2d GetBucketwithGroundRange(double theta1,double theta2,double theta3)
// 得到theta4满足给定的范围的前提下与地面的夹角范围
{
	Vector2d BucketwithGroundRange;
	BucketwithGroundRange(0) = GetAngleOfBucketWithGround(theta1, theta2, theta3, theta4Range(0));
	BucketwithGroundRange(1) = GetAngleOfBucketWithGround(theta1, theta2, theta3, theta4Range(1));
	return BucketwithGroundRange;
}

vector <MatrixXd> groundAngleRangeTOtheta4Range(double theta1, double  theta2, double  theta3, MatrixXd WithGroundAngleRange, int & YES) //WithGroundAngleRange，左为下限，右为上限，逆时针为正，超出180即跳变为负
{
	//YES 为1 时表明在theta4满足给定的范围的条件下能找出一范围来，满足WithGroundAngleRange
	//WithGroundAngleRange的角度体系也是 - 180 180
	WithGroundAngleRange(0) = legalizAnger(WithGroundAngleRange(0));
	WithGroundAngleRange(1) = legalizAnger(WithGroundAngleRange(1));
	vector <MatrixXd>  Theta4Range;
	Vector2d BucketwithGroundRange = GetBucketwithGroundRange(theta1, theta2, theta3);

	vector <MatrixXd> SetBucketwithGroundRange;
	if (BucketwithGroundRange(1) > BucketwithGroundRange(0))
		SetBucketwithGroundRange.push_back(BucketwithGroundRange);
	else
		if (BucketwithGroundRange(1) < BucketwithGroundRange(0))
		{
			double theta4mid = theta4Range(0) + 180 - BucketwithGroundRange(0);
			MatrixXd tmppush(1,2);
			tmppush(0) = BucketwithGroundRange(0);
			tmppush(1) = 180;
			SetBucketwithGroundRange.push_back(tmppush);
			tmppush(0) = theta4Range(0);
			tmppush(1) = theta4mid;
			SetBucketwithGroundRange.push_back(tmppush);
			tmppush(0) = -179.99999;
			tmppush(1) = BucketwithGroundRange(1);
			SetBucketwithGroundRange.push_back(tmppush);
			tmppush(0) = theta4mid;
			tmppush(1) = theta4Range(1);
			SetBucketwithGroundRange.push_back(tmppush);
		}
	
	vector <MatrixXd> SetWithGroundAngleRange;
	if (WithGroundAngleRange(1) >= WithGroundAngleRange(0))
        SetWithGroundAngleRange.push_back( WithGroundAngleRange);
    else
		if (WithGroundAngleRange(1) < WithGroundAngleRange(0))
		{
			MatrixXd tmppush(1, 2);
			tmppush(0) = WithGroundAngleRange(0);
			tmppush(1) = 180;
			SetWithGroundAngleRange.push_back(tmppush);
			tmppush(0) = -179.99999;
			tmppush(1) = WithGroundAngleRange(1);
			SetWithGroundAngleRange.push_back(tmppush);
		}
	    
	vector <MatrixXd> intersectionSet;
	for (int i = 0; i < SetBucketwithGroundRange.size(); i = i + 2)
	{
		for (int j = 0; j < SetWithGroundAngleRange.size(); j = j + 1)
		{
			MatrixXd intersecttmp = GetIntersection(SetBucketwithGroundRange[i], SetWithGroundAngleRange[j]);
			if (isempty(intersecttmp) == 0)
				intersectionSet.push_back(intersecttmp);
		}
	}

	if (BucketwithGroundRange(1) > BucketwithGroundRange(0))
	{
		for (int i = 0; i < intersectionSet.size(); i++)
		{
			MatrixXd tmp = intersectionSet[i];
			if (size(tmp, 2) == 2)
			{
				MatrixXd rangethis(1, 2);
				rangethis(0) = theta4Range(1) - (BucketwithGroundRange(1) - tmp(0));
				rangethis(1) = theta4Range(1) - (BucketwithGroundRange(1) - tmp(1));
				//cout << rangethis << endl;
				Theta4Range.push_back(rangethis);
			}
			else
			{
				if (size(tmp, 2) == 1)
				{
					MatrixXd rangethis(1, 1);
					rangethis(0) = theta4Range(1) - (BucketwithGroundRange(1) - tmp(0));
					Theta4Range.push_back(rangethis);
				}
				else
					error("程序逻辑出错");
			}	
		}
	}
	else
	{
		if (BucketwithGroundRange(1) < BucketwithGroundRange(0))
		{
			for (int i = 0; i < SetBucketwithGroundRange.size(); i = i + 2)
			{
				for (int j = 0; j < intersectionSet.size(); j++)
				{
					if (isempty(GetIntersection(intersectionSet[j], SetBucketwithGroundRange[i])) == 0)
					{
						if (size(intersectionSet[j], 2) == 1)
						{
							MatrixXd rangethis(1, 1);
							rangethis(0) = SetBucketwithGroundRange[i+1](0) + intersectionSet[j](0) - SetBucketwithGroundRange[i](0);
							Theta4Range.push_back(rangethis);
						}
						else
						{
							MatrixXd rangethis(1, 2);
							rangethis(0) = SetBucketwithGroundRange[i+1](0) + intersectionSet[j](0) - SetBucketwithGroundRange[i](0);
							rangethis(1) = SetBucketwithGroundRange[i+1](0) + intersectionSet[j](1) - SetBucketwithGroundRange[i](0);
							
							Theta4Range.push_back(rangethis);
						}
							
					}
				}
			}
		}
	}

	if (Theta4Range.empty() == 1)
		YES = 0;
	else
		YES = 1;
	if (YES == 1)
	{
		for (int i = 0; i < Theta4Range.size(); i++)
		{
			MatrixXd validcheck;
			validcheck = GetIntersection(Theta4Range[i], theta4Range.transpose());
			//cout << validcheck << endl;
			//cout << Theta4Range[i] << endl;
			if ((validcheck - Theta4Range[i]).norm()>0.001)
				error("程序逻辑出错");
		}
	}

	return Theta4Range;
}