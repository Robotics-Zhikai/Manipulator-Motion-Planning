#include "FIKinematic.h"

double mathAtan2(double y, double x)
{
	if (fabs(y) < ZERO)
		y = 0.0;
	if (fabs(x) < ZERO)
		x = 0.0;

	return atan2(y, x);

}

void mathMatxMult(double *matrix1, double *matrix2, double *matrix3, int row, int xxx, int col)
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

//position1Ϊ������ת���ĵ����꣬position2Ϊ������ĩ�˵�����
void ForwardKinematics(double *jointangle, double *position1, double *position2)
{
	double m[4], a[4], theta[4], d[4], tool;
	double m_matrix1[16], m_matrix2[16], m_matrix3[16], m_matrix4[16], m_matrix5[16];
	double m_matrixtemp1[16], m_matrixtemp2[16], m_matrixtemp3[16], m_matrixtemp4[16];
	double m_dRPY[3];

	m[0] = 0.0*M_PI / 180.0;
	m[1] = 90.0*M_PI / 180.0;
	m[2] = 0.0*M_PI / 180.0;
	m[3] = 0.0*M_PI / 180.0;

	a[0] = 0.0;
	a[1] = 12.0;
	a[2] = 460.0;
	a[3] = 210.9;

	d[0] = 57.9;
	d[1] = 13.7;//13.7
	d[2] = 0.0;
	d[3] = 0.0;
	tool = 123.5;

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

	//ŷ����A��B��C begin
	m_dRPY[1] = mathAtan2(-m_matrixtemp4[8], sqrt(m_matrixtemp4[0] * m_matrixtemp4[0] + m_matrixtemp4[4] * m_matrixtemp4[4]));//ŷ����B

	if ((m_dRPY[1]<M_PI_2 + ZERO) && (m_dRPY[1]>M_PI_2 - ZERO))
	{
		m_dRPY[2] = mathAtan2(m_matrixtemp4[1], m_matrixtemp4[5]);
		m_dRPY[0] = 0.0;
	}
	else if ((m_dRPY[1]<-M_PI_2 + ZERO) && (m_dRPY[1]>-M_PI_2 - ZERO))
	{
		m_dRPY[2] = -mathAtan2(m_matrixtemp4[1], m_matrixtemp4[5]);//ŷ����A
		m_dRPY[0] = 0.0;//ŷ����C
	}
	else
	{
		m_dRPY[2] = mathAtan2(m_matrixtemp4[4] / cos(m_dRPY[1]), m_matrixtemp4[0] / cos(m_dRPY[1]));//ŷ����A
		m_dRPY[0] = mathAtan2(m_matrixtemp4[9] / cos(m_dRPY[1]), m_matrixtemp4[10] / cos(m_dRPY[1]));//ŷ����C
	}
	//ŷ����A��B��C end

	//printf("\nEuler%f %f %f\n", m_dRPY[0] * 180.0 / M_PI, m_dRPY[1] * 180.0 / M_PI, m_dRPY[2] * 180.0 / M_PI);

	for (int i = 0; i < 16; i++)
	{
		position1[i] = m_matrixtemp3[i];//������ת��������
		position2[i] = m_matrixtemp4[i];//ĩ�˲�������
	}
}

int InverseKinematics(double position[16], double *jointAngle)
{
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4], tool;

	double mTempAngleOne[2];

	m[0] = 0.0*M_PI / 180.0;
	m[1] = 90.0*M_PI / 180.0;
	m[2] = 0.0*M_PI / 180.0;
	m[3] = 0.0*M_PI / 180.0;

	a[0] = 0.0;
	a[1] = 12.0;
	a[2] = 460.0;
	a[3] = 210.9;
	tool = 123.5;

	d[0] = 57.9;
	d[1] = 13.7;//13.7
	d[2] = 0.0;
	d[3] = 0.0;

	nx = position[0];
	ny = position[4];
	nz = position[8];

	ox = position[1];
	oy = position[5];
	oz = position[9];

	ax = position[2];
	ay = position[6];
	az = position[10];

	px = position[3];
	py = position[7];
	pz = position[11];

	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle[0] = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

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

//���Ϊǰ�����ǵĽǶ�
int InverseKinematicsPos2Angle(double position[3], double * jointAngle)
{
	double nx, ny, nz;
	double ox, oy, oz;
	double ax, ay, az;
	double px, py, pz;
	double m[4], a[4], theta[4], d[4], tool;

	double mTempAngleOne[2];

	m[0] = 0.0*M_PI / 180.0;
	m[1] = 90.0*M_PI / 180.0;
	m[2] = 0.0*M_PI / 180.0;
	m[3] = 0.0*M_PI / 180.0;

	a[0] = 0.0;
	a[1] = 12.0;
	a[2] = 460.0;
	a[3] = 210.9;
	tool = 123.5;

	d[0] = 57.9;
	d[1] = 13.7;//13.7
	d[2] = 0.0;
	d[3] = 0.0;

	px = position[0];
	py = position[1];
	pz = position[2];

	mTempAngleOne[0] = mathAtan2(py, px) - mathAtan2(-d[1], sqrt(px*px + py*py - d[1] * d[1]));
	mTempAngleOne[1] = mathAtan2(py, px) - mathAtan2(-d[1], -sqrt(px*px + py*py - d[1] * d[1]));

	//    printf("\n mTempAngleOne=%f %f\n",mTempAngleOne[0]*180.0/M_PI,mTempAngleOne[1]*180.0/M_PI);
	jointAngle[0] = mTempAngleOne[0];
	double Mtemp1 = (pow(cos(jointAngle[0])*px + sin(jointAngle[0])*py - a[1], 2) + pow(pz - d[0], 2) - pow(a[2], 2) - pow(a[3], 2)) / (2 * a[2] * a[3]);

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

	return 0;
}

