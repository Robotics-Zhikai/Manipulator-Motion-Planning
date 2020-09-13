#include "main.h"
#include "Planning.h"
#include "FIKinematic.h"
#include "EigenMatrixOperate.h"

//using Eigen::MatrixXd;
double m0 = 0 * pi / 180;
double m1 = 90 * pi / 180;
double m2 = 0 * pi / 180;
double m3 = 0 * pi / 180;

double a0 = 0;
double d3 = 0;
double d4 = 0;
double a1 = 12;
double a2 = 460;
double a3 = 210.9;
double d1 = 57.9;
double d2 = 13.7;
double tool = 123.5;

double cosinetheta = -a1 / (sqrt(pow(d2 , 2) + pow(a1 , 2))); //描述机械臂与圆半径的夹角余弦值

Vector2d theta1Range(-179.9999, 180);
Vector2d theta2Range(-40, 44);
Vector2d theta3Range(-130, - 20);
Vector2d theta4Range(-100, 30);

double tinterval = 0.01;

void error(string msg)
{
	cout << "error:"<<msg << endl;
	system("pause");
}

void warning(string msg)
{
	cout << msg << endl;
}

double legalizAnger(double angle) //把角度限制在(-180, 180]
{
	double thetaout = angle;
	while (thetaout > 180)
	{
		thetaout = thetaout - 360;
	}
	while (thetaout <= -180)
	{
		thetaout = thetaout + 360;
	}
	return thetaout;
}



int main()
{
	//////////////////////////////////////////////////////////////////////////////////
	//eigen 函数库用法举例
	//MatrixXd t(1, 0);
	//cout << t;

	//cout << (int)1.9;

	//MatrixXd t(0, 0);
	//cout << t;

	//MatrixXd t(2, 2);
	//t << 1, 2, 3, 4;
	//cout << t.mean() << endl;

	//vector <MatrixXd> tst;
	//MatrixXd a(1, 3);
	//tst.push_back(a);
	//MatrixXd b(3, 3);
	//tst.push_back(b);
	//cout << tst[0] << endl;
	//cout << tst[1] << endl;



	//Vector3d test1, test2;
	//test1 << 1, 2, 3;
	//test2 << 2, 3, 4;
	//cout << test1.dot(test2) << endl;

	
	//Vector3d test;
	//test << 1, 2, 3;
	//cout << test << endl;
	//test.normalize();
	//cout << test << endl;

	//MatrixXd test(2,2);
	//test << 1, 2, 3, 4;
	//cout << test << endl;
	//test.resize(0, 0);
	//cout << test << endl;

	//MatrixXd test(2,3);
	//MatrixXd test1(2,4);
	//test << 1,2,3,4,5,6;
	//test1 << 7,8,9,10,11,12,13,42;
	//cout << test << endl;
	//test = addMatrix2Matrix(test, ";", test1);
	//cout << test << endl;

	/*MatrixXd test(2, 2);
	test << 1, 2, 3, 4;
	cout << test << endl;
	cout << test(0) << " " << test(1) << " " << test(2) << endl;*/

	//Matrix<double, Dynamic, 4> a(4, 4);
	//a = MatrixXd::Random(4, 4);
	//cout << a << endl<<endl;
	//Matrix<double, Dynamic, 3> c(5, 3);
	//c = a.block(0, 0, 3, 4);
	//cout << c << endl; //对于要删除的行 最好弄成dynamic

	//Matrix<double, Dynamic, Dynamic> a(4, 4);
	//a = MatrixXd::Random(4, 4);
	//cout << a << endl << endl;
	//Matrix<double, Dynamic, Dynamic> c(5, 3);
	//c = a.block(0, 0, 3, 4);
	//cout << c << endl; //对于要删除的行 最好弄成dynamic

	//Vector3d s;
	//s[0] = 1;
	//s[1] = 2;
	////s[2] = 3;
	////s[3] = 4;
	//cout << s << endl;
	//error("似乎超出了工作空间，结果不可信");
	//MatrixXd x(3,3);
	//x << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	//MatrixXd y(2,2);
	//y << 1, 2, 3, 4;
	//cout << x << endl;
	//cout << y << endl;
	//x = y;
	//cout << x << endl;
	//cout << y << endl;
	//try
	//{
	//	double a = 1 ;
	//	double b = 0;
	//	double c = a / b;
	//	throw "Division by zero condition!";
	//}
	//catch (const char* msg)
	//{
	//	cerr << msg << endl;
	//}
	//cerr << "ss" << endl;
	//MatrixXd m(2, 2);
	//m(0, 0) = 3;
	//m(1, 0) = 2.5;
	//m(0, 1) = -1;
	//m(1, 1) = m(1, 0) + m(0, 1);
	//std::cout << m << std::endl;

	//double tform[16] = { 1 ,2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
	//double Tsequence[4];
	//////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////
	//以下是运行本程序的demo 各DEMO是独立的

	cout << "******************************************************************************************************" << endl;
	FastReachP2PDEMO();
	cout << "******************************************************************************************************" << endl;

	cout << "******************************************************************************************************" << endl;
	BucketTipLinearPlanningDEMO();
	cout << "******************************************************************************************************" << endl;

	cout << "******************************************************************************************************" << endl;
	ctrajDEMO();
	cout << "******************************************************************************************************" << endl;

	cout << "******************************************************************************************************" << endl;
	ManipulatorPlanningJointSpaceDEMO();
	cout << "******************************************************************************************************" << endl;

	cout << "******************************************************************************************************" << endl;
	WholeSystemDEMO(); //这个跟MATLAB的程序写的内容是一样的 比较一下输出的序列是否一样就可判断程序是否写对了
	cout << "******************************************************************************************************" << endl;

	system("pause");
	//////////////////////////////////////////////////////////////////////////////////
} 