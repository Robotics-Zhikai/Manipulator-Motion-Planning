#include "main.h"
#include "Planning.h"
#include "FIKinematic.h"

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
	cout << msg << endl;
	system("pause");
}

void warning(string msg)
{
	cout << msg << endl;
}

int main()
{
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

	BucketTipLinearPlanningDEMO();
	//ctrajDEMO();
	//ManipulatorPlanningJointSpaceDEMO();
	//system("pause");
}