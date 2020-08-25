#include "Planning.h"


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

/* Function Definitions */
/*
* t必须在[0,tf]区间内 k=ta/tb,取值范围为[0,0.5] k_amax取值范围为[0,1]
* -180,180 角度体系
* Arguments    : double theta0
*                double thetaf
*                double tf
*                double AMAX
*                double VMAX
*                double t
*                double k
*                double k_amax
*                double *PointCurrentTime
*                int *success
* Return Type  : void
*/
void ManipulatorPlanningJointSpaceSub(double theta0, double thetaf, double tf, double AMAX,
	double VMAX, double t, double k, double k_amax, double *PointCurrentTime,
	int *success)
{
	
	int flagNeedXG;
	boolean_T guard1 = false;
	boolean_T guard2 = false;
	boolean_T guard3 = false;
	double a_Uppertemp;
	double a_LowerBound_tmp_tmp;
	double a_LowerBound;
	double amax;
	double tb;
	double ta_tmp;

	theta0 = legalizAnger(theta0);
	thetaf = legalizAnger(thetaf);

	*success = 0;
	*PointCurrentTime = theta0;
	if ((t >= 0.0) && (t <= tf))
	{
		flagNeedXG = 0;
		if (theta0 * thetaf < 0.0)
		{
			if (theta0 > 0.0)
			{
				if (theta0 - thetaf > 180.0)
				{
					thetaf += 360.0;
					flagNeedXG = 1;
				}
			}
			else
			{
				if (thetaf - theta0 > 180.0)
				{
					theta0 += 360.0;
					flagNeedXG = 1;
				}
			}
		}

		/*      k = 0.1; %k=ta/tb,取值范围为[0,0.5] */
		if (theta0 == thetaf)
		{
			*PointCurrentTime = theta0;
			*success = 1;
		}
		else 
		{
			guard1 = false;
			guard2 = false;
			guard3 = false;
			if (theta0 < thetaf)
			{
				a_Uppertemp = theta0 - thetaf;
				a_LowerBound_tmp_tmp = tf * tf;
				a_LowerBound = 4.0 * a_Uppertemp / ((k - 1.0) * a_LowerBound_tmp_tmp);
				a_Uppertemp = VMAX * VMAX / ((1.0 - k) * (a_Uppertemp + tf * VMAX));
				if (a_Uppertemp >= a_LowerBound)
				{
					if (a_Uppertemp >= AMAX)
					{
						if (AMAX >= a_LowerBound)
						{
							a_Uppertemp = AMAX;
							guard3 = true;
						}
						else
						{
							cout << "加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX" << endl;
						}
					}
					else if (a_Uppertemp >= a_LowerBound)
					{
						guard3 = true;
					}
					else
					{
						cout << "2加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX" << endl;
					}
				}
				else
				{
					cout << "1VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX" << endl;
				}
			}
			else
			{
				a_Uppertemp = thetaf - theta0;
				a_LowerBound_tmp_tmp = tf * tf;
				a_LowerBound = 4.0 * a_Uppertemp / ((k - 1.0) * a_LowerBound_tmp_tmp);
				a_Uppertemp = VMAX * VMAX / ((1.0 - k) * (a_Uppertemp + tf * VMAX));
				if (a_Uppertemp >= a_LowerBound)
				{
					if (a_Uppertemp >= AMAX)
					{
						if (AMAX >= a_LowerBound)
						{
							a_Uppertemp = AMAX;
							guard2 = true;
						}
						else
						{
							cout << "加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX" << endl;
						}
					}
					else if (a_Uppertemp >= a_LowerBound)
					{
						guard2 = true;
					}
					else
					{
						cout << "2VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX" << endl;
					}
				}
				else
				{
					cout << "1VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX" << endl;
				}
			}

			if (guard3)
			{
				/*          amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1); */
				amax = a_LowerBound + k_amax * (a_Uppertemp - a_LowerBound);

				/*          amax = a_UpperBound; */
				a_Uppertemp = amax * k;
				tb = -((sqrt(-amax * (k - 1.0) * (((4.0 * theta0 - 4.0 * thetaf) + amax *
					a_LowerBound_tmp_tmp) - a_Uppertemp * a_LowerBound_tmp_tmp))
					- amax * tf) + a_Uppertemp * tf) / (2.0 * (amax - a_Uppertemp));
				ta_tmp = k * tb;

				/*              PointsSequence(1,i) = (i-1)*SampleTime; */
				/*              t = PointsSequence(1,i); */
				if ((t >= 0.0) && (t < ta_tmp))
				{
					*PointCurrentTime = amax * pow(t, 3.0) / (6.0 * k * tb) + theta0;
				}

				if ((t >= ta_tmp) && (t < tb - ta_tmp))
				{
					*PointCurrentTime = (theta0 + amax * t * (t - ta_tmp) / 2.0) + amax *
						(k * k) * (tb * tb) / 6.0;
				}

				a_LowerBound = tb - ta_tmp;
				if ((t >= a_LowerBound) && (t < tb))
				{
					a_Uppertemp = k * k;
					*PointCurrentTime = (((((theta0 + amax * (t * t) / (2.0 * k)) + amax *
						(k * k) * (tb * tb) / 6.0) + amax * a_LowerBound * (tb - 2.0 * k *
							tb) / 2.0) - amax * tb * ((6.0 * t * a_Uppertemp - 6.0 * t * k) +
								3.0 * t) / (6.0 * k)) - amax * pow(t, 3.0) / (6.0 * k * tb)) - amax *
								(tb * tb) * (((7.0 * pow(k, 3.0) - 12.0 * a_Uppertemp) + 6.0 * k) -
									1.0) / (6.0 * k);
				}

				if ((t >= tb) && (t < tf - tb))
				{
					a_Uppertemp = amax * (tb - k * tb);
					*PointCurrentTime = (((((theta0 + amax * (tb * tb) / (3.0 * k)) + amax
						* (k * k) * (tb * tb) / 6.0) + a_Uppertemp * (t - tb)) + a_Uppertemp
						* (tb - 2.0 * k * tb) / 2.0) - amax * tb * ((6.0
							* tb * (k * k) - 6.0 * tb * k) + 3.0 * tb) / (6.0 * k)) - amax * (tb
								* tb) * (((7.0 * pow(k, 3.0) - 12.0 * (k * k)) + 6.0 * k) - 1.0) /
								(6.0 * k);
				}

				a_LowerBound = tf - tb;
				if ((t >= a_LowerBound) && (t < a_LowerBound + ta_tmp))
				{
					a_Uppertemp = tb * tb;
					*PointCurrentTime = ((((((theta0 + amax * (tb * tb) / (3.0 * k)) +
						amax * (k * k) * (tb * tb) / 6.0) - amax * (2.0 * tb - tf) * (tb - k
							* tb)) + amax * (tb - k * tb) * (tb - 2.0 * k * tb) / 2.0) - amax *
						tb * ((6.0 * tb * (k * k) - 6.0 * tb * k) + 3.0 *
							tb) / (6.0 * k)) - amax * (tb * tb) * (((7.0 * pow(k, 3.0) - 12.0 *
							(k * k)) + 6.0 * k) - 1.0) / (6.0 * k)) - amax * ((t + tb) - tf) *
								(((((((6.0 * (k * k) * a_Uppertemp - 6.0 * k * a_Uppertemp) + t * t)
									+ 2.0 * t * tb) - 2.0 * t * tf) + a_Uppertemp) - 2.0 * tb * tf)
									+ a_LowerBound_tmp_tmp) / (6.0 * k * tb);
				}

				if ((t >= (tf - tb) + ta_tmp) && (t < tf - ta_tmp))
				{
					a_Uppertemp = a_LowerBound + ta_tmp;
					*PointCurrentTime = (((((((theta0 - amax * (((((((a_Uppertemp *
						a_Uppertemp - 2.0 * tb * tf) - 6.0 * k * (tb * tb)) + 2.0 * tb *
						a_Uppertemp) - 2.0 * tf * a_Uppertemp) + tb * tb) +
						a_LowerBound_tmp_tmp) + 6.0 * (k * k) * (tb * tb)) / 6.0) + amax *
						(tb * tb) / (3.0 * k)) + amax * (k * k) * (tb * tb) / 6.0) - amax *
						(((t + tb) - tf) - ta_tmp) * (((t - tb) - tf) + 2.0 * k * tb) / 2.0)
						- amax * (2.0 * tb - tf) * (tb - k * tb)) + amax * (tb - k * tb) *
						(tb - 2.0 * k * tb) / 2.0) - amax * tb * ((6.0 *
							tb * (k * k) - 6.0 * tb * k) + 3.0 * tb) / (6.0 * k)) - amax * (tb *
								tb) * (((7.0 * pow(k, 3.0) - 12.0 * (k * k)) + 6.0 * k) - 1.0) /
								(6.0 * k);
				}

				if ((t >= tf - ta_tmp) && (t <= tf))
				{
					a_Uppertemp = (tf - tb) + k * tb;
					*PointCurrentTime = (((((((theta0 - amax * (((((((a_Uppertemp *
						a_Uppertemp - 2.0 * tb * tf) - 6.0 * k * (tb * tb)) + 2.0 * tb *
						((tf - tb) + k * tb)) - 2.0 * tf * ((tf - tb) + k * tb)) + tb * tb)
						+ a_LowerBound_tmp_tmp) + 6.0 * (k * k) * (tb * tb)) / 6.0) + amax *
						(tb * tb) / (3.0 * k)) + amax * (k * k) * (tb * tb) / 3.0) - amax *
						(2.0 * tb - tf) * (tb - k * tb)) + amax * (tb - k * tb) * (tb - 2.0 *
							k * tb)) - amax * tb * ((6.0 * tb * (k * k) - 6.0 * tb * k) + 3.0 *
								tb) / (6.0 * k)) + amax * (((pow(t, 3.0) - 3.0 * (t * t) * tf) + 3.0
									* t * a_LowerBound_tmp_tmp) - pow(tf, 3.0)) / (6.0 * k * tb)) - amax
						* (tb * tb) * (((7.0 * pow(k, 3.0) - 12.0 * (k * k)) + 6.0 * k) -
							1.0) / (6.0 * k);
				}

				guard1 = true;
			}

			if (guard2)
			{
				/*          amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1); */
				/*          amax = a_UpperBound; */
				amax = a_LowerBound + k_amax * (a_Uppertemp - a_LowerBound);
				a_Uppertemp = amax * k;
				tb = -((sqrt(-amax * (k - 1.0) * (((4.0 * thetaf - 4.0 * theta0) + amax *
					a_LowerBound_tmp_tmp) - a_Uppertemp * a_LowerBound_tmp_tmp))
					- amax * tf) + a_Uppertemp * tf) / (2.0 * (amax - a_Uppertemp));
				ta_tmp = k * tb;

				/*          for i=1:tf/SampleTime+1 */
				/*              PointsSequence(1,i) = (i-1)*SampleTime; */
				/*              t = PointsSequence(1,i); */
				t = 2.0 * (tf / 2.0) - t;
				if ((t >= 0.0) && (t < ta_tmp))
				{
					*PointCurrentTime = amax * pow(t, 3.0) / (6.0 * k * tb) + thetaf;
				}

				if ((t >= ta_tmp) && (t < tb - ta_tmp))
				{
					*PointCurrentTime = (thetaf + amax * t * (t - ta_tmp) / 2.0) + amax *
						(k * k) * (tb * tb) / 6.0;
				}

				if ((t >= tb - ta_tmp) && (t < tb))
				{
					*PointCurrentTime = (((((thetaf + amax * (t * t) / (2.0 * k)) + amax *
						(k * k) * (tb * tb) / 6.0) + amax * (tb - k * tb) * (tb - 2.0 * k *
							tb) / 2.0) - amax * tb * ((6.0 * t * (k * k) - 6.0 * t * k) + 3.0 *
								t) / (6.0 * k)) - amax * pow(t, 3.0) / (6.0 * k * tb)) - amax * (tb *
									tb) * (((7.0 * pow(k, 3.0) - 12.0 * (k * k)) + 6.0 * k) - 1.0) /
									(6.0 * k);
				}

				if ((t >= tb) && (t < tf - tb))
				{
					*PointCurrentTime = (((((thetaf + amax * (tb * tb) / (3.0 * k)) + amax
						* (k * k) * (tb * tb) / 6.0) + amax * (tb - k * tb) * (t - tb)) +
						amax * (tb - k * tb) * (tb - 2.0 * k * tb) / 2.0)
						- amax * tb * ((6.0 * tb * (k * k) - 6.0 * tb * k)
							+ 3.0 * tb) / (6.0 * k)) - amax * (tb * tb) * (((7.0 * pow(k, 3.0) -
								12.0 * (k * k)) + 6.0 * k) - 1.0) / (6.0 * k);
				}

				if ((t >= tf - tb) && (t < (tf - tb) + ta_tmp))
				{
					*PointCurrentTime = ((((((thetaf + amax * (tb * tb) / (3.0 * k)) +
						amax * (k * k) * (tb * tb) / 6.0) - amax * (2.0 * tb - tf) * (tb - k
							* tb)) + amax * (tb - k * tb) * (tb - 2.0 * k * tb) / 2.0) - amax *
						tb * ((6.0 * tb * (k * k) - 6.0 * tb * k) + 3.0 *
							tb) / (6.0 * k)) - amax * (tb * tb) * (((7.0 * pow(k, 3.0) - 12.0 *
							(k * k)) + 6.0 * k) - 1.0) / (6.0 * k)) - amax * ((t + tb) - tf) *
								(((((((6.0 * (k * k) * (tb * tb) - 6.0 * k * (tb * tb)) + t * t) +
									2.0 * t * tb) - 2.0 * t * tf) + tb * tb) - 2.0 * tb * tf) + tf *
									tf) / (6.0 * k * tb);
				}

				if ((t >= (tf - tb) + ta_tmp) && (t < tf - ta_tmp))
				{
					a_Uppertemp = (tf - tb) + k * tb;
					*PointCurrentTime = (((((((thetaf - amax * (((((((a_Uppertemp *
						a_Uppertemp - 2.0 * tb * tf) - 6.0 * k * (tb * tb)) + 2.0 * tb *
						a_Uppertemp) - 2.0 * tf * a_Uppertemp) + tb * tb) +
						a_LowerBound_tmp_tmp) + 6.0 * (k * k) * (tb * tb)) / 6.0) + amax *
						(tb * tb) / (3.0 * k)) + amax * (k * k) * (tb * tb) / 6.0) - amax *
						(((t + tb) - tf) - ta_tmp) * (((t - tb) - tf) + 2.0 * k * tb) / 2.0)
						- amax * (2.0 * tb - tf) * (tb - k * tb)) + amax * (tb - k * tb) *
						(tb - 2.0 * k * tb) / 2.0) - amax * tb * ((6.0 *
							tb * (k * k) - 6.0 * tb * k) + 3.0 * tb) / (6.0 * k)) - amax * (tb *
								tb) * (((7.0 * pow(k, 3.0) - 12.0 * (k * k)) + 6.0 * k) - 1.0) /
								(6.0 * k);
				}

				if ((t >= tf - ta_tmp) && (t <= tf))
				{
					a_Uppertemp = (tf - tb) + k * tb;
					*PointCurrentTime = (((((((thetaf - amax * (((((((a_Uppertemp *
						a_Uppertemp - 2.0 * tb * tf) - 6.0 * k * (tb * tb)) + 2.0 * tb *
						((tf - tb) + k * tb)) - 2.0 * tf * ((tf - tb) + k * tb)) + tb * tb)
						+ a_LowerBound_tmp_tmp) + 6.0 * (k * k) * (tb * tb)) / 6.0) + amax *
						(tb * tb) / (3.0 * k)) + amax * (k * k) * (tb * tb) / 3.0) - amax *
						(2.0 * tb - tf) * (tb - k * tb)) + amax * (tb - k * tb) * (tb - 2.0 *
							k * tb)) - amax * tb * ((6.0 * tb * (k * k) - 6.0 * tb * k) + 3.0 *
								tb) / (6.0 * k)) + amax * (((pow(t, 3.0) - 3.0 * (t * t) * tf) + 3.0
									* t * a_LowerBound_tmp_tmp) - pow(tf, 3.0)) / (6.0 * k * tb)) - amax
						* (tb * tb) * (((7.0 * pow(k, 3.0) - 12.0 * (k * k)) + 6.0 * k) -
							1.0) / (6.0 * k);
				}

				guard1 = true;
			}

			if (guard1)
			{
				
				if (flagNeedXG == 1)// && (*PointCurrentTime > 180.0))
				{
					*PointCurrentTime = legalizAnger(*PointCurrentTime);
					//*PointCurrentTime = (*PointCurrentTime - 180.0) + -180.0;
				}

				*success = 1;
			}
		}
	}
	else
	{
		cout << t << " " << tf << " t不能大于tf或小于0" << endl;
		/*          disp("t不能小于0！"); */
		/*          disp('t不能大于tf！'); */
	}
}

void ManipulatorPlanningJointSpaceDEMO()//如何使用ManipulatorPlanningJointSpaceSub函数的导引函数
{
	//theta0, thetaf, tf, AMAX, VMAX, SampleTime 
	//theta0 是初始角度
	//thetaf 是末角度
	//tf 是持续时间
	//AMAX 是整个过程中加速度的最大值 单位是度每二次方秒
	//VMAX 是整个过程中速度的最大值 单位是度每秒
	//SampleTime 是采样时间

	double theta0 = -122.1958;
	double thetaf = 80.0281;
	double tf = 10;
	double AMAX = 500;
	double VMAX = 30;
	double SampleTime = 0.1;

	struct Element
	{
		double time;
		double angle;
	};
	vector <Element> Sequence; //存放时间戳和对应规划的角度

	for (int i = 1; i <= tf / SampleTime + 1; i++)
	{
		double t = (i - 1)*SampleTime;
		double PointCurrentTime;
		int success;
		ManipulatorPlanningJointSpaceSub(theta0, thetaf, tf, AMAX, VMAX, t, 0.1, 1, &PointCurrentTime, &success);
		cout << t << " " << PointCurrentTime << endl;
		if (success == 1)
		{
			Element tmp;
			tmp.time = t;
			tmp.angle = PointCurrentTime;
			Sequence.push_back(tmp);
		}
		else
		{
			cout << "加速度和速度限制导致不能成功规划，或者有其他如参数t的异常，需要修改相关参数" << endl;
			return;
		}
	}
}

vector<double> lspb_c(double q0, double q1, double t_input)
{
	vector<double> s;
	s.clear();
	double t0 = t_input;
	vector <double> t;
	for (int i = 0; i <= t_input - 1; i++)
	{
		t.push_back(i);
	}
	double tf = *max_element(t.begin(), t.end());
	double V = (q1 - q0) / tf * 1.5;

	if (q0 == q1)
	{
		for (int i = 0; i < t.size(); i++)
		{
			s.push_back(q0);
		}
		return s;
	}
	double tb = (q0 - q1 + V*tf) / V;
	double a = V / tb;
	vector<double> p,pd,pdd;
	for (int i = 0; i < t.size(); i++)
	{
		p.push_back(0);
	}
	pd = p;
	pdd = p;
	for (int i = 0; i < t.size(); i++)
	{
		double tt = t[i];
		if (tt <= tb)
		{
			p[i] = q0 + a / 2 * pow(tt , 2);
			pd[i] = a*tt;
			pdd[i] = a;
		}
		else if (tt <= (tf - tb))
		{
			p[i] = (q1 + q0 - V*tf) / 2 + V*tt;
			pd[i] = V;
			pdd[i] = 0;
		}
		else
		{
			p[i] = q1 - a / 2 * pow(tf , 2) + a*tf*tt - a / 2 * pow(tt , 2);
			pd[i] = a*tf - a*tt;
			pdd[i] = -a;
		}
	}
	s = p;
	return s;
}

class UnitQuaternion
{
public:
	double s;
	//vector <double> v;
	Vector3d v;

	UnitQuaternion()
	{
		this->s = 0;
		this->v << 0, 0, 0;
	}
	Matrix3d R() //四元数到3*3的旋转矩阵的转换
	{
		Matrix3d r;
		r = Matrix3d::Zero();
		double s = this->s;
		double x = this->v(0);
		double y = this->v(1);
		double z = this->v(2);
		r(0, 0) = 1 - 2 * (pow(y , 2) + pow(z , 2));
		r(0, 1) = 2 * (x*y - s*z);
		r(0, 2) = 2 * (x*z + s*y);
		r(1, 0) = 2 * (x*y + s*z);
		r(1, 1) = 1 - 2 * (pow(x , 2) + pow(z , 2));
		r(1, 2) = 2 * (y*z - s*x);
		r(2, 0) = 2 * (x*z - s*y);
		r(2, 1) = 2 * (y*z + s*x);
		r(2, 2) = 1 - 2 * (pow(x , 2) + pow(y , 2));
		return r;
	}
};

double sign(double x)
{
	if (x >= 0)
		return 1;
	else
		return -1;
}

MatrixXd Shuzu2Matrix(double * shuzu,int Length,int rows,int cols)
{
	//输入为数组、数组的长度、要转换的矩阵的行和列的值，输出为矩阵的形式
	if (rows*cols != Length)
	{
		cerr << "ERROR:输入的数组数量不足或超过" << endl;
		system("pause");
	}
	MatrixXd TransformMatrix(rows,cols);
	int k = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			TransformMatrix(i, j) = shuzu[k];
			k++;
		}
	}
	return TransformMatrix;
}

UnitQuaternion Tran2Quaternion_c(Matrix4d T)
{
	Matrix<double, 4, 4> TransformMatrix = T;

	Matrix<double, 3, 3> R;
	R = TransformMatrix.block(0, 0, 3, 3);
	double s = sqrt(R.trace() + 1) / 2.0;
	double kx = R(2, 1) - R(1, 2);   // Oz - Ay;
	double ky = R(0, 2) - R(2, 0);   // Ax - Nz
	double kz = R(1, 0) - R(0, 1);   // Ny - Ox
	double maxdiag = R.diagonal().maxCoeff();
	int k_diag;
	if (maxdiag == R(0, 0))
		k_diag = 1;
	else if (maxdiag == R(1, 1))
		k_diag = 2;
	else if (maxdiag == R(2, 2))
		k_diag = 3;
	else
	{
		cerr << "用法用错了" << endl;
		system("pause");
	}

	double kx1, ky1, kz1, sgn;
	switch (k_diag)
	{
	case 1:
		kx1 = R(0, 0) - R(1, 1) - R(2, 2) + 1; // Nx - Oy - Az + 1
		ky1 = R(1, 0) + R(0, 1);          // Ny + Ox
		kz1 = R(2, 0) + R(0, 2);          // Nz + Ax
		sgn = sign(kx);
		break;
	case 2:
		kx1 = R(1, 0) + R(0, 1);          // Ny + Ox
		ky1 = R(1, 1) - R(0, 0) - R(2, 2) + 1; // Oy - Nx - Az + 1
		kz1 = R(2, 1) + R(1, 2);          // Oz + Ay
		sgn = sign(ky);
		break;
	case 3:
		kx1 = R(2, 0) + R(0, 2);          // Nz + Ax
		ky1 = R(2, 1) + R(1, 2);          // Oz + Ay
		kz1 = R(2, 2) - R(0, 0) - R(1, 1) + 1; // Az - Nx - Oy + 1
		sgn = sign(kz);
		break;
	default:
		break;
	}

	kx = kx + sgn*kx1;
	ky = ky + sgn*ky1;
	kz = kz + sgn*kz1;

	UnitQuaternion q;
	double nm = sqrt(pow(kx, 2) + pow(ky, 2) + pow(kz, 2));
	if (nm == 0)
	{
		//handle special case of null quaternion
		q.s = 1;
		q.v << 0, 0, 0;
	}
	else
	{
		q.s = s;

		q.v(0) = (kx*sqrt(1 - pow(s, 2)) / nm);
		q.v(1) = (ky*sqrt(1 - pow(s, 2)) / nm);
		q.v(2) = (kz*sqrt(1 - pow(s, 2)) / nm);
	}
	return q;
}

UnitQuaternion interp(UnitQuaternion Q0, UnitQuaternion Q1,double s, int Isshortest)
{
	Vector4d q1(Q0.s, Q0.v(0), Q0.v(1), Q0.v(2));
	Vector4d q2(Q1.s, Q1.v(0), Q1.v(1), Q1.v(2));
	double r = s;
	
	double cosTheta = (q1.transpose())*q2;

	if (Isshortest == 1)
	{
		if (cosTheta < 0)
		{
			q1 = -q1;
			cosTheta = -cosTheta;
		}
	}

	double theta = acos(cosTheta);

	if (r < 0 || r>1)
	{
		cerr << "ERRO:SMTB:UnitQuaternion:interp', 'values of S outside interval [0,1]" << endl;
		system("pause");
	}

	UnitQuaternion q;
	if (theta == 0)
		q = Q0;
	else
	{
		Vector4d tmp = (sin((1 - r)*theta) * q1 + sin(r*theta) * q2) / sin(theta);
		tmp = tmp / tmp.norm();

		q.s = tmp[0]; //这个在索引后不知为何值会变
		q.v = tmp.block(1, 0, 3, 1);
	}
	return q;
}

Matrix<double,Dynamic,Dynamic> trinterp_c(Matrix4d A, Matrix4d B, double C,int Isshortest)
{
	Matrix4d T0, T1;
	T0 = A;
	T1 = B;

	vector <double> s ;
	s.push_back(C);

	if ((s.size() == 1) && (s[0] > 1) && (s[0] == floor(s[0])))
	{
		//s = linspace(0, 1, s);
		s[0] = 0;
		for (double i = (1.0 / (s[1] - 1)); i <= 1; i = i + (1.0 / (s[1]-1)))
		{
			s.push_back(i);
		}
		*(s.end() - 1) = 1;
	}
	for (int i = 0; i < s.size(); i++)
	{
		if (s[i] < 0 || s[i]>1)
		{
			cerr << "ERROR:SMTB:trinterp:badarg', 'values of S outside interval [0,1]" << endl;
			system("pause");
		}
	}

	UnitQuaternion q0 = Tran2Quaternion_c(T0);
	UnitQuaternion q1 = Tran2Quaternion_c(T1);
	
	Vector3d p0 = T0.block(0, 3, 3, 1);
	Vector3d p1 = T1.block(0, 3, 3, 1);

	UnitQuaternion qr;
	Vector3d pr;
	Matrix<double, Dynamic, Dynamic> T(4,4) ;
	T.resize(4, 4 * s.size());
	for (int i = 0; i < s.size(); i++)
	{
		qr = interp(q0, q1, s[i], Isshortest);
		pr = p0*(1 - s[i]) + s[i]*p1;
		Matrix4d Ttmp = Matrix4d::Zero();
		Ttmp.block(0, 0, 3, 3) = qr.R();
		Ttmp.block(0, 3, 3, 1) = pr;
		Ttmp.block(3, 0, 1, 4) << 0, 0, 0, 1;
		T.block(0, Ttmp.cols()*i, Ttmp.rows(), Ttmp.cols()) = Ttmp;
	}
	return T;
}

Matrix<double, Dynamic, Dynamic> ctraj_c(Matrix4d T0, Matrix4d T1, double t_input,int Isshortest)
{
	//插值函数
	vector<double> s;
	s = lspb_c(0, 1, t_input);
	Matrix<double, Dynamic, Dynamic> traj(4,4);
	traj.resize(4, 4 * s.size());
	int lie = 0;
	for (int i = 0; i < s.size(); i++)
	{
		Matrix<double, Dynamic, Dynamic> TMP;
		TMP = trinterp_c(T0, T1, s[i], Isshortest);
		traj.block(0, lie, TMP.rows(), TMP.cols()) = TMP;
		lie = lie + TMP.cols();
	}
	return traj;
}

void ctrajDEMO()
{
	//如何调用ctraj_c 
	//Matrix4d T0 4*4的矩阵，表示初始的齐次转换矩阵
	//Matrix4d T1 4*4的矩阵，表示末的齐次转换矩阵
	//double t_input,表示插多少值（数量） 正整数
	Matrix4d T0;
	Matrix4d T1;

	//T0 << 0.9320, 0.3623, -0.0120, 681.1443,
	//	-0.0112, -0.0043, -0.9999, -21.8567,
	//	-0.3623, 0.9321, 0.0000, 172.4116,
	//	0, 0, 0, 1.0000;
	//
	//T1 << -0.3652, 0.9309, -0.0120, 546.0533,
	//	0.0044, -0.0111, -0.9999, -20.2392,
	//	-0.9309, -0.3652, 0.0000, -29.5721,
	//	0, 0, 0, 1.0000;

	T0 << 0.3395, 0.8375, -0.4283, 550.8013,
		-0.1609, -0.3969, -0.9037, -276.1908,
		-0.9268, 0.3756, 0.0000, -343.4893,
		0, 0, 0, 1.0000;
	T1 << -0.8333, 0.3495, -0.4283, 398.1105,
		0.3949, -0.1656, -0.9037, -203.8291,
		-0.3868, -0.9222, 0.0000, -197.4437,
		0, 0, 0, 1.0000;


	double t = 200;
	Matrix<double, Dynamic, Dynamic> traj; //存放插值后的齐次转换矩阵 4行 4*t列
	traj = ctraj_c(T0, T1, t, 1);
	//traj = ctraj_c(T0, T1, t, 0); 

	for (int i = 1; i <= t; i++)
	{
		//cout << i << endl;
		cout << traj.block(0, 4 * (i - 1), 4, 4) << endl;
	}
}

int IsAnglesInLimitRange(Vector4d jointAngles)
{
	if ((jointAngles(0) < theta1Range(0) || jointAngles(0) > theta1Range(1)) || (jointAngles(1) < theta2Range(0) || jointAngles(1) > theta2Range(1)) \
		|| (jointAngles(2) < theta3Range(0) || jointAngles(2) > theta3Range(1)) || (jointAngles(3) < theta4Range(0) || jointAngles(3) > theta4Range(1)))
		return 0;
	else
		return 1;
}

void BucketTipLinearPlanningDEMO()
{
	//如何调用BucketTipLinearPlanning 
	//Matrix4d T0 4*4的矩阵，表示初始的齐次转换矩阵
	//Matrix4d T1 4*4的矩阵，表示末的齐次转换矩阵
	//Vtheta2Max Vtheta3Max Vtheta4Max 各自的最大允许角速度 单位为度每秒
	//atheta2max atheta3max atheta4max 各自的最大允许角加速度 单位为度每二次方秒

	//输出Sequence 储存带时间戳的四个角度 5行 num列矩阵

	Matrix4d T0;
	Matrix4d T1;

	//T0 << 0.9320, 0.3623, -0.0120, 681.1443,
	//	-0.0112, -0.0043, -0.9999, -21.8567,
	//	-0.3623, 0.9321, 0.0000, 172.4116,
	//	0, 0, 0, 1.0000;
	//
	//T1 << -0.3652, 0.9309, -0.0120, 546.0533,
	//	0.0044, -0.0111, -0.9999, -20.2392,
	//	-0.9309, -0.3652, 0.0000, -29.5721,
	//	0, 0, 0, 1.0000;

	T0 << 0.3395, 0.8375, -0.4283, 550.8013,
		-0.1609, -0.3969, -0.9037, -276.1908,
		-0.9268, 0.3756, 0.0000, -343.4893,
		0, 0, 0, 1.0000;
	T1 << -0.8333, 0.3495, -0.4283, 398.1105,
		0.3949, -0.1656, -0.9037, -203.8291,
		-0.3868, -0.9222, 0.0000, -197.4437,
		0, 0, 0, 1.0000;

	MatrixXd Sequence = BucketTipLinearPlanning(T0, T1, 35, 35, 35, 25, 25, 25);
	for (int i = 0; i < Sequence.cols(); i++)
	{
		cout << Sequence.col(i).transpose() << endl;
	}
}

MatrixXd abs(MatrixXd mat)
{
	MatrixXd matout(mat.rows(),mat.cols());
	for (int i = 0; i < mat.rows(); i++)
	{
		for (int j = 0; j < mat.cols(); j++)
		{
			matout(i, j) = abs(mat(i, j));
		}
	}
	return matout;
}

MatrixXd BucketTipLinearPlanning(Matrix4d BeginT, Matrix4d EndT, double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta2max, double atheta3max, double atheta4max)
{
	Matrix<double, Dynamic, Dynamic> Tsequence = ctraj_c(BeginT, EndT, 200, 1); //优先是1 若1不能就0  优先shortest

	Matrix<double, Dynamic, Dynamic> jointangleSeq(Tsequence.cols() / 4,4);

	int countInDead = 0;
	int countValidjointangleSeqRow = 0;
	for (int i = 1; i <= Tsequence.cols()/4; i++)
	{
		Matrix4d tform = Tsequence.block(0, 4 * (i - 1), 4, 4);
		Vector4d jointangle = InverseKinematics(tform);
		if (IsAnglesInLimitRange(jointangle) == 0)
		{
			countInDead++;
			if (countInDead == 2)
			{
				warning("设计的规划算法使得角度超出了物理限制,最终输出的序列不是完整到达目标位置的");
				break;
			}
			warning(" 有可能插值算法插的方向错了'shortest'或非'shortest'");
			
			Tsequence = MatrixXd::Zero(Tsequence.rows(), Tsequence.cols());
			Tsequence = ctraj_c(BeginT, EndT, 200, 0);
			//cout << jointangleSeq;
			jointangleSeq = MatrixXd::Zero(Tsequence.cols() / 4, 4);
			//cout << "asdfasdfasdf" << endl;
			//cout << jointangleSeq;
			i = 0;
			countValidjointangleSeqRow = 0;
			continue;
		}
		jointangleSeq.row((i - 1)) = jointangle.transpose(); //如果想要动态扩展的话需要初始化一个空间 然后resize。不能用resize 每次用都要清除原来的数据，只能预分配
		countValidjointangleSeqRow++;
	}
	jointangleSeq = jointangleSeq.block(0, 0, countValidjointangleSeqRow, jointangleSeq.cols());
	//20200825 检验这个对不对 然后把0 和1 对调 应该是正确的 误差在0.1度

	double Leftbeishu = 0;
	double Rightbeishu = 4000; //这意味着40秒走最多8cm 一秒2mm 基本可以认定是静止的了
	double timesBEISHU = (Leftbeishu + Rightbeishu) / 2.0;
	int FLAGWHILE = 0;

	while (FLAGWHILE < 2)
	{
		MatrixXd djointangleSeq(jointangleSeq.rows() - 1, 4);
		for (int i = 0; i < jointangleSeq.rows() - 1; i++)
			djointangleSeq.row(i) = (jointangleSeq.row(i + 1) - jointangleSeq.row(i)) / (timesBEISHU*tinterval);
		MatrixXd ddjointangleSeq(djointangleSeq.rows() - 1, 4);
		for (int i = 0;i<djointangleSeq.rows() - 1;i++)
			ddjointangleSeq.row(i) = (djointangleSeq.row(i + 1) - djointangleSeq.row(i)) / (timesBEISHU*tinterval);
	
		if (Rightbeishu - Leftbeishu <= 1) // 表明二分法精度到1 最后得到timesBEISHU的临界取值
		{
			FLAGWHILE = FLAGWHILE + 1;
			timesBEISHU = Rightbeishu;
			timesBEISHU = ceil(timesBEISHU);
			if (abs(djointangleSeq.col(1)).maxCoeff() > Vtheta2Max || abs(djointangleSeq.col(2)).maxCoeff() > Vtheta3Max \
				|| abs(djointangleSeq.col(3)).maxCoeff() > Vtheta4Max || abs(ddjointangleSeq.col(1)).maxCoeff() > atheta2max \
				|| abs(ddjointangleSeq.col(2)).maxCoeff() > atheta3max || abs(ddjointangleSeq.col(3)).maxCoeff() > atheta4max)
			{
				if (FLAGWHILE == 2)
					error("设置的精度出错");
			}
		}
		else
		{
			if (abs(djointangleSeq.col(1)).maxCoeff() > Vtheta2Max || abs(djointangleSeq.col(2)).maxCoeff() > Vtheta3Max \
				|| abs(djointangleSeq.col(3)).maxCoeff() > Vtheta4Max || abs(ddjointangleSeq.col(1)).maxCoeff() > atheta2max \
				|| abs(ddjointangleSeq.col(2)).maxCoeff() > atheta3max || abs(ddjointangleSeq.col(3)).maxCoeff() > atheta4max)
				Leftbeishu = timesBEISHU;
			else
				Rightbeishu = timesBEISHU;
			timesBEISHU = Leftbeishu + (Rightbeishu - Leftbeishu) / 2.0;
		}
	}

	MatrixXd Sequence(5, (int)(jointangleSeq.rows()+ (jointangleSeq.rows()-1)*(timesBEISHU-1))) ; //最终返回带时间戳的各角度
	int k_Sequence = 0;
	
	for (int i = 0; i < jointangleSeq.rows() - 1; i++)
	{
		
		double timethis = i*(timesBEISHU*tinterval);
		MatrixXd tmpSequencecol(5, 1);
		tmpSequencecol << timethis, jointangleSeq.row(i)(0), jointangleSeq.row(i)(1), jointangleSeq.row(i)(2), jointangleSeq.row(i)(3);
		Sequence.col(k_Sequence) = tmpSequencecol;
		//cout << tmpSequencecol << endl;
		MatrixXd tmp = jointangleSeq.row(i + 1) - jointangleSeq.row(i);
		tmp = tmp / timesBEISHU;
		
		for (int j = 1; j <= timesBEISHU - 1; j++)
		{
			Sequence.col(k_Sequence + j)(0) = timethis + j*tinterval;
			MatrixXd tmpthis = Sequence.col(k_Sequence + j - 1).block(1, 0, 4, 1) + tmp.transpose();
			Sequence.col(k_Sequence + j).block(1, 0, 4, 1) = tmpthis;
		}
		k_Sequence = k_Sequence + timesBEISHU;
	}
	double timethis = (jointangleSeq.rows() - 1)*(timesBEISHU*tinterval);
	MatrixXd tmpSequencecol(5, 1);
	tmpSequencecol << timethis, jointangleSeq.row(jointangleSeq.rows() - 1)(0), jointangleSeq.row(jointangleSeq.rows() - 1)(1), jointangleSeq.row(jointangleSeq.rows() - 1)(2), jointangleSeq.row(jointangleSeq.rows() - 1)(3);
	Sequence.col(Sequence.cols() - 1) = tmpSequencecol;

	return Sequence;
}

