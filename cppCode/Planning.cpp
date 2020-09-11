#include "Planning.h"



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

void FastReachP2PDEMO()
{
	Vector4d AnglesA;
	AnglesA << 90, 18.6639, -120.3572, 7.4600;
	Vector4d AnglesB;
	AnglesB << -87.7251, -15.0823, -27.8304, -7.0873;
	MatrixXd AngleSequence = FastReachP2P(AnglesA, AnglesB, 35, 35, 35, 35, 25, 25, 25, 25);
	for (int i = 0; i < AngleSequence.cols(); i++)
	{
		cout << (AngleSequence.col(i)).transpose() << endl;
	}
	
}

MatrixXd GetOneJointSequence(double ThetaBegin, double ThetaEnd, double Vmaxtheta, double amaxtheta)
{
	double SampleTime = tinterval;
	double LeftTF = 0;
	double RightTF = 100 * 180 / Vmaxtheta; //一般不会到100秒都不到的

	double tf = LeftTF + (RightTF - LeftTF) / 2;
	MatrixXd AngleSequence;
	while (RightTF - LeftTF>0.5 || isempty(AngleSequence) == 1)
	{
		AngleSequence.resize(2, tf / SampleTime + 1);
		for (int i = 1; i <= tf / SampleTime + 1; i++)
		{
			double t = (i - 1)*SampleTime;
			double theta;
			int success;
			ManipulatorPlanningJointSpaceSub(ThetaBegin, ThetaEnd, tf, amaxtheta, Vmaxtheta, t, 0.1, 1, &theta, &success);
			if (success == 1)
			{
				MatrixXd timetheta(2, 1);
				timetheta(0) = t;
				timetheta(1) = theta;
				//AngleSequence = addMatrix2Matrix(AngleSequence, " ", timetheta); //速度非常慢！
				AngleSequence.block(0, (i - 1), 2, 1) = timetheta;
			}
			else
			{
				AngleSequence.resize(0, 0);
				break;
			}
		}

		if (isempty(AngleSequence) == 1)
			LeftTF = tf;
		else
			RightTF = tf;
		tf = LeftTF + (RightTF - LeftTF) / 2;
	}
	return AngleSequence;
}

MatrixXd FastReachP2P(Vector4d jointAnglesBegin, Vector4d JointAnglesEnd,double Vtheta1Max, double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta1max, double atheta2max, double atheta3max, double atheta4max)
{
	MatrixXd AngleSequence1 = GetOneJointSequence(jointAnglesBegin(0), JointAnglesEnd(0), Vtheta1Max, atheta1max);
	MatrixXd AngleSequence2 = GetOneJointSequence(jointAnglesBegin(1), JointAnglesEnd(1), Vtheta2Max, atheta2max);
	MatrixXd AngleSequence3 = GetOneJointSequence(jointAnglesBegin(2), JointAnglesEnd(2), Vtheta3Max, atheta3max);
	MatrixXd AngleSequence4 = GetOneJointSequence(jointAnglesBegin(3), JointAnglesEnd(3), Vtheta4Max, atheta4max);
	Vector4d Colums;
	Colums(0) = AngleSequence1.cols();
	Colums(1) = AngleSequence2.cols();
	Colums(2) = AngleSequence3.cols();
	Colums(3) = AngleSequence4.cols();
	int MaxColum = Colums.maxCoeff();
	MatrixXd AngleSequence1mid = addMatrix2Matrix(AngleSequence1.row(1), " ", MatrixXd::Ones(1, MaxColum - AngleSequence1.cols())*AngleSequence1(1, AngleSequence1.cols() - 1));
	MatrixXd AngleSequence2mid = addMatrix2Matrix(AngleSequence2.row(1), " ", MatrixXd::Ones(1, MaxColum - AngleSequence2.cols())*AngleSequence2(1, AngleSequence2.cols() - 1));
	MatrixXd AngleSequence3mid = addMatrix2Matrix(AngleSequence3.row(1), " ", MatrixXd::Ones(1, MaxColum - AngleSequence3.cols())*AngleSequence3(1, AngleSequence3.cols() - 1));
	MatrixXd AngleSequence4mid = addMatrix2Matrix(AngleSequence4.row(1), " ", MatrixXd::Ones(1, MaxColum - AngleSequence4.cols())*AngleSequence4(1, AngleSequence4.cols() - 1));
	MatrixXd result;
	result = addMatrix2Matrix(AngleSequence1mid, ";", AngleSequence2mid);
	result = addMatrix2Matrix(result, ";", AngleSequence3mid);
	result = addMatrix2Matrix(result, ";", AngleSequence4mid);
	MatrixXd time(1, result.cols());
	for (int i = 0; i < result.cols(); i++)
	{
		time(i) = i*tinterval;
	}
	cout << time;
	result = addMatrix2Matrix(time, ";", result);
	return result;
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

	MatrixXd Sequence = BucketTipLinearPlanning(T0, T1, 35, 35, 35, 35, 25, 25, 25, 25);
	for (int i = 0; i < Sequence.cols(); i++)
	{
		cout << Sequence.col(i).transpose() << endl;
	}
}



MatrixXd BucketTipLinearPlanning(Matrix4d BeginT, Matrix4d EndT, double Vtheta1Max,double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta1max,double atheta2max, double atheta3max, double atheta4max)
{
	int ChaZhiNum = 50;
	Matrix<double, Dynamic, Dynamic> Tsequence = ctraj_c(BeginT, EndT, ChaZhiNum, 1); //优先是1 若1不能就0  优先shortest

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
			Tsequence = ctraj_c(BeginT, EndT, ChaZhiNum, 0);
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
	//cout << jointangleSeq << endl;
	MatrixXd tmpjointangleSeq;
	tmpjointangleSeq = jointangleSeq.block(0, 0, countValidjointangleSeqRow, jointangleSeq.cols());
	jointangleSeq = tmpjointangleSeq;
	//cout << jointangleSeq << endl;
	//20200825 检验这个对不对 然后把0 和1 对调 应该是正确的 误差在0.1度

	double Leftbeishu = 0;
	double Rightbeishu = 4000; //这意味着40秒走最多8cm 一秒2mm 基本可以认定是静止的了
	double timesBEISHU = (Leftbeishu + Rightbeishu) / 2.0;
	int FLAGWHILE = 0;

	while (FLAGWHILE < 2)
	{
		MatrixXd djointangleSeq(jointangleSeq.rows() - 1, 4);
		for (int i = 0; i < jointangleSeq.rows() - 1; i++)
		{
			djointangleSeq.row(i) = (jointangleSeq.row(i + 1) - jointangleSeq.row(i)) / (timesBEISHU*tinterval);

			MatrixXd tmpChazhithis = (jointangleSeq.row(i + 1) - jointangleSeq.row(i));
			if (isempty(find(abs(tmpChazhithis), ">", 180)) == 0) // 避免出现(-180, 180]体系下的角度突变
			{
				MatrixXd foundIndex = find(abs(tmpChazhithis),">",180);
				for (int thisi = 0; thisi < foundIndex.rows()*foundIndex.cols(); thisi++)
				{
					double tmpi_1, tmpi;
					if (jointangleSeq(i + 1, foundIndex(thisi)) < 0)
					{
						tmpi_1 = 360 + jointangleSeq(i + 1, foundIndex(thisi));
						tmpi = jointangleSeq(i, foundIndex(thisi));
					}
					else if (jointangleSeq(i + 1, foundIndex(thisi)) > 0)
					{
						tmpi_1 = jointangleSeq(i + 1, foundIndex(thisi)) - 360;
						tmpi = jointangleSeq(i, foundIndex(thisi));
					}
					djointangleSeq(i, foundIndex(thisi)) = (tmpi_1 - tmpi) / (timesBEISHU*tinterval);
				}
			}
		}
			
		MatrixXd ddjointangleSeq(djointangleSeq.rows() - 1, 4);
		for (int i = 0;i<djointangleSeq.rows() - 1;i++)
			ddjointangleSeq.row(i) = (djointangleSeq.row(i + 1) - djointangleSeq.row(i)) / (timesBEISHU*tinterval);
	
		if (Rightbeishu - Leftbeishu <= 1) // 表明二分法精度到1 最后得到timesBEISHU的临界取值
		{
			FLAGWHILE = FLAGWHILE + 1;
			timesBEISHU = Rightbeishu;
			timesBEISHU = ceil(timesBEISHU);
			if (abs(djointangleSeq.col(0)).maxCoeff() > Vtheta1Max || abs(ddjointangleSeq.col(0)).maxCoeff() > atheta1max \
				|| abs(djointangleSeq.col(1)).maxCoeff() > Vtheta2Max || abs(djointangleSeq.col(2)).maxCoeff() > Vtheta3Max \
				|| abs(djointangleSeq.col(3)).maxCoeff() > Vtheta4Max || abs(ddjointangleSeq.col(1)).maxCoeff() > atheta2max \
				|| abs(ddjointangleSeq.col(2)).maxCoeff() > atheta3max || abs(ddjointangleSeq.col(3)).maxCoeff() > atheta4max)
			{
				if (FLAGWHILE == 2)
					error("设置的精度出错");
			}
		}
		else
		{
			if (abs(djointangleSeq.col(0)).maxCoeff() > Vtheta1Max || abs(ddjointangleSeq.col(0)).maxCoeff() > atheta1max \
				|| abs(djointangleSeq.col(1)).maxCoeff() > Vtheta2Max || abs(djointangleSeq.col(2)).maxCoeff() > Vtheta3Max \
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
		
		////////////////////////////////////////////////////
		MatrixXd tmpChazhithis = (jointangleSeq.row(i + 1) - jointangleSeq.row(i));
		if (isempty(find(abs(tmpChazhithis), ">", 180)) == 0) // 避免出现(-180, 180]体系下的角度突变
		{
			MatrixXd foundIndex = find(abs(tmpChazhithis), ">", 180);
			for (int thisi = 0; thisi < foundIndex.rows()*foundIndex.cols(); thisi++)
			{
				double tmpi_1, tmpi;
				if (jointangleSeq(i + 1, foundIndex(thisi)) < 0)
				{
					tmpi_1 = 360 + jointangleSeq(i + 1, foundIndex(thisi));
					tmpi = jointangleSeq(i, foundIndex(thisi));
				}
				else if (jointangleSeq(i + 1, foundIndex(thisi)) > 0)
				{
					tmpi_1 = jointangleSeq(i + 1, foundIndex(thisi)) - 360;
					tmpi = jointangleSeq(i, foundIndex(thisi));
				}
				tmp(foundIndex(thisi)) = (tmpi_1 - tmpi) / timesBEISHU;
			}
		}
		////////////////////////////////////////////////////


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

MatrixXd BucketRotateCenterLinearPlanningCPP(Matrix4d BeginT, Matrix4d EndT, double Vtheta1Max, double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta1max, double atheta2max, double atheta3max, double atheta4max,int & IsComplete)
{
	IsComplete = 1;
	int ChaZhiNum = 50;
	Matrix<double, Dynamic, Dynamic> Tsequence = ctraj_c(BeginT, EndT, ChaZhiNum, 1); //优先是1 若1不能就0  优先shortest

	Matrix<double, Dynamic, Dynamic> jointangleSeq(Tsequence.cols() / 4, 4);

	int countInDead = 0;
	int countValidjointangleSeqRow = 0;
	for (int i = 1; i <= Tsequence.cols() / 4; i++)
	{
		Matrix4d tform = Tsequence.block(0, 4 * (i - 1), 4, 4);
		int tmp;
		Vector4d jointangle = InverseKinematicsT40(tform, tmp);
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
			Tsequence = ctraj_c(BeginT, EndT, ChaZhiNum, 0);
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
	//cout << jointangleSeq << endl;
	MatrixXd tmpjointangleSeq;
	tmpjointangleSeq = jointangleSeq.block(0, 0, countValidjointangleSeqRow, jointangleSeq.cols());
	jointangleSeq = tmpjointangleSeq;
	//cout << jointangleSeq << endl;
	//20200825 检验这个对不对 然后把0 和1 对调 应该是正确的 误差在0.1度

	double Leftbeishu = 0;
	double Rightbeishu = 4000; //这意味着40秒走最多8cm 一秒2mm 基本可以认定是静止的了
	double timesBEISHU = (Leftbeishu + Rightbeishu) / 2.0;
	int FLAGWHILE = 0;

	while (FLAGWHILE < 2)
	{
		MatrixXd djointangleSeq(jointangleSeq.rows() - 1, 4);
		for (int i = 0; i < jointangleSeq.rows() - 1; i++)
		{
			djointangleSeq.row(i) = (jointangleSeq.row(i + 1) - jointangleSeq.row(i)) / (timesBEISHU*tinterval);

			MatrixXd tmpChazhithis = (jointangleSeq.row(i + 1) - jointangleSeq.row(i));
			if (isempty(find(abs(tmpChazhithis), ">", 180)) == 0) // 避免出现(-180, 180]体系下的角度突变
			{
				MatrixXd foundIndex = find(abs(tmpChazhithis), ">", 180);
				for (int thisi = 0; thisi < foundIndex.rows()*foundIndex.cols(); thisi++)
				{
					double tmpi_1, tmpi;
					if (jointangleSeq(i + 1, foundIndex(thisi)) < 0)
					{
						tmpi_1 = 360 + jointangleSeq(i + 1, foundIndex(thisi));
						tmpi = jointangleSeq(i, foundIndex(thisi));
					}
					else if (jointangleSeq(i + 1, foundIndex(thisi)) > 0)
					{
						tmpi_1 = jointangleSeq(i + 1, foundIndex(thisi)) - 360;
						tmpi = jointangleSeq(i, foundIndex(thisi));
					}
					djointangleSeq(i, foundIndex(thisi)) = (tmpi_1 - tmpi) / (timesBEISHU*tinterval);
				}
			}
		}

		MatrixXd ddjointangleSeq(djointangleSeq.rows() - 1, 4);
		for (int i = 0; i<djointangleSeq.rows() - 1; i++)
			ddjointangleSeq.row(i) = (djointangleSeq.row(i + 1) - djointangleSeq.row(i)) / (timesBEISHU*tinterval);

		if (Rightbeishu - Leftbeishu <= 1) // 表明二分法精度到1 最后得到timesBEISHU的临界取值
		{
			FLAGWHILE = FLAGWHILE + 1;
			timesBEISHU = Rightbeishu;
			timesBEISHU = ceil(timesBEISHU);
			if (abs(djointangleSeq.col(0)).maxCoeff() > Vtheta1Max || abs(ddjointangleSeq.col(0)).maxCoeff() > atheta1max \
				|| abs(djointangleSeq.col(1)).maxCoeff() > Vtheta2Max || abs(djointangleSeq.col(2)).maxCoeff() > Vtheta3Max \
				|| abs(djointangleSeq.col(3)).maxCoeff() > Vtheta4Max || abs(ddjointangleSeq.col(1)).maxCoeff() > atheta2max \
				|| abs(ddjointangleSeq.col(2)).maxCoeff() > atheta3max || abs(ddjointangleSeq.col(3)).maxCoeff() > atheta4max)
			{
				if (FLAGWHILE == 2)
					error("设置的精度出错");
			}
		}
		else
		{
			if (abs(djointangleSeq.col(0)).maxCoeff() > Vtheta1Max || abs(ddjointangleSeq.col(0)).maxCoeff() > atheta1max \
				|| abs(djointangleSeq.col(1)).maxCoeff() > Vtheta2Max || abs(djointangleSeq.col(2)).maxCoeff() > Vtheta3Max \
				|| abs(djointangleSeq.col(3)).maxCoeff() > Vtheta4Max || abs(ddjointangleSeq.col(1)).maxCoeff() > atheta2max \
				|| abs(ddjointangleSeq.col(2)).maxCoeff() > atheta3max || abs(ddjointangleSeq.col(3)).maxCoeff() > atheta4max)
				Leftbeishu = timesBEISHU;
			else
				Rightbeishu = timesBEISHU;
			timesBEISHU = Leftbeishu + (Rightbeishu - Leftbeishu) / 2.0;
		}
	}

	MatrixXd Sequence(5, (int)(jointangleSeq.rows() + (jointangleSeq.rows() - 1)*(timesBEISHU - 1))); //最终返回带时间戳的各角度
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

		////////////////////////////////////////////////////
		MatrixXd tmpChazhithis = (jointangleSeq.row(i + 1) - jointangleSeq.row(i));
		if (isempty(find(abs(tmpChazhithis), ">", 180)) == 0) // 避免出现(-180, 180]体系下的角度突变
		{
			MatrixXd foundIndex = find(abs(tmpChazhithis), ">", 180);
			for (int thisi = 0; thisi < foundIndex.rows()*foundIndex.cols(); thisi++)
			{
				double tmpi_1, tmpi;
				if (jointangleSeq(i + 1, foundIndex(thisi)) < 0)
				{
					tmpi_1 = 360 + jointangleSeq(i + 1, foundIndex(thisi));
					tmpi = jointangleSeq(i, foundIndex(thisi));
				}
				else if (jointangleSeq(i + 1, foundIndex(thisi)) > 0)
				{
					tmpi_1 = jointangleSeq(i + 1, foundIndex(thisi)) - 360;
					tmpi = jointangleSeq(i, foundIndex(thisi));
				}
				tmp(foundIndex(thisi)) = (tmpi_1 - tmpi) / timesBEISHU;
			}
		}
		////////////////////////////////////////////////////


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

double mean(vector <double> data)
{
	double sum = 0;
	for (int i = 0; i < data.size(); i++)
	{
		sum = sum + data[i];
	}
	double avg = sum / data.size();
	return avg;
}

MatrixXd CarryAndReleaseTaskCartesianSpaceSub(MatrixXd StableRange,Vector4d AnglesBegin, Vector4d AnglesEnd,double Vtheta1Max, double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta1max, double atheta2max, double atheta3max, double atheta4max, int & NeedModified)
//StableRange是行向量
{
	NeedModified = 0;
	MatrixXd AngleSequence;
	Matrix4d Matrixbegin,tmp, Matrixend;

	ForwardKinematics(AnglesBegin, Matrixbegin, tmp);
	ForwardKinematics(AnglesEnd, Matrixend, tmp);

	MatrixXd DirectionVector;
	DirectionVector = Matrixend.block(0, 3, 3, 1) - Matrixbegin.block(0, 3, 3, 1);
	DirectionVector.normalize();

	Matrix4d Matrixbegin1;


	int FlagHaveSequence0 = 1;
	/////////////////////////////////////////////////////////////////////
	//T0->T1
	double Angle = GetAngleOfBucketWithGround(AnglesBegin(0), AnglesBegin(1), AnglesBegin(2), AnglesBegin(3));
	if (Angle<0)
		Angle = 360 + Angle;
	MatrixXd StableRangetmp = StableRange;
	for (int i = 0; i < size(StableRangetmp, 2); i++)
	{
		if (StableRangetmp(i)<0)
			StableRangetmp(i) = 360 + StableRangetmp(i);
	}
	if (isempty(GetIntersection(StableRangetmp, Angle)) == 0) // 说明一开始就已经保证稳定了 并不需要转化到稳定区域
	{
		Matrixbegin1 = Matrixbegin;
		FlagHaveSequence0 = 0;
	}
	else //否则需要转化到稳定区域
	{
		MatrixXd jointAngle = AnglesBegin;
		int YES;
		cout << jointAngle << endl;
		cout << StableRange << endl;
		vector <MatrixXd> StableTheta4 = groundAngleRangeTOtheta4Range(jointAngle(0), jointAngle(1), jointAngle(2), StableRange, YES);
		if (YES == 0)
			error("此时无法按照先搬运东西再卸载释放东西的顺序完成任务");
		vector <double> avgstore;
		for (int i = 0; i < StableTheta4.size(); i++)
			avgstore.push_back(StableTheta4[i].mean());
		for (int i = 0; i < avgstore.size(); i++)
			if (avgstore[i]<0)
				avgstore[i] = 360 + avgstore[i];
		double theta4selected = mean(avgstore);
		theta4selected = legalizAnger(theta4selected);

		Matrix4d tmp;
		Vector4d jointAnglestmp;
		jointAnglestmp(0) = jointAngle(0);
		jointAnglestmp(1) = jointAngle(1);
		jointAnglestmp(2) = jointAngle(2);
		jointAnglestmp(3) = theta4selected;
		ForwardKinematics(jointAnglestmp, Matrixbegin1, tmp);
		cout << Matrixbegin1 << endl;
		if (IsAnglesInLimitRange(jointAnglestmp) == 0)
			error("程序逻辑出错");
	}
	/////////////////////////////////////////////////////////////////////




	/////////////////////////////////////////////////////////////////////
	//T1->T2
	//到这一步时默认begin能够让铲斗保持稳定
	Matrix4d RightUpper = Matrixbegin1;
	if (DirectionVector(2)>0)
		RightUpper(2, 3) = 3000; //二分法求垂直上升最大能上升的临界值
	else if (DirectionVector(2)<0)
		RightUpper(2, 3) = -3000;
	else
		RightUpper = Matrixbegin1; //等于0时不进行垂直方向的行走

	Matrix4d LeftUpper = Matrixbegin1;
	Matrix4d Mid = Matrixbegin1;
	Mid.block(0, 3, 3, 1) = (LeftUpper.block(0, 3, 3, 1) + RightUpper.block(0, 3, 3, 1)) / 2;
	while ((RightUpper.block(0, 3, 3, 1) - LeftUpper.block(0, 3, 3, 1)).norm() > 5)
	{
		int Valid,YES;
		Vector4d jointAngle = InverseKinematicsT40(Mid, Valid);
		if (Valid == 1)
			groundAngleRangeTOtheta4Range(jointAngle(0), jointAngle(1), jointAngle(2), StableRange, YES);
		else
			YES = 0;
		if (Valid == 0 || YES == 0)
			RightUpper = Mid;
		else
			LeftUpper = Mid;
		Mid.block(0,3,3,1)= (LeftUpper.block(0, 3, 3, 1) + RightUpper.block(0, 3, 3, 1)) / 2;
		cout << Mid.block(0, 3, 3, 1) << endl;
	}
	Matrix4d Mid1 = LeftUpper;

	MatrixXd tmp1_2(1, 2);
	tmp1_2(0) = Mid1(2, 3);
	tmp1_2(1) = Matrixbegin1(2, 3);
	if (isempty(GetIntersection(Matrixend(2, 3), tmp1_2)) == 0)
		Mid1(2, 3) = Matrixend(2, 3);
	else
		;
		//说明垂直方向上不能保持不漏的过去

	int Valid,YES;
	Vector4d jointAngle = InverseKinematicsT40(Mid1, Valid);
	vector <MatrixXd> StableTheta4;
	if (Valid == 1)
	{
		StableTheta4 = groundAngleRangeTOtheta4Range(jointAngle(0), jointAngle(1), jointAngle(2), StableRange, YES);
		if (YES == 0)
			error("程序逻辑出错");
		vector <double> avgstore;
		for (int i = 0; i < StableTheta4.size(); i++)
			avgstore.push_back(StableTheta4[i].mean());
		for (int i = 0; i < avgstore.size(); i++)
			if (avgstore[i] < 0)
				avgstore[i] = 360 + avgstore[i];
		double theta4selected = mean(avgstore);
		theta4selected = legalizAnger(theta4selected);

		Matrix4d tmp;
		Vector4d jointAnglestmp;
		jointAnglestmp(0) = jointAngle(0);
		jointAnglestmp(1) = jointAngle(1);
		jointAnglestmp(2) = jointAngle(2);
		jointAnglestmp(3) = theta4selected;
		ForwardKinematics(jointAnglestmp, Mid1, tmp);
		if (IsAnglesInLimitRange(jointAnglestmp) == 0)
			error("程序逻辑出错");
	}
	else
	{
		NeedModified = 1; //说明初始位置到了一个非常靠近边缘的位置，使得垂直方向上可行域非凸, 需要修改起始位置 使得垂直方向上可行域为凸集
		return AngleSequence;
	}
	///////////////////////////////////////////////////////////////////
		



	///////////////////////////////////////////////////////////////////
	//T2->T3
	Vector4d jointAngleCurrent = InverseKinematicsT40(Mid1, Valid);
	MatrixXd AngleSequence1_5;
	if (Valid == 1)
	{
		AngleSequence1_5 = GetOneJointSequence(jointAngleCurrent(0), AnglesEnd(0), Vtheta1Max, atheta1max);
	}
	else
	{
		NeedModified = 1; //说明初始位置到了一个非常靠近边缘的位置，使得垂直方向上可行域非凸, 需要修改起始位置 使得垂直方向上可行域为凸集
		return AngleSequence;
	}
	AngleSequence1_5 = addMatrix2Matrix(AngleSequence1_5, ";", jointAngleCurrent(1)*ones(1, size(AngleSequence1_5, 2)));
	AngleSequence1_5 = addMatrix2Matrix(AngleSequence1_5, ";", jointAngleCurrent(2)*ones(1, size(AngleSequence1_5, 2)));
	AngleSequence1_5 = addMatrix2Matrix(AngleSequence1_5, ";", jointAngleCurrent(3)*ones(1, size(AngleSequence1_5, 2)));

	Matrix4d Mid1_5;
	ForwardKinematics(AngleSequence1_5.block(1, size(AngleSequence1_5, 2) - 1, 4, 1), Mid1_5, tmp);
	///////////////////////////////////////////////////////////////////


	
	//////////////////////////////////////////////////////////////////
	//T3->T4
	RightUpper = Mid1_5;

	DirectionVector = Matrixend.block(0, 3, 3, 1) - Mid1_5.block(0, 3, 3, 1);
	DirectionVector.normalize();

	RightUpper.block(0,3,2,1) = RightUpper.block(0, 3, 2, 1) + 3000 * DirectionVector.block(0,0,2,1) / DirectionVector.block(0, 0, 2, 1).norm();
	LeftUpper = Mid1_5;
	Mid = Mid1_5;
	Mid.block(0, 3, 3, 1) = (LeftUpper.block(0, 3, 3, 1) + RightUpper.block(0, 3, 3, 1)) / 2;

	while ((RightUpper.block(0, 3, 3, 1) - LeftUpper.block(0, 3, 3, 1)).norm() > 5)
	{
		int Valid, YES;
		Vector4d jointAngle = InverseKinematicsT40(Mid, Valid);
		if (Valid == 1)
			groundAngleRangeTOtheta4Range(jointAngle(0), jointAngle(1), jointAngle(2), StableRange, YES);
		else
			YES = 0;
		if (Valid == 0 || YES == 0)
			RightUpper = Mid;
		else
			LeftUpper = Mid;
		Mid.block(0, 3, 3, 1) = (LeftUpper.block(0, 3, 3, 1) + RightUpper.block(0, 3, 3, 1)) / 2;
	}
	Matrix4d Mid2 = LeftUpper;

	if (dot((Matrixend.block(0, 3, 2, 1) - Mid2.block(0, 3, 2, 1)), (Matrixend.block(0, 3, 2, 1) - Mid1_5.block(0, 3, 2, 1))) < 0)
	{
		Mid2.block(0,3,2,1) = Matrixend.block(0, 3, 2, 1);
	}
	else
	{
		//说明水平向目标的方向上不能保持不漏的过去
	}

	jointAngle = InverseKinematicsT40(Mid2, Valid);
	if (Valid == 1)
	{
		StableTheta4 = groundAngleRangeTOtheta4Range(jointAngle(0), jointAngle(1), jointAngle(2), StableRange, YES);
		if (YES == 0)
			error("程序逻辑出错");
		vector <double> avgstore;
		for (int i = 0; i < StableTheta4.size(); i++)
			avgstore.push_back(StableTheta4[i].mean());
		for (int i = 0; i < avgstore.size(); i++)
			if (avgstore[i] < 0)
				avgstore[i] = 360 + avgstore[i];
		double theta4selected = mean(avgstore);
		theta4selected = legalizAnger(theta4selected);

		Matrix4d tmp;
		Vector4d jointAnglestmp;
		jointAnglestmp(0) = jointAngle(0);
		jointAnglestmp(1) = jointAngle(1);
		jointAnglestmp(2) = jointAngle(2);
		jointAnglestmp(3) = theta4selected;
		ForwardKinematics(jointAnglestmp, Mid2, tmp);
		if (IsAnglesInLimitRange(jointAnglestmp) == 0)
			error("程序逻辑出错");
	}
	else
	{
		NeedModified = 2; //说明初始位置到了一个非常靠近边缘的位置，使得水平方向上可行域非凸 出现这种情况时暂时还无法解决
		return AngleSequence;
	}
	//////////////////////////////////////////////////////////////////




	//////////////////////////////////////////////////////////////////
	MatrixXd AngleSequence0, AngleSequence1, AngleSequence2;
	int IsComplete;
	if (FlagHaveSequence0 == 1)
	{
		AngleSequence0 = BucketRotateCenterLinearPlanningCPP(Matrixbegin, Matrixbegin1, Vtheta1Max, Vtheta2Max, Vtheta3Max, Vtheta4Max, atheta1max, atheta2max, atheta3max, atheta4max, IsComplete);
		if (IsComplete == 0)
		{
			NeedModified = 3; //说明Matrixbegin Matrixbegin1 之间的位置连线的可行域非凸 这种情况如果上边两个不能反解的情况解决的话一般是不会出现的 如果出现 现在暂时还不能解决
			return AngleSequence;
		}
	}
	else
	{
		AngleSequence0 = addMatrix2Matrix(0, ";", AnglesBegin.transpose());
	}
	cout << Matrixbegin1 << endl;
	AngleSequence1 = BucketRotateCenterLinearPlanningCPP(Matrixbegin1, Mid1, Vtheta1Max, Vtheta2Max, Vtheta3Max, Vtheta4Max, atheta1max, atheta2max, atheta3max, atheta4max, IsComplete);
	if (IsComplete == 0)
	{
		NeedModified = 1; //说明Matrixbegin1 Mid1 之间的位置连线的可行域非凸
		return AngleSequence;
	}

	AngleSequence2 = BucketRotateCenterLinearPlanningCPP(Mid1_5, Mid2, Vtheta1Max, Vtheta2Max, Vtheta3Max, Vtheta4Max, atheta1max, atheta2max, atheta3max, atheta4max, IsComplete);
	if (IsComplete == 0)
	{
		NeedModified = 1; //说明 Mid1_5 Mid2 之间的位置连线的可行域非凸或者无法插值 这种情况如果上边两个不能反解的情况解决的话一般是不会出现的 如果出现 现在暂时还不能解决 似乎可以按照错误码为1进行处理 原理还不知道
			//         NeedModified = 5;
		return AngleSequence;
	}

	MatrixXd theta1tmp = GetOneJointSequence(AngleSequence1(1, 0), AngleSequence2(1, size(AngleSequence2,2)-1), Vtheta1Max, atheta1max);
	MatrixXd tmpxd = theta1tmp.row(1);
	theta1tmp = tmpxd;

	MatrixXd theta234tmp;
	theta234tmp = addMatrix2Matrix(ExtractRows(AngleSequence1, 2, 4)," ", ExtractRows(AngleSequence2, 2, 4));

	int FlagMode = 0;
	double ChaZhi;
	if (size(theta1tmp, 2) > size(theta234tmp, 2))
	{
		FlagMode = 1;
		ChaZhi = size(theta1tmp, 2) - size(theta234tmp, 2);
		MatrixXd added = ExtractRows(theta234tmp, 0, theta234tmp.rows() - 1);
		MatrixXd addedsum(added.rows(), (int)ChaZhi);
		VectorXd addvector = added.col(added.cols() - 1);
		for (int i = 0; i < ChaZhi; i++)
		{
			addedsum.block(0,i, added.rows(),1) = addvector;
		}
		theta234tmp = addMatrix2Matrix(theta234tmp, " ", addedsum);
	}
	else if (size(theta1tmp, 2) < size(theta234tmp, 2))
	{
		FlagMode = 2;
		MatrixXd added = ExtractRows(theta1tmp, 0, theta1tmp.rows() - 1);
		theta1tmp = addMatrix2Matrix(theta1tmp, " ", added.col(added.cols() - 1));
	}
	
	MatrixXd theta1234 = addMatrix2Matrix(theta1tmp, ";", theta234tmp);
	MatrixXd AngleSequence3 = BucketRotateCenterLinearPlanningCPP(Mid2, Matrixend, Vtheta1Max, Vtheta2Max, Vtheta3Max, Vtheta4Max, atheta1max, atheta2max, atheta3max, atheta4max, IsComplete);
	if (IsComplete == 0)
		// NeedModified = 6; %说明 Mid2 Matrixend 之间的位置连线的可行域非凸或者warning提到的原因 这种情况如果上边两个不能反解的情况解决的话一般是不会出现的 如果出现 现在暂时还不能解决
		//其实也是可以妥协的，因为已经到了最后一个阶段了，不完整的也是可以的
		//         NeedModified = 0;
		//         return;
		warning("最后一个阶段没有完整到达，最后的角度不是目标的角度");

	if (FlagMode == 1)
	{
		double splitNum = 5;
		if ((int)(ChaZhi / splitNum) != 0)
		{
			if (size(AngleSequence3, 2) >= (int)(ChaZhi / splitNum) + 1)
			{
				theta1234.block(1, size(theta1234, 2) - (int)(ChaZhi / splitNum) - 1, 3, (int)(ChaZhi / splitNum) + 1) = AngleSequence3.block(2, 0, 3, (int)(ChaZhi / splitNum) + 1);
				MatrixXd emptymat;
				emptymat.resize(0, 0);
				AngleSequence3 = ExtractCols(AngleSequence3, (int)(ChaZhi / splitNum), size(AngleSequence3,2) - 1);
				//AngleSequence3.block(0, 0, AngleSequence3.rows(), (int)(ChaZhi / splitNum)).resize(0, 0);
			}
		}
	}

	AngleSequence = addMatrix2Matrix(ExtractRows(AngleSequence0, 1, 4), " ", theta1234);
	AngleSequence = addMatrix2Matrix(AngleSequence, " ", ExtractRows(AngleSequence3, 1, 4));
	MatrixXd times(1, size(AngleSequence, 2));
	for (int i = 0; i < times.cols(); i++)
		times(i) = i*tinterval;
	AngleSequence = addMatrix2Matrix(times, ";", AngleSequence);

	//////////////////////////////////////////////////////////////////
	 //= [jointAngleCurrent(2)*ones(1, size(AngleSequence1_5, 2)); jointAngleCurrent(3)*ones(1, size(AngleSequence1_5, 2)); jointAngleCurrent(4)*ones(1, size(AngleSequence1_5, 2))];

	return AngleSequence;
}

MatrixXd CarryAndReleaseTaskCartesianSpace(MatrixXd StableRange, Vector4d & AnglesBegin, Vector4d & AnglesEnd, double Vtheta1Max, double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta1max, double atheta2max, double atheta3max, double atheta4max)
{
	int NeedModified;
	MatrixXd AngleSequence;
	AngleSequence = CarryAndReleaseTaskCartesianSpaceSub(StableRange, AnglesBegin, AnglesEnd, Vtheta1Max, Vtheta2Max, Vtheta3Max, Vtheta4Max, atheta1max, atheta2max, atheta3max, atheta4max, NeedModified);
	if (NeedModified == 0)
	{
		//说明一切正常
	}
	else if (NeedModified == 1)
	{
		Matrix4d BeginPoint,mat4dtmp, EndPoint;
		ForwardKinematics(AnglesBegin, BeginPoint, mat4dtmp);
		Vector4d vec4dtmp;
		vec4dtmp(0) = AnglesBegin(0);
		vec4dtmp(1) = AnglesEnd(1);
		vec4dtmp(2) = AnglesEnd(2);
		vec4dtmp(3) = AnglesEnd(3);
		ForwardKinematics(vec4dtmp, EndPoint, mat4dtmp);
		MatrixXd MovDirection = EndPoint.block(0, 3, 2, 1) - BeginPoint.block(0, 3, 2, 1);
		MovDirection.normalize();
		
		double ScaleTimes = 5;
		int Valid = 0;
		NeedModified = 1;
		MatrixXd Origin = BeginPoint.block(0, 3, 2, 1);
		int Count = 0;

		while (Valid == 0 || NeedModified == 1)
		{
			Count++;
			if (Count == 6)
				error("无法处理这种情况");
			BeginPoint.block(0, 3, 2, 1) = Origin + MovDirection*ScaleTimes;
			Vector4d jointAngle = InverseKinematicsT40(BeginPoint, Valid);
			if (Valid == 1)
			{
				AnglesBegin = jointAngle;
				AngleSequence = CarryAndReleaseTaskCartesianSpaceSub(StableRange, AnglesBegin, AnglesEnd, Vtheta1Max, Vtheta2Max, Vtheta3Max, Vtheta4Max, atheta1max, atheta2max, atheta3max, atheta4max, NeedModified);
			}
			ScaleTimes = ScaleTimes*1.1;
		}
	}
	else
	{
		cout << "错误码：" << NeedModified << endl;
		error("目前暂时无法处理这样的情况");
	}
	AnglesEnd = AngleSequence.block(1, size(AngleSequence, 2) - 1, 4, 1);
	return AngleSequence;
}

void WholeSystemDEMO()
//整个系统实现的demo
{
	///////////////////////////////////////////////////
	Matrix4d BeginT, EndT;
	BeginT << 0.0255, 0.0304, -0.9992, 13.7000,
		-0.6423, -0.7654, -0.0397, -690.0000,
		-0.7660, 0.6428, 0.0000, -300.0000,
		0, 0, 0, 1;
	EndT << -0.0658, 0.0380, -0.9971, 13.7000,
		0.8635, -0.4986, -0.0760, -360.0000,
		-0.5000, -0.8660, 0.0000, -300.0000,
		0, 0, 0, 1;

	MatrixXd Seq1 = BucketTipLinearPlanning(BeginT, EndT, 35, 35, 35, 35, 25, 25, 25, 25);
	///////////////////////////////////////////////////


	///////////////////////////////////////////////////
	MatrixXd StableRange(1, 2);
	StableRange(0) = 160;
	StableRange(1) = -160;
	Vector4d AnglesBegin;
	AnglesBegin << -85.6413, -10.6827, -78.5222, -60.7951;
	Vector4d AnglesEnd;
	AnglesEnd << 90.0000, 18.6639, -120.3572, 7.4600;
	MatrixXd Seq2 = CarryAndReleaseTaskCartesianSpace(StableRange, AnglesBegin, AnglesEnd, 35, 35, 35, 35, 25, 25, 25, 25);
	cout << AnglesBegin << endl;
	cout << AnglesEnd << endl;
	///////////////////////////////////////////////////


	///////////////////////////////////////////////////
	Vector4d AnglesA;
	AnglesA << 90, 18.6639, -120.3572, 7.4600;
	Vector4d AnglesB;
	AnglesB << -87.7251, -15.0823, -27.8304, -7.0873;
	MatrixXd Seq3 = FastReachP2P(AnglesA, AnglesB, 35, 35, 35, 35, 25, 25, 25, 25);
	///////////////////////////////////////////////////

}