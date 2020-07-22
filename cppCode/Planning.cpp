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
	//AMAX 是整个过程中加速度的最大值
	//VMAX 是整个过程中速度的最大值
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

