#ifndef PLANNING_H
#define PLANNING_H

#include "main.h"

#include "FIKinematic.h"



class MatlabDouble
{
public:
	double value;
	MatlabDouble(double num)
	{
		MatlabDouble::value = num ;
	}
	MatlabDouble(int num)
	{
		MatlabDouble::value = num;
	}
	MatlabDouble()
	{
		double valuetmp;
		MatlabDouble::value = valuetmp;
	}
	MatlabDouble operator ^(MatlabDouble zhishu)
	{
		double result;
		result = pow(this->value, zhishu.value);
		MatlabDouble tmp;
		tmp.value = result;
		return tmp;
	}
	MatlabDouble operator + (MatlabDouble num)
	{
		double result = this->value + num.value;
		MatlabDouble tmp;
		tmp.value = result;
		return tmp;
	}
	MatlabDouble operator - (MatlabDouble num)
	{
		double result = this->value - num.value;
		MatlabDouble tmp;
		tmp.value = result;
		return tmp;
	}
	MatlabDouble operator * (MatlabDouble num)
	{
		double result = this->value * num.value;
		MatlabDouble tmp;
		tmp.value = result;
		return tmp;
	}
	MatlabDouble operator / (MatlabDouble num)
	{
		double result = this->value / num.value;
		MatlabDouble tmp;
		tmp.value = result;
		return tmp;
	}
	MatlabDouble operator = (MatlabDouble num)
	{
		MatlabDouble tmp;
		tmp.value = num.value;
		return tmp;
	}
	MatlabDouble operator = (double num)
	{
		MatlabDouble tmp;
		tmp.value = num;
		return tmp;
	}
	
	int operator < (MatlabDouble num)
	{
		if (this->value < num.value)
			return 1;
		else
			return 0;
	}
	int operator == (MatlabDouble num)
	{
		if (this->value == num.value)
			return 1;
		else
			return 0;
	}
	int operator >= (MatlabDouble num)
	{
		if (this->value >= num.value)
			return 1;
		else
			return 0;
	}
	int operator <= (MatlabDouble num)
	{
		if (this->value <= num.value)
			return 1;
		else
			return 0;
	}
	int operator > (MatlabDouble num)
	{
		if (this->value > num.value)
			return 1;
		else
			return 0;
	}
};

//
//void ManipulatorPlanningJointSpace(MatlabDouble theta0, MatlabDouble thetaf, MatlabDouble tf, MatlabDouble AMAX, MatlabDouble VMAX, MatlabDouble SampleTime, MatlabDouble t, MatlabDouble & Pointthistime,int & success) //默认初始末点速度为0，加速度为0;
//{
////%-180,180 角度体系
//	
//    success = 0;
//    int flagNeedXG = 0;
//	if (theta0*thetaf < 0)
//	{
//		if (theta0 > 0)
//		{
//			if (theta0 - thetaf > 180)
//			{
//				thetaf = thetaf + 180 + 180 ;
//				flagNeedXG = 1;
//			}
//		}
//		else
//		{
//			if (thetaf - theta0 > 180)
//			{
//				theta0 = theta0 + 180 + 180  ;
//				flagNeedXG = 1;
//			}
//		}
//	}
//        
//    //PointsSequence = [];
//    double k = 0.1; //%k=ta/tb,取值范围为[0,0.5]
//	if (theta0 == thetaf)
//	{
//		Pointthistime = theta0;
//		return;
//	}
//	double a_LowerBound, a_Uppertemp,a_UpperBound;
//	if (theta0 < thetaf)
//	{
//		a_LowerBound = 4*(theta0-thetaf)/( (k-1)*pow(tf,2) );
//        a_Uppertemp = pow(VMAX,2) / ( (1-k)*(theta0-thetaf+tf*VMAX) );
//        if (a_Uppertemp<a_LowerBound)
//		{
//			cout<<"1VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX"<<endl;
//			return;
//		}
//            
//        
//		if (a_Uppertemp >= AMAX)
//		{
//			if (AMAX < a_LowerBound)
//			{
//				cout<<"加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX"<<endl;
//				return;
//			}
//            a_UpperBound = AMAX;
//		}
//		else
//		{
//			if (a_Uppertemp < a_LowerBound)
//			{
//				cout << "2VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX" << endl;
//				return;
//			}
//            a_UpperBound = a_Uppertemp;
//		}
//            
//        double amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
//        double tb = -(pow((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*pow(tf,2) - amax*k*pow(tf,2))),(1/2)) - amax*tf + amax*k*tf)/(2*(amax - amax*k));
//        double ta = k*tb;
//        for i=1:tf/SampleTime+1
//            PointsSequence(1,i) = (i-1)*SampleTime;
//            t = PointsSequence(1,i);
//            if t>=0&&t<ta
//                PointsSequence(2,i) = (amax*t^3)/(6*k*tb) + theta0;
//                continue;
//            end
//            if t>=ta && t<tb-ta
//                PointsSequence(2,i) = theta0 + (amax*t*(t - k*tb))/2 + (amax*k^2*tb^2)/6;
//                continue;
//            end
//            if t>=tb-ta && t<tb
//                PointsSequence(2,i) = theta0 + (amax*t^2)/(2*k) + (amax*k^2*tb^2)/6 + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*t*k^2 - 6*t*k + 3*t))/(6*k) - (amax*t^3)/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//            if t>=tb && t<tf-tb
//                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 + amax*(tb - k*tb)*(t - tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//            if t>=tf-tb && t<tf-tb+ta
//                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k) - (amax*(t + tb - tf)*(6*k^2*tb^2 - 6*k*tb^2 + t^2 + 2*t*tb - 2*t*tf + tb^2 - 2*tb*tf + tf^2))/(6*k*tb);
//                continue;
//            end
//            if t>=tf-tb+ta && t<tf-ta
//                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - (amax*(t + tb - tf - k*tb)*(t - tb - tf + 2*k*tb))/2 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//            if t>=tf-ta && t<=tf
//                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/3 - amax*(2*tb - tf)*(tb - k*tb) + amax*(tb - k*tb)*(tb - 2*k*tb) - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) + (amax*(t^3 - 3*t^2*tf + 3*t*tf^2 - tf^3))/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//        end
//	}
//	else
//	{
//		thetatemp = thetaf;
//        thetaf = theta0;
//        theta0 = thetatemp;
//        a_LowerBound = 4*(theta0-thetaf)/( (k-1)*tf^2 );
//        a_Uppertemp = VMAX^2 / ( (1-k)*(theta0-thetaf+tf*VMAX) );
//        if a_Uppertemp<a_LowerBound
//            disp('1VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX');
//            return;
//        end
//        if a_Uppertemp>=AMAX
//            if AMAX<a_LowerBound
//                disp('加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX');
//                return;
//            end
//            a_UpperBound = AMAX;
//        else
//            if a_Uppertemp<a_LowerBound
//                disp('2VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX');
//                return;
//            end
//            a_UpperBound = a_Uppertemp;
//        end
//        amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
//        tb = -((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*tf^2 - amax*k*tf^2))^(1/2) - amax*tf + amax*k*tf)/(2*(amax - amax*k));
//        ta = k*tb;
//        for i=1:tf/SampleTime+1
//            PointsSequence(1,i) = (i-1)*SampleTime;
//            t = PointsSequence(1,i);
//            t = 2*(tf/2)-t;
//            if t>=0&&t<ta
//                PointsSequence(2,i) = (amax*t^3)/(6*k*tb) + theta0;
//                continue;
//            end
//            if t>=ta && t<tb-ta
//                PointsSequence(2,i) = theta0 + (amax*t*(t - k*tb))/2 + (amax*k^2*tb^2)/6;
//                continue;
//            end
//            if t>=tb-ta && t<tb
//                PointsSequence(2,i) = theta0 + (amax*t^2)/(2*k) + (amax*k^2*tb^2)/6 + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*t*k^2 - 6*t*k + 3*t))/(6*k) - (amax*t^3)/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//            if t>=tb && t<tf-tb
//                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 + amax*(tb - k*tb)*(t - tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//            if t>=tf-tb && t<tf-tb+ta
//                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k) - (amax*(t + tb - tf)*(6*k^2*tb^2 - 6*k*tb^2 + t^2 + 2*t*tb - 2*t*tf + tb^2 - 2*tb*tf + tf^2))/(6*k*tb);
//                continue;
//            end
//            if t>=tf-tb+ta && t<tf-ta
//                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - (amax*(t + tb - tf - k*tb)*(t - tb - tf + 2*k*tb))/2 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//            if t>=tf-ta && t<=tf
//                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/3 - amax*(2*tb - tf)*(tb - k*tb) + amax*(tb - k*tb)*(tb - 2*k*tb) - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) + (amax*(t^3 - 3*t^2*tf + 3*t*tf^2 - tf^3))/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
//                continue;
//            end
//        end
//	}
//    if flagNeedXG==1
//        for i =1:size(PointsSequence,2)
//            if PointsSequence(2,i)>180
//                PointsSequence(2,i) = -180 + (PointsSequence(2,i)-180);
//            end
//        end
//    end
//    success = 1;
//}

void ManipulatorPlanningJointSpaceSub(double theta0, double thetaf, double tf, double AMAX,
	double VMAX, double t, double k, double k_amax, double *PointCurrentTime,
	int *success);
void ManipulatorPlanningJointSpaceDEMO();

Matrix<double, Dynamic, Dynamic> ctraj_c(Matrix4d T0, Matrix4d T1, double t_input, int Isshortest);
void ctrajDEMO();

Matrix<double, Dynamic, Dynamic> BucketTipLinearPlanning(Matrix4d BeginT, Matrix4d EndT, double Vtheta2Max, double Vtheta3Max, double Vtheta4Max, double atheta2max, double atheta3max, double atheta4max);
void BucketTipLinearPlanningDEMO();


#endif
