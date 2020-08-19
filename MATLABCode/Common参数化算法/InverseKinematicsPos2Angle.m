function [jointAngle,reliable] = InverseKinematicsPos2Angle(position)%输入为铲斗旋转中心位置的列向量，输出为前三个角的角度 为横向量 p40
    reliable = 1;
    GlobalDeclarationCommon
    ZERO = ZeroDefine;
    M_PI = pi;
%     nx, ny, nz;
%     ox, oy, oz;
% 	ax, ay, az;
% 	px, py, pz;
%     m[4], a[4], theta[4], d[4], tool;
% 	mTempAngleOne[2];

%     m(1) = 0.0*M_PI / 180.0;
% 	m(2) = 90.0*M_PI / 180.0;
% 	m(3) = 0.0*M_PI / 180.0;
% 	m(4) = 0.0*M_PI / 180.0;
    m(1) = m0;
    m(2) = m1;
    m(3) = m2;
    m(4) = m3;
    
%     a(1) = 0.0;
% 	a(2) = 12.0;
% 	a(3) = 460.0;
% 	a(4) = 210.9;
    a(1) = a0;
    a(2) = a1;
    a(3) = a2;
    a(4) = a3;
    
% 	tool = 123.5;
    tool = tool;

% 	d(1) = 57.9;
% 	d(2) = 13.7;%//13.7
% 	d(3) = 0.0;
% 	d(4) = 0.0;
    d(1) = d1;
	d(2) = d2;%//13.7
	d(3) = d3;
	d(4) = d4;
    
%     nx = position(1,1);
% 	ny = position(2,1);
% 	nz = position(3,1);
% 
% 	ox = position(1,2);
% 	oy = position(2,2);
% 	oz = position(3,2);
% 
% 	ax = position(1,3);
% 	ay = position(2,3);
% 	az = position(3,3);

	px = position(1);
	py = position(2);
	pz = position(3);

    mTempAngleOne(1) = mathAtan2(py, px) - mathAtan2(-d(2), (px*px + py*py - d(2) * d(2))^(1/2));
	mTempAngleOne(2) = mathAtan2(py, px) - mathAtan2(-d(2), -(px*px + py*py - d(2) * d(2))^(1/2));
    
	jointAngle(1) = mTempAngleOne(1);
	Mtemp1 = (pow(cos(jointAngle(1))*px + sin(jointAngle(1))*py - a(2), 2) + pow(pz - d(1), 2) - pow(a(3), 2) - pow(a(4), 2)) / (2 * a(3) * a(4));
    
    if abs(Mtemp1)>1
        reliable = 0;
        return;
        error('似乎超出了工作空间，结果不可信');
    end
	jointAngle(3) = -abs(acos(Mtemp1));


%     double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle(1)) + py*sin(jointAngle(1)) - a(2);
	tempn = pz - d(1);
    if abs(tempm*tempm + tempn*tempn) <= ZERO
        disp("error!");
        reliable = 0;
        return;
        error('error'); %zk20200730
        return ;
    else
        tempTwo1 = ((a(3) + a(4) * cos(jointAngle(3)))*tempn - a(4) * sin(jointAngle(3))*tempm) / (tempm*tempm + tempn*tempn);
    end
	
    if abs(tempTwo1)>1
        reliable = 0;
        return;
        error('似乎超出了工作空间，结果不可信');
    end
	jointAngle(2) = asin(tempTwo1);

% 	double tempfour1, tempfour2;
% 	tempfour1 = -(cos(jointAngle(1))*cos(jointAngle(2) + jointAngle(3))*ox + sin(jointAngle(1))*cos(jointAngle(2) + jointAngle(3))*oy + sin(jointAngle(2) + jointAngle(3))*oz);
% 	tempfour2 = -cos(jointAngle(1))*sin(jointAngle(2) + jointAngle(3))*ox - sin(jointAngle(1))*sin(jointAngle(2) + jointAngle(3))*oy + cos(jointAngle(2) + jointAngle(3))*oz;
% 	jointAngle(4) = mathAtan2(tempfour1, tempfour2);

    for i=1:3
        jointAngle(i) = jointAngle(i) * 180.0 / M_PI;
    end
end
