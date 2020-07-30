clc
clear
close all

ParametersConfigCommon

PointA = [-32.1682 -588.1967  218.8374];
PointB = [-30.6171 -538.7770  260.6377];







PointsSequence = LinearDigPlanningRandomLine(PointA,PointB,-100,30,150,150,150,15,25,25);

figure
plot(PointsSequence(1,:),PointsSequence(2,:),'r-');
hold on 
plot(PointsSequence(1,:),PointsSequence(3,:),'b-');
hold on
plot(PointsSequence(1,:),PointsSequence(4,:),'y-');

cartesianSequence = [];
for i=1:size(PointsSequence,2)
    [position1,position2] = ForwardKinematics([PointsSequence(2:4,i);0]);
    cartesianSequence = [cartesianSequence [PointsSequence(1,i);position1(1:3,4)]];
end
figure
plot(cartesianSequence(1,:),cartesianSequence(2,:),'r-');
hold on
plot(cartesianSequence(1,:),cartesianSequence(3,:),'b-');
hold on
plot(cartesianSequence(1,:),cartesianSequence(4,:),'y-');
hold on

figure
for i=1:5:size(cartesianSequence,2)
    plot3(cartesianSequence(2,i),cartesianSequence(3,i),cartesianSequence(4,i),'.');
    hold on
    pause(0.1);
end

% kd_k2_divide_d_k3_x1NEx0(1,1,1,0,3,2,1) %0.014
% re = db(1,1,1,0,3,2,1) %0.001
% kd_k2_divide_d_k3_y1NEy0(1,1,1,0,3,2,1) %0.017
% db2(1,1,1,0,3,2,1) %0.001

% result = mathAtan2(1,2)

function[PointsSequence,theta4sequence] = LinearDigPlanningRandomLine(StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4)%解决任意可直达直线挖掘
%为了解决加上pid控制后不能精确到达目标位置的问题
    begindistance = norm(StartPoint-EndPoint)
    if begindistance<=1
        jointAngle = InverseKinematicsPos2Angle(StartPoint);
        PointsSequence = [0;jointAngle(1);jointAngle(2);jointAngle(3)];
        theta4sequence = [0;theta4begin];
        disp('输入的两点距离太近了，小于1厘米');
        return;
    end
    EndPointtmp = EndPoint;
    
    kp = 1.1;
    ki = 0.002;
    kd = 0.65;
%     
%     kp = 1.2;
%     ki = 0.002;
%     kd = 0.65;
    
    [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(kp,ki,kd,StartPoint,EndPointtmp,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4);
    if DirectReachable==0
        error('输入的两点不是直接的直线可达！返回上一层函数403');
        return;
    end
    
    lastdirection = 0;
    countTimes = 0;
    Sumerro = 0;
    
    kpScale = 0.001;
    kiScale = 0.00015;%这个得尽量小，否则一不小心就积大了
    
    countDirectReachable0 = 0;
    while norm(EndPoint-endPosition)>1
        PointsSequenceStorage = PointsSequence;
        theta4sequenceStorage = theta4sequence;
        endPositionStorage = endPosition;
        DirectReachableStorage = DirectReachable;
%         [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(1.1,0.002,0.65,StartPoint,EndPointtmp,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4);
        [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(kp,ki,kd,StartPoint,EndPointtmp,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4);
        
        if DirectReachable==0
            countDirectReachable0 = countDirectReachable0 + 1;
            EndPointtmp = StartPoint + (1-0.1) * (EndPoint-StartPoint);
            while IsDirectReachablePlane(StartPoint,EndPointtmp)==0
                EndPointtmp = StartPoint + (1-0.1) * (EndPointtmp-StartPoint);
            end
%             kiScale = 0;
            endPosition = endPositionStorage;
            
            if countDirectReachable0 == 2
                %说明有一点太靠近边界了，再这样下去也没啥用！
                %PointA = [-32.1682 -588.1967  218.8374];
                % PointB = [-30.6171 -538.7770  260.6377];
                PointsSequence = PointsSequenceStorage;
                theta4sequence = theta4sequenceStorage;
                endPosition = endPositionStorage;
                DirectReachable = DirectReachableStorage;
                return;
            end
            continue;
        else
            if dot(StartPoint-endPosition,EndPoint-endPosition)<0
                direction = 1;
            else
                direction = -1;
            end
            
            if direction*lastdirection == -1
                countTimes = countTimes + 1;
                if countTimes>5
                    ki = 0;
                    kiScale = 0;
                    EndPointtmp = EndPoint;
                    lastdirection = 0;
                    countTimes = 0;
                    continue;
                end
            else
                countTimes = 0;
            end
            lastdirection = direction;

            %         EndPointtmp = StartPoint + (1+direction*0.1) * (EndPointtmp-StartPoint);
            
            Sumerro = Sumerro+norm(EndPoint-endPosition);
            sumI = kiScale*Sumerro
            xishu = kpScale*norm(EndPoint-endPosition) + kiScale*Sumerro;
            xishu = Limit2range(xishu,[0,0.1]);
            EndPointtmp = StartPoint + (1+direction*xishu) * (EndPointtmp-StartPoint);
        end
        
%         plot3(EndPointtmp(1),EndPointtmp(2),EndPointtmp(3),'o');
%         hold on 
%         pause(0.1);
        norm(EndPoint-endPosition)
    end
    [pos1,pos2] = ForwardKinematics([PointsSequence(2,end) PointsSequence(3,end) PointsSequence(4,end) 0]);
    CurrentPoint = pos1(1:3,4);
    CurrentPoint = CurrentPoint';
    enddistance = norm(CurrentPoint-EndPoint)
end

function [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(kp,ki,kd,StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4)%解决任意可直达直线挖掘的子函数
  %PointsSequence 存储theta1 theta2 theta3 随时间变化的序列
    if IsDirectReachablePlane(StartPoint,EndPoint) == 0
        PointsSequence = [];
        theta4sequence = [];
        DirectReachable = 0;
        endPosition = [];
        disp('输入的两点不是直接的直线可达！返回上一层函数');
        return;
    end
    DirectReachable = 1;
    PointsSequence = [];
    tinterval = 0.01; %匀速运动时间是0.01秒
    angleinterval = 1; %每次关节运动1度
    tcurrent = 0;
    
    x0 = StartPoint(1);
    y0 = StartPoint(2);
    z0 = StartPoint(3);
    x1 = EndPoint(1);
    y1 = EndPoint(2);
    z1 = EndPoint(3);

    fuhao = 1;
    num = 0;
    
    vtheta2Last = 0;
    vtheta3Last = 0; 
    
    jointAngleEnd = InverseKinematicsPos2Angle(EndPoint);
    CurrentPoint = StartPoint;
    jointAngle = InverseKinematicsPos2Angle(StartPoint);
    theta1 = jointAngle(1);
    theta2 = jointAngle(2);
    theta3 = jointAngle(3);

    Sumerrovalue = 0;
    Lasterrovalue = norm(CurrentPoint-EndPoint)/norm(StartPoint-EndPoint);
    Queue = [];
    QueueSize = 15;
%     flagSlowDown = 0;
% figure
    
    while norm(CurrentPoint-EndPoint)>1
        Queue = [Queue norm(CurrentPoint-EndPoint)];
        if size(Queue,2)==QueueSize+1
            Queue(1) = [];
        end
        if size(Queue,2)==QueueSize
            if abs(mean(Queue)-Queue(1))<0.1
                break;
            end
        end
        num = num+1;
        PointsSequence(1,num) = tcurrent;
        PointsSequence(2,num) = theta1;
        PointsSequence(3,num) = theta2;
        PointsSequence(4,num) = theta3;
        
        errovalue = norm(CurrentPoint-EndPoint)/norm(StartPoint-EndPoint);
        Sumerrovalue = Sumerrovalue+errovalue;
        derrovalue = abs((errovalue-Lasterrovalue)/tinterval);
%         k_xishuV = 1.1*errovalue + 0.002*Sumerrovalue - 0.65*derrovalue; %修改这个pid的参数达到更好的效果
        k_xishuV = kp*errovalue + ki*Sumerrovalue - kd*derrovalue;
        k_xishuV = Limit2range(k_xishuV,[0,1]);
        Lasterrovalue = errovalue;
        
        [vtheta2,vtheta3] = GetCurrentvtheta3RandomLine(k_xishuV,jointAngle,StartPoint,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3);
        x3 = vtheta3^2/(2*amaxtheta3)+abs(vtheta3)*tinterval/2;
      
%         subplot(131)
%         plot(num,x3-abs(jointAngleEnd(3)-jointAngle(3)),'.');
%         hold on 
%         subplot(133)
%         plot(num,norm(CurrentPoint-EndPoint),'.');
%         hold on 
%         subplot(132)
%         plot(num,vtheta3,'.');
%         hold on
%         pause(0.1);

%         if x3 > abs(jointAngleEnd(3)-jointAngle(3))
% %         if abs(x3-abs(jointAngleEnd(3)-jointAngle(3)))<2
% %         if abs(x3-(jointAngleEnd(3)-jointAngle(3)))<2 %&& norm(CurrentPoint-EndPoint)<10
% %             if norm(CurrentPoint-EndPoint)<250
%                 flagSlowDown = 1;
% %             end
%         end
        
        vtheta2Last = vtheta2;
        vtheta3Last = vtheta3;
        theta2 = theta2 + vtheta2*tinterval;
        theta3 = theta3 + vtheta3*tinterval;
        theta2 = legalizAnger(theta2);
        theta3 = legalizAnger(theta3);
        jointAngle = [theta1 theta2 theta3];
        tcurrent = tcurrent + tinterval;
        [pos1,pos2] = ForwardKinematics([theta1 theta2 theta3 0]);
        CurrentPoint = pos1(1:3,4);
        CurrentPoint = CurrentPoint';
%         norm(CurrentPoint-EndPoint)
%         if flagSlowDown == 1
%             break; %之后就开始减速 以两个关节都能承受的加速度进行减速，只不过最后可能会超出末位置 如果是以最大值进行减速，则正好到目标点，否则会超过目标点。要尽可能使加速度达到最大！
%         end
    end

%     if flagSlowDown == 0 %必须减速
%         flagSlowDown = 1;
%         EndPoint = StartPoint + 2*(EndPoint-StartPoint);
%         x1 = EndPoint(1);
%         y1 = EndPoint(2);
%         z1 = EndPoint(3);
%     end
    
%     if flagSlowDown == 1
   
    acurrenttheta3 = amaxtheta3;
    while abs(vtheta3)>(acurrenttheta3*tinterval)
        k1 = theta1*pi/180;
        k2 = theta2*pi/180;
        k3 = theta3*pi/180;
        acurrenttheta2 = amaxtheta2;
        acurrenttheta3 = amaxtheta3;
        if x0 ~= x1
            tmpkxishu = kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1);
            if amaxtheta2 < abs(tmpkxishu)*amaxtheta3
                %说明amaxtheta2 带不动amaxtheta3
                acurrenttheta3 = acurrenttheta2/tmpkxishu;
                acurrenttheta3 = abs(acurrenttheta3);
                disp('要注意铲斗不是精确到目标位置!要想精确，首先提高加速度约束的大小');
            end
        else
            if y0~=y1
                tmpkxishu = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1);
                if amaxtheta2 <abs(tmpkxishu)*amaxtheta3
                    acurrenttheta3 = acurrenttheta2/tmpkxishu;
                    acurrenttheta3 = abs(acurrenttheta3);
                    disp('要注意铲斗不是精确到目标位置!要想精确，首先提高加速度约束的大小');
                end
            else
                disp('要完成的直线是垂直于地面的')
            end
        end
        if vtheta3 < 0
            vtheta3 = vtheta3 + acurrenttheta3*tinterval;
        else
            vtheta3 = vtheta3 - acurrenttheta3*tinterval;
        end
        d_k3 = vtheta3*pi/180;
        if x0~=x1
            d_k2 = kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1)*d_k3;
        else
            if y0~=y1
                d_k2 = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1)*d_k3;
            else
                disp('要完成的直线是垂直于地面的')
            end
        end
%             d_k2 = -(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)); %这是通过求雅可比矩阵得到的对应关系
        vtheta2 = d_k2*180/pi;
        theta2 = theta2 + vtheta2*tinterval;
        theta3 = theta3 + vtheta3*tinterval;
        theta2 = legalizAnger(theta2);
        theta3 = legalizAnger(theta3);

        num = num+1;
        PointsSequence(1,num) = tcurrent;
        PointsSequence(2,num) = theta1;
        PointsSequence(3,num) = theta2;
        PointsSequence(4,num) = theta3;
        tcurrent = tcurrent + tinterval;

%         [pos1,pos2] = ForwardKinematics([theta1 theta2 theta3 0]);
%         CurrentPoint = pos1(1:3,4);
%         CurrentPoint = CurrentPoint';
%         norm(CurrentPoint-EndPoint)
    end
  
    [pos1,pos2] = ForwardKinematics([PointsSequence(2,end) PointsSequence(3,end) PointsSequence(4,end) 0]);
    CurrentPoint = pos1(1:3,4);
    CurrentPoint = CurrentPoint';
    
    %接下来只要在上述规划时间内把铲斗摆动到预期位置即可
    
    success = 0;
    k_tf = 1.1;
    tf = PointsSequence(1,end)/k_tf;
    while success == 0
        tf = k_tf*tf;
        [theta4sequence,success] = ManipulatorPlanningJointSpace(theta4begin,theta4end,tf,amaxtheta4,Vmaxtheta4,tinterval);
    end
%     enddistance = norm(CurrentPoint-EndPoint); %这是误差，经过少量测试，误差不会超过2cm
    endPosition = CurrentPoint;
end

function thetaout = legalizAnger(theta) %把角度限制在(-180,180]
    thetaout = theta;
    while thetaout>180 
        thetaout = thetaout-360;
    end
    while thetaout<=-180
        thetaout = thetaout+360;
    end
end

function valueout = Limit2range(value,range)
    valueout = value;
    if value<range(1)
        valueout = range(1);
    else
        if value>range(2)
            valueout = range(2);
        end
    end
end

function jointAngle = InverseKinematicsPos2Angle(position)%输入为列向量，输出为前三个角的角度 为横向量
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
    

	jointAngle(3) = -abs(acos(Mtemp1));


%     double tempm, tempn, tempTwo1;
	tempm = px*cos(jointAngle(1)) + py*sin(jointAngle(1)) - a(2);
	tempn = pz - d(1);
    if abs(tempm*tempm + tempn*tempn) <= ZERO
        disp("error!");
        return ;
    else
        tempTwo1 = ((a(3) + a(4) * cos(jointAngle(3)))*tempn - a(4) * sin(jointAngle(3))*tempm) / (tempm*tempm + tempn*tempn);
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

function [vtheta2,vtheta3] = GetCurrentvtheta3RandomLine(k_xishu,CurrentJointAngle,StartPoint,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3)%输出满足加速度限制的当前时刻速度值
    GlobalDeclarationCommon
    amaxtheta2 = amaxtheta2*pi/180;
    amaxtheta3 = amaxtheta3*pi/180;
    fuhao = 1;
    lastd_k2 = vtheta2Last*pi/180;
    lastd_k3 = vtheta3Last*pi/180;
    flagonce = 0;
    xishu = 0.3; %每次迭代的缩放系数
    k_yuzhi = 0.9; %先设置为允许的留有一定裕度的最大值
    
    x0 = StartPoint(1);
    y0 = StartPoint(2);
    z0 = StartPoint(3);
    x1 = EndPoint(1);
    y1 = EndPoint(2);
    z1 = EndPoint(3);
   
%      jointAngle = InverseKinematicsPos2Angle(CurrentPoint);
    theta1 = CurrentJointAngle(1);
    theta2 = CurrentJointAngle(2);
    theta3 = CurrentJointAngle(3);
    [pos1,pos2] = ForwardKinematics([theta1 theta2 theta3 0]);
    CurrentPoint = pos1(1:3,4);
    CurrentPoint = CurrentPoint';
    
    [Vtheta2range,Vtheta3range] = LinearDiggingGetRangeRandomLine(Vmaxtheta2,Vmaxtheta3,theta1,theta2,theta3,StartPoint,EndPoint);
    
    theta1 = theta1*pi/180;
    theta2 = theta2*pi/180;
    theta3 = theta3*pi/180;
    k1 = theta1; k2 = theta2; k3 = theta3; k4 = theta4Range(2)*pi/180;
    
    lasta_k2 = 0;
    lasta_k3 = 0;
    
    absvtheta3Upper = abs(lastd_k3) + amaxtheta3*tinterval;
    absvtheta3Lower = abs(lastd_k3) - amaxtheta3*tinterval;
    absvtheta2Upper = abs(lastd_k2) + amaxtheta2*tinterval;
    absvtheta2Lower = abs(lastd_k2) - amaxtheta2*tinterval;
    huduVtheta2range = Vtheta2range*pi/180;
    huduVtheta3range = Vtheta3range*pi/180;
    if (huduVtheta2range<absvtheta2Lower) || (huduVtheta3range<absvtheta3Lower)
        error('需要重新修改相关参数，这组参数无法满足加速度约束');
        return;
    end
    
    if huduVtheta3range>=absvtheta3Upper %求得满足加速度限制的Vtheta3的上下界
        Fita_Vtheta3Upper = absvtheta3Upper;
        Fita_Vtheta3Lower = absvtheta3Lower;
    else
        Fita_Vtheta3Upper = huduVtheta3range;
        Fita_Vtheta3Lower = absvtheta3Lower;
    end
    if huduVtheta2range>=absvtheta2Upper %求得满足加速度限制的Vtheta3的上下界
        Fita_Vtheta2Upper = absvtheta2Upper;
        Fita_Vtheta2Lower = absvtheta2Lower;
    else
        Fita_Vtheta2Upper = huduVtheta2range;
        Fita_Vtheta2Lower = absvtheta2Lower;
    end
    
    
    if vtheta3Last<0
        Fita_Vtheta3Upper = -Fita_Vtheta3Upper;
        Fita_Vtheta3Lower = -Fita_Vtheta3Lower;
        exchangetmp = Fita_Vtheta3Upper;
        Fita_Vtheta3Upper = Fita_Vtheta3Lower;
        Fita_Vtheta3Lower = exchangetmp;
    end
    if vtheta2Last<0
        Fita_Vtheta2Upper = -Fita_Vtheta2Upper;
        Fita_Vtheta2Lower = -Fita_Vtheta2Lower;
        exchangetmp = Fita_Vtheta2Upper;
        Fita_Vtheta2Upper = Fita_Vtheta2Lower;
        Fita_Vtheta2Lower = exchangetmp;
    end
    
    Fita_Vtheta3 = [Fita_Vtheta3Lower Fita_Vtheta3Upper];%这是弧度
    flagYESsolution = 0;
    Vtheta2set = [];
    Vtheta3set = [];
    d_k2Storage = [];
    d_k3Storage = [];
    for i=1:2
        d_k3 = Fita_Vtheta3(i);
%         d_k3 = fuhao * vtheta3 * pi/180;
        if x0~=x1
            kd_k2_divide_d_k3 = kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1);
            d_k2 = kd_k2_divide_d_k3*d_k3;
%             d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
        else
            if y0~=y1
                kd_k2_divide_d_k3 = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1);
                d_k2 = kd_k2_divide_d_k3*d_k3;
%                 d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))*d_k3/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
            end
        end
        %         Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
%         Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
%         Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;
        Vx = d_k2*((sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + cos(k4)*cos(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) - (cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))) - d_k3*((a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + cos(k4)*cos(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2))) - (a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*sin(m3)*(cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2))));
        Vy = - d_k2*((cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2)) + (sin(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + cos(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - cos(k4)*cos(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3))) - d_k3*((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + cos(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - cos(k4)*cos(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + a3*sin(m3)*(cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))));
        Vz = d_k2*((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))) + d_k3*((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*sin(m3)*(sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))));
        if dot(EndPoint-CurrentPoint,[Vx,Vy,Vz])<0
            if Fita_Vtheta3(1)*Fita_Vtheta3(2)<0
                d_k3 = 0;
                d_k2 = 0;
            else
                d_k3 = -d_k3;
                d_k2 = -d_k2;
            end
%             Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
%             Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
%             Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;
            Vx = d_k2*((sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + cos(k4)*cos(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) - (cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))) - d_k3*((a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + cos(k4)*cos(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2))) - (a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*sin(m3)*(cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2))));
            Vy = - d_k2*((cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2)) + (sin(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + cos(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - cos(k4)*cos(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3))) - d_k3*((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + cos(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - cos(k4)*cos(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + a3*sin(m3)*(cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))));
            Vz = d_k2*((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))) + d_k3*((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*sin(m3)*(sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))));
            if dot(EndPoint-CurrentPoint,[Vx,Vy,Vz])<0
                error('');
            end
        end
        d_k2Storage = [d_k2Storage d_k2*180/pi];
        d_k3Storage = [d_k3Storage d_k3*180/pi];
%         yuzhithistime = 0;
%         if d_k2>=Fita_Vtheta2Lower-yuzhithistime && d_k2<=Fita_Vtheta2Upper+yuzhithistime
%             vtheta3 = d_k3*180/pi;
%             vtheta2 = d_k2*180/pi;
%             Vtheta2set = [Vtheta2set vtheta2];
%             Vtheta3set = [Vtheta3set vtheta3];
%             flagYESsolution = flagYESsolution+1;
%         end

%         a_k2 = abs((d_k2-lastd_k2)/tinterval);
%         a_k3 = abs((d_k3-lastd_k3)/tinterval);
    end

    vtheta2intersection = GetIntersection(d_k2Storage,[Fita_Vtheta2Lower*180/pi Fita_Vtheta2Upper*180/pi]);
    vtheta3intersection = GetIntersection(d_k3Storage,[Fita_Vtheta3Lower*180/pi Fita_Vtheta3Upper*180/pi]);
    
    if isempty(vtheta2intersection)==1 || isempty(vtheta3intersection)==1
        error('需要重新修改相关参数，这组参数无法满足加速度约束');
    end
    vtheta3intersectiontmp = [];
    for i=1:size(vtheta2intersection,2)
        d_k2 = vtheta2intersection(i)*pi/180;
        if x0~=x1
            Vtheta3tmp = (- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1))));
        else
            if y0~=y1
                Vtheta3tmp = (- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1))));
            else
                disp('说明只有z方向，此时只有一组vtheta2 vthata3的解')
            end
        end
        vtheta3intersectiontmp(1,i) = Vtheta3tmp*180/pi;
    end
    
    if isempty(GetIntersection(vtheta3intersectiontmp,vtheta3intersection))==1
        erro('程序逻辑出错！');
    end
    
    vtheta2reliablerange = vtheta2intersection; %可靠的取值范围
    vtheta3reliablerange = vtheta3intersectiontmp;
    
    if isempty(vtheta2reliablerange)==1 || isempty(vtheta3reliablerange)==1
        erro('程序逻辑出错！');
    end
    
%     vtheta3 = vtheta3reliablerange(end);
%     vtheta2 = vtheta2reliablerange(end);
%     if abs(vtheta2) <=0.00001 && abs(vtheta3) <= 0.00001
%         vtheta3 = vtheta3reliablerange(1);
%         vtheta2 = vtheta2reliablerange(1);
%     end

%     k_xishu = 0.9; %k_xishu取值范围为[0,1] k_xishu越大，平均速度越大
    if abs(vtheta3reliablerange(end))>=abs(vtheta3reliablerange(1))
        vtheta3 = vtheta3reliablerange(1) + (vtheta3reliablerange(end)-vtheta3reliablerange(1))*(k_xishu);
        vtheta2 = vtheta2reliablerange(1) + (vtheta2reliablerange(end)-vtheta2reliablerange(1))*(k_xishu);
    else
        vtheta3 = vtheta3reliablerange(1) + (vtheta3reliablerange(end)-vtheta3reliablerange(1))*(1-k_xishu);
        vtheta2 = vtheta2reliablerange(1) + (vtheta2reliablerange(end)-vtheta2reliablerange(1))*(1-k_xishu);
    end
end

function out = GetIntersection(qujianA,qujianB)
%20200717 写这个交集函数
    out = [];
    if isempty(qujianA)==1 || isempty(qujianB)==1
        out = [];
        return;
    end
    if size(qujianA,2)==2 && qujianA(1)==qujianA(2)
        qujianA = qujianA(1);
    end
    if size(qujianB,2)==2 && qujianB(1)==qujianB(2)
        qujianB = qujianB(1);
    end
    if size(qujianA,2)==1 && size(qujianB,2)==1
        if qujianA == qujianB
            out = qujianA;
        else
            out = [];
        end
        return;
    end
    if size(qujianA,2)==2 && qujianA(1)>qujianA(2)
        tmp = qujianA(1);
        qujianA(1) = qujianA(2);
        qujianA(2) = tmp;
    end
    if size(qujianB,2)==2 && qujianB(1)>qujianB(2)
        tmp = qujianB(1);
        qujianB(1) = qujianB(2);
        qujianB(2) = tmp;
    end
    if size(qujianA,2)==1 && size(qujianB,2)==2
        if qujianA(1)<qujianB(1) || qujianA(1)>qujianB(2)
            out=[];
        else
            out = qujianA(1);
        end
        return;
    end
    if size(qujianA,2)==2 && size(qujianB,2)==1
        if qujianB(1)<qujianA(1) || qujianB(1)>qujianA(2)
            out=[];
        else
            out = qujianB(1);
        end
        return;
    end
    
    if qujianA(2)<qujianB(1) || qujianA(1)>qujianB(2)
        out = [];
    else 
        if qujianA(2)==qujianB(1)
            out = qujianA(2);
        end
        if qujianA(1)==qujianB(2)
            out = qujianB(2);
        end
        if isempty(out) == 0
            return;
        end
        if qujianA(1)<=qujianB(1)
            if qujianA(2)>qujianB(1) && qujianA(2)<=qujianB(2)
                out = [qujianB(1) qujianA(2)];
            else
                out = [qujianB(1) qujianB(2)];
            end
        else
            if qujianA(2)>qujianB(1) && qujianA(2)<=qujianB(2)
                out = [qujianA(1) qujianA(2)];
            else
                out = [qujianA(1) qujianB(2)];
            end
        end
    end
end

function [Vtheta2,Vtheta3] = LinearDiggingGetRangeRandomLine(Vmaxtheta2,Vmaxtheta3,theta1,theta2,theta3,StartPoint,EndPoint)
     %要保证输入Vmax是正值
    Vmaxtheta2 = Vmaxtheta2*pi/180;
    Vmaxtheta3 = Vmaxtheta3*pi/180;
    theta1 = theta1*pi/180;
    theta2 = theta2*pi/180;
    theta3 = theta3*pi/180;
    x0 = StartPoint(1);
    y0 = StartPoint(2);
    z0 = StartPoint(3);
    x1 = EndPoint(1);
    y1 = EndPoint(2);
    z1 = EndPoint(3);
    
    d_k3 = Vmaxtheta3;
    k1 = theta1;
    k2 = theta2;
    k3 = theta3;
    if x0~=x1
        kd_k2_divide_d_k3 = kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1);
        Vtheta2tmp = kd_k2_divide_d_k3*d_k3;
%         Vtheta2tmp = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
        d_k2 = Vmaxtheta2;
        Vtheta3tmp = 1/kd_k2_divide_d_k3*d_k2;
%         Vtheta3tmp = (- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1))));
    else
        if y0~=y1
            kd_k2_divide_d_k3 = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1);
            Vtheta2tmp = kd_k2_divide_d_k3 * d_k3;
%             Vtheta2tmp = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))*d_k3/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
            d_k2 = Vmaxtheta2;
            Vtheta3tmp = 1/kd_k2_divide_d_k3*d_k2;
%             Vtheta3tmp = (- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1))));
        else
            disp('说明只有z方向，此时只有一组vtheta2 vthata3的解')
        end
    end
    %     Vtheta2tmp = -(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)); %这是通过求雅可比矩阵得到的对应关系
%     d_k2 = Vmaxtheta2;
%     Vtheta3tmp = -((2109*cos(k2 + k3) + 4600*cos(k2))/(2109*cos(k2 + k3)))*d_k2;
    if abs(Vtheta2tmp)<=Vmaxtheta2
        Vtheta2 = abs(Vtheta2tmp);
        Vtheta3 = Vmaxtheta3;
    else
        if abs(Vtheta3tmp)<= Vmaxtheta3
            Vtheta3 = abs(Vtheta3tmp);
            Vtheta2 = Vmaxtheta2;
        end
    end   
    Vtheta2 = Vtheta2*180/pi;
    Vtheta3 = Vtheta3*180/pi;
end

function result = kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1)
    GlobalDeclarationCommon
    k4 = theta4Range(2) * pi / 180; %这个是任意取的 对最终结果没有影响
    result = -(- ((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*sin(m3)*(sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))))/((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))) - ((z0 - z1)*((a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + cos(k4)*cos(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2))) - (a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*sin(m3)*(cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))))/((x0 - x1)*((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2)))))/((((z0 - z1)*((sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + cos(k4)*cos(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) - (cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))))/((x0 - x1)*((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2)))) - 1));
end

function result = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1)
    GlobalDeclarationCommon
    k4 = theta4Range(2) * pi / 180; %这个是任意取的 对最终结果没有影响
    result = -((- ((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*sin(m3)*(sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))))/((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))) - ((z0 - z1)*((a3*cos(m3)*sin(k4) + d4*cos(k4)*sin(m3))*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) - (a3*cos(k4)*cos(m3) - d4*sin(k4)*sin(m3))*(sin(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + cos(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - cos(k4)*cos(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + a3*sin(m3)*(cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))))/((y0 - y1)*((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2))))))/((- ((z0 - z1)*((cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2)) + (sin(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + cos(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - cos(k4)*cos(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3))))/((y0 - y1)*((sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(a2*cos(m3)*sin(m2) + a3*cos(m2)*sin(m3) + a2*cos(k3)*cos(m2)*sin(m3) + a3*cos(k3)*cos(m3)*sin(m2) - d3*sin(k3)*sin(m2)*sin(m3)) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - cos(k4)*cos(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(m2)*sin(k3)*sin(k4) - a3*cos(k4)*cos(m2)*cos(m3) + a2*cos(k4)*sin(m2)*sin(m3) + d3*cos(k3)*sin(k4)*sin(m2) + d4*cos(k4)*sin(k3)*sin(m2) + d4*cos(m2)*sin(k4)*sin(m3) + a3*cos(k3)*cos(k4)*sin(m2)*sin(m3) + d3*cos(k4)*cos(m3)*sin(k3)*sin(m2) + d4*cos(k3)*cos(m3)*sin(k4)*sin(m2) - a2*cos(k3)*cos(k4)*cos(m2)*cos(m3)) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))))*(a2*cos(k4)*cos(m2)*sin(k3) + a3*cos(m2)*cos(m3)*sin(k4) + d3*cos(k3)*cos(k4)*sin(m2) + d4*cos(k4)*cos(m2)*sin(m3) - a2*sin(k4)*sin(m2)*sin(m3) - d4*sin(k3)*sin(k4)*sin(m2) + a2*cos(k3)*cos(m2)*cos(m3)*sin(k4) + d4*cos(k3)*cos(k4)*cos(m3)*sin(m2) - a3*cos(k3)*sin(k4)*sin(m2)*sin(m3) - d3*cos(m3)*sin(k3)*sin(k4)*sin(m2)))) - 1));
end

% function re = db(k1,k2,k3,x0,z0,x1,z1) 
% re = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
% 
% end
% 
% function re = db2(k1,k2,k3,y0,z0,y1,z1)
% re = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
% 
% end


function blockcodeA = LocateWhereBlock(PointA)
    GlobalDeclarationCommon
    blockcodeA = 0;
    down = TwoInnerTangentPoints(1,2);
    up = TwoInnerTangentPoints(2,2);
    
    if isempty(GetIntersection([down-10000 down],PointA(3)))==0
        blockcodeA = 1;
    else
        if isempty(GetIntersection([down up],PointA(3)))==0
            blockcodeA = 2;
        else
            blockcodeA = 3;
        end
    end
end

function YES = IsDirectReachablePlaneSub(CH,waitforprocess,anotherPoint) %判断直达的其中一些情况判断 一个圆外的点，有两条切线
    num = 0;
    for i=1:size(CH,1)
        if norm(CH(i,:)-waitforprocess)<0.001
            num = i;
            break;
        end
    end
    if num == 0
        error('逻辑出错了！！');
    end
    
    if num == 1
        leftPoint = CH(end,:);
        rightPoint = CH(num+1,:);
    else
        leftPoint = CH(num-1,:);
        rightPoint = CH(num+1,:);
    end
    
    if ToLeftTest([waitforprocess,0],[rightPoint 0],[anotherPoint,0])>=0 ...
            && ToLeftTest([waitforprocess,0],[leftPoint 0],[anotherPoint,0])<=0 ...
            && InTriangle([waitforprocess 0], [leftPoint 0], [rightPoint 0],[anotherPoint,0]) == -1
        YES = 0;
    else
        YES = 1;
    end
end

function YES = ProcessPointABandblock(waitforprocess,anotherPoint,blockwaitforprocess) %组合运用IsDirectReachablePlaneSub，适应不同的形状
    GlobalDeclarationCommon
    if blockwaitforprocess == 1 
        InnerCH = [InnerEdgeDown;waitforprocess];
        InnerCH = [InnerCH zeros(size(InnerCH,1),1)];
        InnerCH = GetCHGrahamScan(InnerCH);
        InnerCH(:,3) = [];
        YESsub = IsDirectReachablePlaneSub(InnerCH,waitforprocess,anotherPoint);
        if YESsub == 1
            YES = 1;
        else
            YES = 0;
        end
    else
        if blockwaitforprocess == 3
            InnerCH = [InnerEdgeUp;waitforprocess];
            InnerCH = [InnerCH zeros(size(InnerCH,1),1)];
            InnerCH = GetCHGrahamScan(InnerCH);
            InnerCH(:,3) = [];
            YESsub = IsDirectReachablePlaneSub(InnerCH,waitforprocess,anotherPoint);
            if YESsub == 1
                YES = 1;
            else
                YES = 0;
            end
        else
            if blockwaitforprocess == 2
                InnerCH = [InnerEdgeDown;waitforprocess];
                InnerCH = [InnerCH zeros(size(InnerCH,1),1)];
                InnerCH = GetCHGrahamScan(InnerCH);
                InnerCH(:,3) = [];
                YESsub1 = IsDirectReachablePlaneSub(InnerCH,waitforprocess,anotherPoint);

                InnerCH = [InnerEdgeUp;waitforprocess];
                InnerCH = [InnerCH zeros(size(InnerCH,1),1)];
                InnerCH = GetCHGrahamScan(InnerCH);
                InnerCH(:,3) = [];
                YESsub2 = IsDirectReachablePlaneSub(InnerCH,waitforprocess,anotherPoint);

                if YESsub1==1 && YESsub2==1
                    YES = 1;
                else
                    YES = 0;
                end
            end
        end
    end
end

function YES = IsDirectReachablePlane(PointA,PointB) %二维机械臂平面的可达性判断
    GlobalDeclarationCommon
    YES = [];
    jointAngle1 = InverseKinematicsPos2Angle(PointA);
    jointAngle2 = InverseKinematicsPos2Angle(PointB);
    if abs(jointAngle1(1)-jointAngle2(1))>0.001
        error('无法判断这种不和机械臂在同一平面的可达性');
        return;
    end
    if isempty(GetIntersection(jointAngle1(1),theta1Range)) || isempty(GetIntersection(jointAngle1(2),theta2Range)) || isempty(GetIntersection(jointAngle1(3),theta3Range)) ...
          || isempty(GetIntersection(jointAngle2(1),theta1Range)) || isempty(GetIntersection(jointAngle2(2),theta2Range)) || isempty(GetIntersection(jointAngle2(3),theta3Range))
        error('有一点处于工作空间之外！');
        return;
    end
    
    [A,B,C] = GetLineABC(TwoInnerTangentPoints(1,:),TwoInnerTangentPoints(2,:));
    if TwoInnerTangentPoints(1,2)>TwoInnerTangentPoints(2,2)
        tmpswap = TwoInnerTangentPoints(2,2);
        TwoInnerTangentPoints(2,2) = TwoInnerTangentPoints(1,2);
        TwoInnerTangentPoints(1,2) = tmpswap;
    end
    
%     z=(-A*y-C)/B;
%     y=(-B*z-C)/A;
    PointALocatey = (PointA(1)^2+PointA(2)^2-d2^2)^(1/2) ;
    yCriticalA = (-B*PointA(3)-C)/A;
    PointBLocatey = (PointB(1)^2+PointB(2)^2-d2^2)^(1/2) ;
    yCriticalB = (-B*PointB(3)-C)/A;
    FakePointA = [0 PointALocatey PointA(3)];
    FakePointB = [0 PointBLocatey PointB(3)];
    if PointALocatey>=yCriticalA && PointBLocatey>=yCriticalB
        YES = 1;
    else
        if PointALocatey<=yCriticalA && PointBLocatey<=yCriticalB
            blockcodeA = LocateWhereBlock(FakePointA);
            blockcodeB = LocateWhereBlock(FakePointB);
            if blockcodeA==0||blockcodeB==0
                error('程序逻辑出错！');
            end
            if blockcodeA ~= blockcodeB
                YES = 0;
            else
                waitforprocess = FakePointA(2:3);
                anotherPoint = FakePointB(2:3);
                blockwaitforprocess = blockcodeA;
                YES = ProcessPointABandblock(waitforprocess,anotherPoint,blockwaitforprocess);
            end
        else
            blockcodeA = 0;
            blockcodeB = 0;
            if PointALocatey<=yCriticalA
                blockcodeA = LocateWhereBlock(FakePointA);
            else
                blockcodeB = LocateWhereBlock(FakePointB);
            end
            if blockcodeA==0 && blockcodeB==0
                error('程序逻辑出错！');
            end
            blockwaitforprocess = 0;
            if blockcodeA~=0
                waitforprocess = FakePointA(2:3);
                anotherPoint = FakePointB(2:3);
                blockwaitforprocess = blockcodeA;
            else
                waitforprocess = FakePointB(2:3);
                anotherPoint = FakePointA(2:3);
                blockwaitforprocess = blockcodeB;
            end
            YES = ProcessPointABandblock(waitforprocess,anotherPoint,blockwaitforprocess);
        end
    end
    if isempty(YES)==1
        error('程序逻辑出错！');
    end
end

function [PointsSequence,success] = ManipulatorPlanningJointSpace(theta0,thetaf,tf,AMAX,VMAX,SampleTime) %默认初始末点速度为0，加速度为0;
%-180,180 角度体系
    theta0 = legalizAnger(theta0);
    thetaf = legalizAnger(thetaf);
    
    success = 0;
    flagNeedXG = 0;
    if theta0*thetaf<0
        if theta0>0
            if theta0-thetaf>180
                thetaf = 180+180+thetaf;
                flagNeedXG = 1;
            end
        else
            if thetaf-theta0>180
                theta0 = 180+180+theta0;
                flagNeedXG = 1;
            end
        end
    end
    PointsSequence = [];
    k = 0.1; %k=ta/tb,取值范围为[0,0.5]
    if theta0 == thetaf
        for i=1:tf/SampleTime+1
            PointsSequence(1,i) = (i-1)*SampleTime;
            PointsSequence(2,i) = theta0;
        end
        success = 1;
        return;
    end
    if (theta0<thetaf)
        a_LowerBound = 4*(theta0-thetaf)/( (k-1)*tf^2 );
        a_Uppertemp = VMAX^2 / ( (1-k)*(theta0-thetaf+tf*VMAX) );
        if a_Uppertemp<a_LowerBound
            disp('1VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX');
            return;
        end
        if a_Uppertemp>=AMAX
            if AMAX<a_LowerBound
                disp('加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX');
                return;
            end
            a_UpperBound = AMAX;
        else
            if a_Uppertemp<a_LowerBound
                disp('2VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX');
                return;
            end
            a_UpperBound = a_Uppertemp;
        end
%         amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
        amax = a_UpperBound;
        tb = -((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*tf^2 - amax*k*tf^2))^(1/2) - amax*tf + amax*k*tf)/(2*(amax - amax*k));
        ta = k*tb;
        for i=1:tf/SampleTime+1
            PointsSequence(1,i) = (i-1)*SampleTime;
            t = PointsSequence(1,i);
            if t>=0&&t<ta
                PointsSequence(2,i) = (amax*t^3)/(6*k*tb) + theta0;
                continue;
            end
            if t>=ta && t<tb-ta
                PointsSequence(2,i) = theta0 + (amax*t*(t - k*tb))/2 + (amax*k^2*tb^2)/6;
                continue;
            end
            if t>=tb-ta && t<tb
                PointsSequence(2,i) = theta0 + (amax*t^2)/(2*k) + (amax*k^2*tb^2)/6 + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*t*k^2 - 6*t*k + 3*t))/(6*k) - (amax*t^3)/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
            if t>=tb && t<tf-tb
                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 + amax*(tb - k*tb)*(t - tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
            if t>=tf-tb && t<tf-tb+ta
                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k) - (amax*(t + tb - tf)*(6*k^2*tb^2 - 6*k*tb^2 + t^2 + 2*t*tb - 2*t*tf + tb^2 - 2*tb*tf + tf^2))/(6*k*tb);
                continue;
            end
            if t>=tf-tb+ta && t<tf-ta
                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - (amax*(t + tb - tf - k*tb)*(t - tb - tf + 2*k*tb))/2 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
            if t>=tf-ta && t<=tf
                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/3 - amax*(2*tb - tf)*(tb - k*tb) + amax*(tb - k*tb)*(tb - 2*k*tb) - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) + (amax*(t^3 - 3*t^2*tf + 3*t*tf^2 - tf^3))/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
        end
    else
        thetatemp = thetaf;
        thetaf = theta0;
        theta0 = thetatemp;
        a_LowerBound = 4*(theta0-thetaf)/( (k-1)*tf^2 );
        a_Uppertemp = VMAX^2 / ( (1-k)*(theta0-thetaf+tf*VMAX) );
        if a_Uppertemp<a_LowerBound
            disp('1VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX');
            return;
        end
        if a_Uppertemp>=AMAX
            if AMAX<a_LowerBound
                disp('加速度不满足要求，修改tf或者theta0 thetaf或者改变AMAX');
                return;
            end
            a_UpperBound = AMAX;
        else
            if a_Uppertemp<a_LowerBound
                disp('2VMAX不满足要求，修改tf或者theta0 thetaf或者改变VMAX');
                return;
            end
            a_UpperBound = a_Uppertemp;
        end
%         amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
        amax = a_UpperBound;
        tb = -((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*tf^2 - amax*k*tf^2))^(1/2) - amax*tf + amax*k*tf)/(2*(amax - amax*k));
        ta = k*tb;
        for i=1:tf/SampleTime+1
            PointsSequence(1,i) = (i-1)*SampleTime;
            t = PointsSequence(1,i);
            t = 2*(tf/2)-t;
            if t>=0&&t<ta
                PointsSequence(2,i) = (amax*t^3)/(6*k*tb) + theta0;
                continue;
            end
            if t>=ta && t<tb-ta
                PointsSequence(2,i) = theta0 + (amax*t*(t - k*tb))/2 + (amax*k^2*tb^2)/6;
                continue;
            end
            if t>=tb-ta && t<tb
                PointsSequence(2,i) = theta0 + (amax*t^2)/(2*k) + (amax*k^2*tb^2)/6 + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*t*k^2 - 6*t*k + 3*t))/(6*k) - (amax*t^3)/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
            if t>=tb && t<tf-tb
                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 + amax*(tb - k*tb)*(t - tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
            if t>=tf-tb && t<tf-tb+ta
                PointsSequence(2,i) = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k) - (amax*(t + tb - tf)*(6*k^2*tb^2 - 6*k*tb^2 + t^2 + 2*t*tb - 2*t*tf + tb^2 - 2*tb*tf + tf^2))/(6*k*tb);
                continue;
            end
            if t>=tf-tb+ta && t<tf-ta
                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - (amax*(t + tb - tf - k*tb)*(t - tb - tf + 2*k*tb))/2 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
            if t>=tf-ta && t<=tf
                PointsSequence(2,i) = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/3 - amax*(2*tb - tf)*(tb - k*tb) + amax*(tb - k*tb)*(tb - 2*k*tb) - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) + (amax*(t^3 - 3*t^2*tf + 3*t*tf^2 - tf^3))/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);
                continue;
            end
        end
    end
    if flagNeedXG==1
        for i =1:size(PointsSequence,2)
            PointsSequence(2,i) = legalizAnger(PointsSequence(2,i));
%             if PointsSequence(2,i)>180
%                 PointsSequence(2,i) = -180 + (PointsSequence(2,i)-180);
%             end
        end
    end
    success = 1;
end





