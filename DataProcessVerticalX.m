clc
clear 
close all

% load('垂直于x与x交于13.7.txt');
load('垂直于x与x交于13.7更精细的.txt');
% Data = X___x_x__13_7(:,2:3);
Data = X___x_x__13_7____(:,2:3);

% SortedData = SortData(Data);
load('SortedData更精细.mat');
plot(SortedData(:,1),SortedData(:,2),'.');
figure
plot(Data(:,1),Data(:,2),'.');

EdgeSet = EdgeExtraction(SortedData);
figure
plot(EdgeSet(:,1),EdgeSet(:,2),'.');
xlabel('y');
ylabel('z');

segmentdown = [];
segmentmid = [];
segmentup = [];


segmentdown = EdgeSet( find(EdgeSet(:,2)<=-274.3) , :);
[~,sortedindex] = sort(segmentdown(:,1));
sortedsegmentdown = [];
for i=1:size(sortedindex,1)
    sortedsegmentdown = [sortedsegmentdown;segmentdown(sortedindex(i),:)];
end
segmentdownleft = sortedsegmentdown(find(sortedsegmentdown(:,1)<400),:);
segmentdownright = sortedsegmentdown(find(sortedsegmentdown(:,1)>400),:);

segmentmid = EdgeSet( intersect( find(EdgeSet(:,2)>-275),find (EdgeSet(:,2)<=165.4) ) , :);
[a,yindex] = sort(segmentmid(:,1));
ysortedsegmentmid = [];
for i = 1:size(yindex,1)
    ysortedsegmentmid = [ysortedsegmentmid;segmentmid(yindex(i),:)] ;
end
segmentmidleft = [];
segmentmidright = [];

segmentmidleft = ysortedsegmentmid(find(ysortedsegmentmid(:,1)<500),:);
segmentmidright = ysortedsegmentmid(find(ysortedsegmentmid(:,1)>500),:);

segmentup = EdgeSet( intersect( find(EdgeSet(:,2)>163.5),find (EdgeSet(:,2)<=459.9) ) , :);
segmentupleft = [];
segmentupright = [];
PA = segmentup(86,:);
PB = segmentup(84,:);
segmentupleft = [segmentupleft;PA];
segmentupleft = [segmentupleft;PB];
for i= 1:size(segmentup,1)
    result = ToLeftTest(PA,PB,segmentup(i,:));
    if result ==-1
        segmentupleft = [segmentupleft;segmentup(i,:)];
    else
        if result == 1
            segmentupright = [segmentupright;segmentup(i,:)];
        end
    end 
end
segmentupright = [segmentupright;segmentupleft(end,:)];

figure
plot(segmentupleft(:,1),segmentupleft(:,2),'.');
hold on
plot(segmentupright(:,1),segmentupright(:,2),'.');
hold on 
plot(segmentmidleft(:,1),segmentmidleft(:,2),'.');
hold on
plot(segmentmidright(:,1),segmentmidright(:,2),'.');
hold on
plot(segmentdownleft(:,1),segmentdownleft(:,2),'.');
hold on
plot(segmentdownright(:,1),segmentdownright(:,2),'.');



segmentupleftY = segmentupleft(:,1);
segmentupleftZ = segmentupleft(:,2);

segmentuprightY = segmentupright(:,1);
segmentuprightZ = segmentupright(:,2);

segmentmidleftY = segmentmidleft(:,1);
segmentmidleftZ = segmentmidleft(:,2);

segmentmidrightY = segmentmidright(:,1);
segmentmidrightZ = segmentmidright(:,2);

segmentdownleftY = segmentdownleft(:,1);
segmentdownleftZ = segmentdownleft(:,2);

segmentdownrightY = segmentdownright(:,1);
segmentdownrightZ = segmentdownright(:,2);

pause(0.1);
hold on 
% figure
lidu = 150;
for z = 459.8:-lidu:-441
    if IsInUp(z)
        up = -0.0009554*z^2+0.1704*z+662.7;
        down = (-0.3377*z^2+810.5*z-90990)/(z-74.85);
        for y = up:-lidu:down
            plot(y,z,'.');
            hold on
        end
    end
    if IsInMid(z)
        up = -0.000818*z^2+0.0902*z+671.8;
        down = 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370;
        for y = up:-lidu:down
            plot(y,z,'.');
            hold on
        end
    end
    if IsInDown(z)
        up = 2.453*10^-5*z^3+0.02358*z^2+8.212*z+1572;
        down = -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3;
        for y = up:-lidu:down
            plot(y,z,'.');
            hold on
        end
    end
    pause(0.1);
end

%%
global yudu;
for times=1:1
    %中间空洞的曲面方程
    
    yudu = 8; %安全裕度
%     P0 = [469.6599 -407.6783  234.6203];
%     P1 = [486.2498  251.1860  441.1052];
%     P0 =[-39.2186 -612.9492  188.5821];
%     P1 = [-186.8685 -525.1849  426.7955];
%     P0 = [ -312.0857 -402.1795  227.4095];
%     P1 = [  384.7074 -211.5290   97.1707];
%     P0 = [-157.1233  541.9149  320.3161];
%     P1 = [ -512.0443  128.0374 -189.1682];
%     P0 = [ 144.2523   93.1530 -299.0419];
%     P1 = [397.1081  467.1360  301.1011];
%     P1 = [286.8212  -10.8620 -207.7470];
%     P0 = [139.3996  596.3981  317.4843];
    P0 = RandGenratePointInWorkSpace(1);
    P1 = RandGenratePointInWorkSpace(1);
%     P1 = [25.3962 -166.7241 -290.1079];
%     P0 = [ -231.0167  413.1952  202.9482];
%     P0 = [-56.9599  556.3866  397.9286];
%     P1 = [ -184.3558 -288.7860 -111.3590];
%     P0 =[ -136.7768   90.0319 -276.0100];
%     P1 = [35.3672  372.5554  -25.3049];
    times
    YESDirectReachable = IsDirectReachable(P0,P1);
    
%     pause(0.1);
end


  
figure
PointsSet = GenratePointInner(2500);
% PointsSet = GenratePointOuter(2500);
plot3(PointsSet(:,1),PointsSet(:,2),PointsSet(:,3),'.');
% pause(0.1);
% hold on
% PointsSet = GenratePointOuter(1500);
% plot3(PointsSet(:,1),PointsSet(:,2),PointsSet(:,3),'.');
xlabel('x');
ylabel('y');
zlabel('z');


%%
%Planning
%joint space
% PointA = [297.1008   40.1643 -433.8036];
% PointB = [  424.5138   63.3608 -433.8036];
% PointA = [463.7004  241.4290  232.3442];
% PointB = [559.8579  294.7425  232.3442];

% PointA = [-369.7680  429.8885  237.5011]; %这组数据没解决
% PointB = [-198.6941  240.4567 -424.4452];

% PointA = [ -391.7412   65.5419   71.7357];
% PointB = [ -633.5337   97.4666   71.7357];

% PointA = [485.3701  -70.5901  -79.9666];
% PointB = [644.7731  -89.2429   61.6302];
% [PointA,PointB] = RandGenratePointLineParallelground();
PointA = [246.1835  433.8464 -198.6287];
PointB = [270.0888  478.8083  189.0505];
% [PointA,PointB] = RandGenratePointDirectLine();
hold on
plot3([PointA(1) PointB(1)],[PointA(2) PointB(2)],[PointA(3) PointB(3)],'o');
% PointsSequence = LinearDigPlanningParallel2ground(PointA,PointB,-100,30,30,30,30,5,15,15);
% PointsSequence = LinearDigPlanningRandomLine(PointA,PointB,-100,30,50,50,50,25,25,25);
PointsSequence = LinearDigPlanningRandomLine(PointA,PointB,-100,30,150,150,150,15,25,25);
% LinearDigPlanningRandomLine(StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4)
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
% figure
% plot(cartesianSequence(1,:),cartesianSequence(2,:),'r-');
% hold on
% plot(cartesianSequence(1,:),cartesianSequence(3,:),'b-');
% hold on
% plot(cartesianSequence(1,:),cartesianSequence(4,:),'y-');
% hold on
% 
% figure
for i=1:10:size(cartesianSequence,2)
    plot3(cartesianSequence(2,i),cartesianSequence(3,i),cartesianSequence(4,i),'.');
    hold on
    pause(0.1);
end
% plot3(PlotSequence(1,:),PlotSequence(2,:),PlotSequence(3,:),'.');











% BeginPoint=[-85.1083  305.1562 -234.3325];
% EndPoint = [-505.7061  -19.8488  209.6305];
BeginPoint = RandGenratePointInWorkSpace(1);
EndPoint = RandGenratePointInWorkSpace(1);

% PointsSequence = ManipulatorPlanningJointSpace(theta0,thetaf,tf,AMAX,VMAX,SampleTime)
jointAngleBegin = InverseKinematicsPos2Angle(BeginPoint');
jointAngleEnd = InverseKinematicsPos2Angle(EndPoint');
for i=1:size(jointAngleBegin,2)
    PointsAngleSequence{i} = ManipulatorPlanningJointSpace(jointAngleBegin(i),jointAngleEnd(i),10,500,30,0.1);
end
Combination = CombineAngleSequences(PointsAngleSequence);
recordEmptyi = [];
flagrecord = 1;
for i =1:size(Combination,2)
    k4_1 = -(Combination(3,i)+Combination(4,i));
    k4_2 = -180-(Combination(3,i)+Combination(4,i));
    k4result = [];
    if k4_1>=-100&&k4_1<=30
         k4result = [k4result;k4_1];
    end
    if k4_2>=-100&&k4_2<=30
         k4result = [k4result;k4_2];
    end
    if isempty(k4result)==1 
        k4result = 0;
        if flagrecord == 1
            recordEmptyi = [recordEmptyi i]
            flagrecord = 0;
        end
        
%         if i~=1 
%             last = Combination(5,i-1)
%             if abs(last+100)<abs(last-30)
%                 k4result = -100;
%             else
%                 k4result = 30;
%             end
% %             next = Combination(5,i+1)
%         else
%             k4result = 30;
%         end
    else
        if flagrecord == 0
            flagrecord = 1;
            recordEmptyi = [recordEmptyi i-1]
        end
    end
    Combination(5,i) = k4result;
end
if mod(size(recordEmptyi,2),2)==1
    recordEmptyi = [recordEmptyi size(Combination,2)];
end
initk4 = 30;
for i=1:size(recordEmptyi,2)/2
    leftnum = recordEmptyi(2*i-1);
    rightnum = recordEmptyi(2*i);
    if leftnum == 1 && rightnum == size(Combination,2)
        for j=1:size(Combination,2)
            Combination(5,j) = initk4;
        end
    else
        if leftnum == 1
            LEFT = initk4;
            RIGHT = Combination(5,rightnum+1);
        else
            if rightnum == size(Combination,2)
                LEFT = Combination(5,leftnum-1);
                RIGHT = LEFT;
            else
                LEFT = Combination(5,leftnum-1);
                RIGHT = Combination(5,rightnum+1);
            end
        end 
        
        for j=leftnum:rightnum
            Combination(5,j) = LEFT+(j-leftnum)*((RIGHT-LEFT)/(rightnum-leftnum));
        end
    end
end

% PointsSequence = [];
% hold on;
% for i = 1:size(Combination,2)
%     [position1,position2] = ForwardKinematics(Combination(2:5,i));
%     plot3(position1(1,4),position1(2,4),position1(3,4),'o');
%     plot3([position1(1,4),position2(1,4)],[position1(2,4),position2(2,4)],[position1(3,4),position2(3,4)],'-');
%     pause(0.1)
% end
% 
% 
% plot(PointsSequence(1,:),PointsSequence(2,:),'.');

position=[-7.9299606329480514e-07 -1.7005665642058626e-06 0.99999999999823963 13.700648789105546;0.34201612592369207 -0.93969408298998514 -1.3267948966763650e-06 156.68473668902345;0.93969408299058710 0.34201612592203784 1.3267948966775328e-06 -274.40499567516673;0 0 0 1];
jointAngle = InverseKinematics(position)


% yuzhithis = 0.0001
% for k0 = -179:180
%     for k1=-40:1:44
%         for k2 = -130:1:-20
%             for k3=-100:30
%                 [position1,position2] = ForwardKinematics([k0 k1 k2 k3]);
%                 if abs(position1(1,1))<yuzhithis && abs(position1(1,2))<yuzhithis && abs(position1(1,3))-1<yuzhithis && ...
%                         abs(position1(2,3))<yuzhithis && abs(position1(3,3))<yuzhithis && position1(2,1)==position1(3,2) && ...
%                             abs(position1(3,1)+position1(2,2))<yuzhithis && cos(asin(position1(2,1)))==position(3,1)
%                         disp(';');
%                 end
%             end
%         end
%     end
% end

%cartesian space
PointsSequence = ManipulatorPlanningcartesian(BeginPoint,EndPoint)
hold on
plot3(PointsSequence(:,1),PointsSequence(:,2),PointsSequence(:,3),'-');













%%
%functions
function [PointsSequence,theta4sequence] = LinearDigPlanningRandomLine(StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4)%解决任意可直达直线挖掘
  %PointsSequence 存储theta1 theta2 theta3 随时间变化的序列
    if IsDirectReachable(StartPoint,EndPoint) == 0
        PointsSequence = [];
        theta4sequence = [];
        error('输入的两点不是直接的直线可达！');
        return;
    end
    begindistance = norm(StartPoint-EndPoint)
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
%     Chazhi = [];
%     X3 = [];
%     X2 = [];
    flagSlowDown = 0;
figure
    while norm(CurrentPoint-EndPoint)>1
        num = num+1;
        PointsSequence(1,num) = tcurrent;
        PointsSequence(2,num) = theta1;
        PointsSequence(3,num) = theta2;
        PointsSequence(4,num) = theta3;
        
        [vtheta2,vtheta3] = GetCurrentvtheta3RandomLine(jointAngle,StartPoint,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3);
        x3 = vtheta3^2/(2*amaxtheta3)+vtheta3*tinterval/2;
%         x2 = vtheta2^2/(2*amaxtheta2)+vtheta2*tinterval/2;
%          Chazhi =  jointAngleEnd(3)-jointAngle(3);
      
        subplot(131)
        plot(num,vtheta2,'.');
        hold on 
        subplot(133)
        plot(num,norm(CurrentPoint-EndPoint),'.');
        hold on 
        subplot(132)
        plot(num,vtheta3,'.');
        hold on
        pause(0.1);

        if abs(x3-(jointAngleEnd(3)-jointAngle(3)))<2 %&& norm(CurrentPoint-EndPoint)<10
            if norm(CurrentPoint-EndPoint)<100
                flagSlowDown = 1;
            end
        end
%         Chazhi = [Chazhi jointAngleEnd(3)-jointAngle(3)];
%         X3 = [X3;x3];
%         X2 = [X2;x2];
        
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
        norm(CurrentPoint-EndPoint)
        if flagSlowDown == 1
            break; %之后就开始减速 以两个关节都能承受的加速度进行减速，只不过最后可能会超出末位置 如果是以最大值进行减速，则正好到目标点，否则会超过目标点。要尽可能使加速度达到最大！
        end
    end
    
%     plot(Chazhi)
%     plot(X3)
%     hold on 
%     plot(X2)

    if flagSlowDown == 1
        while abs(vtheta3)>0.1
            k1 = theta1*pi/180;
            k2 = theta2*pi/180;
            k3 = theta3*pi/180;
            acurrenttheta2 = amaxtheta2;
            acurrenttheta3 = amaxtheta3;
            if x0 ~= x1
                if amaxtheta2 < abs(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))/((- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1)))*amaxtheta3
    
                    %说明amaxtheta2 带不动amaxtheta3
                    acurrenttheta3 = acurrenttheta2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))/((- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1)));
                    acurrenttheta3 = abs(acurrenttheta3);
                    disp('要注意铲斗不是精确到目标位置!要想精确，首先提高加速度约束的大小');
                end
            else
                if y0~=y1
                    if amaxtheta2 <abs(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))/((- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1)))
                        acurrenttheta3 = acurrenttheta2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))/((- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1)));
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
                d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);

            else
                if y0~=y1
                    d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))*d_k3/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
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
        end
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
    enddistance = norm(CurrentPoint-EndPoint) %这是误差，经过少量测试，误差不会超过2cm
end

function [vtheta2,vtheta3] = GetCurrentvtheta3RandomLine(CurrentJointAngle,StartPoint,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3)%输出满足加速度限制的当前时刻速度值
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
    k1 = theta1; k2 = theta2; k3 = theta3;
    
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
    for i=1:2
        d_k3 = Fita_Vtheta3(i);
%         d_k3 = fuhao * vtheta3 * pi/180;
        if x0~=x1
            d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
        else
            if y0~=y1
                d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))*d_k3/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
            end
        end
%         d_k2 = fuhao * (-(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)));

        Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
        Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
        Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;
        if dot(EndPoint-CurrentPoint,[Vx,Vy,Vz])<0
            fuhao = -fuhao;
            d_k3 = -d_k3;
            d_k2 = -d_k2;
            Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
            Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
            Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;
            if dot(EndPoint-CurrentPoint,[Vx,Vy,Vz])<0
                error('');
            end
        end
        d_k2Storage = [d_k2Storage d_k2*180/pi];
        if d_k2>=Fita_Vtheta2Lower-0.01 && d_k2<=Fita_Vtheta2Upper+0.01
            vtheta3 = d_k3*180/pi;
            vtheta2 = d_k2*180/pi;
            Vtheta2set = [Vtheta2set vtheta2];
            Vtheta3set = [Vtheta3set vtheta3];
            flagYESsolution = flagYESsolution+1;
%             break;
        end
%         CurrentPoint = CurrentPoint+tinterval*[Vx Vy 0];
        a_k2 = abs((d_k2-lastd_k2)/tinterval);
        a_k3 = abs((d_k3-lastd_k3)/tinterval);
    end
%     if flagYESsolution == 2
%         Vtheta2set
%         Vtheta3set
%     end
    if flagYESsolution == 0
        error('需要重新修改相关参数，这组参数无法满足加速度约束');
    else
        if flagYESsolution == 1
            %20200716 这个必须得考虑一个区间
            vtheta2 = Vtheta2set(1);
            vtheta3 = Vtheta3set(1);
        else
            kthis = 0.9;
            vtheta2 = GetVelocityInRange(Vtheta2set(1),Vtheta2set(2),kthis);
            vtheta3 = GetVelocityInRange(Vtheta3set(1),Vtheta3set(2),kthis);
            %20200716 这里有bug 对应关系
        end
    end
    
%     while 1
%         if flagonce == 0
%             if vtheta3Last<0
%                 vtheta3 = -k_yuzhi*Vtheta3range; %先设置为允许的留有一定裕度的最大值
%             else
%                 vtheta3 = k_yuzhi*Vtheta3range;
%             end
%             flagonce = 1;
%         else
%             if abs(vtheta3)<abs(vtheta3Last) && abs(vtheta3Last)-abs(vtheta3Last)>0.001
%                 if Vtheta3range>=abs(vtheta3Last)
%                     upper = abs(vtheta3Last);
%                 else
%                     upper = Vtheta3range;
%                 end
%                 vtheta3tmp = abs(vtheta3) + xishu*(upper-abs(vtheta3));
%             else
%                 if abs(abs(vtheta3)-abs(vtheta3Last))<0.001
%                     vtheta3tmp = abs(vtheta3) - xishu*(abs(vtheta3)-abs( 0.5*vtheta3Last));
%                 else
%                     vtheta3tmp = abs(vtheta3) - xishu*(abs(vtheta3)-abs( vtheta3Last));
%                 end
%             end
%             if vtheta3<0
%                vtheta3 = -vtheta3tmp;
%             else
%                vtheta3 = vtheta3tmp;
%             end
%         end
% 
%         d_k3 = fuhao * vtheta3 * pi/180;
%         if x0~=x1
%             d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
%         else
%             if y0~=y1
%                 d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))*d_k3/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
%             end
%         end
% %         d_k2 = fuhao * (-(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)));
% 
%         Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
%         Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
%         Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;
%         if dot(EndPoint-CurrentPoint,[Vx,Vy,Vz])<0
%             fuhao = -fuhao;
%             d_k3 = -d_k3;
%             d_k2 = -d_k2;
%             Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
%             Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
%             Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;
%             if dot(EndPoint-CurrentPoint,[Vx,Vy,Vz])<0
%                 error('');
%             end
%         end
% %         CurrentPoint = CurrentPoint+tinterval*[Vx Vy 0];
%         a_k2 = abs((d_k2-lastd_k2)/tinterval);
%         a_k3 = abs((d_k3-lastd_k3)/tinterval);
%         if abs(a_k2-lasta_k2)<0.001 && abs(a_k3-lasta_k3)<0.000001
%             error('需要重新修改相关参数，这组参数无法满足加速度约束');
%             break;
%         end
%         lasta_k2 = a_k2;
%         lasta_k3 = a_k3;
%         if a_k2<=amaxtheta2 && a_k3<=amaxtheta3
%             vtheta3 = d_k3*180/pi;
%             vtheta2 = d_k2*180/pi;
%             break;
%         end
%     end
end

function out = GetIntersection(qujianA,qujianB)
%20200717 写这个交集函数
    if qujianA(1)>qujianA(2)
        tmp = qujianA(1);
        qujianA(1) = qujianA(2);
        qujianA(2) = tmp;
    end
    if qujianB(1)>qujianB(2)
        tmp = qujianB(1);
        qujianB(1) = qujianB(2);
        qujianB(2) = tmp;
    end
    if qujianA(2)<qujianB(1)
        out = [];
    else
        if qujianA(2)==qujianB(1)
            out = qujian(1);
        else
            if qujianA(1)<=qujianB(1)
                out = [qujianB(1),qujianA(2)];
            end
        end
    end
end

function v = GetVelocityInRange(Lower,Upper,k) %k=1时，走满足范围的最大的绝对值速度 k=0时，走0
    if Lower>Upper
        tmp =Upper;
        Upper = Lower;
        Lower = tmp;
    end
    absLower = abs(Lower);
    absUpper = abs(Upper);
    if absUpper>=absLower
        v_value = k*absUpper;
        if Upper<0
            v = -v_value;
        else
            v = v_value;
        end
    else
        v_value = k*absLower;
        if Lower<0
            v = -v_value;
        else
            v = v_value;
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
        Vtheta2tmp = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
        d_k2 = Vmaxtheta2;
        Vtheta3tmp = (- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1))));
    else
        if y0~=y1
            Vtheta2tmp = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)))*d_k3/(- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1);
            d_k2 = Vmaxtheta2;
            Vtheta3tmp = (- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1))));
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

function [PointsSequence,theta4sequence] = LinearDigPlanningParallel2ground(StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4)%只能解决平行于水平地面的直线挖掘
  %PointsSequence 存储theta1 theta2 theta3 随时间变化的序列
    begindistance = norm(StartPoint-EndPoint)
    PointsSequence = [];
    tinterval = 0.01; %匀速运动时间是0.01秒
    angleinterval = 1; %每次关节运动1度
    tcurrent = 0;

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
%     Chazhi = [];
%     X3 = [];
%     X2 = [];
    flagSlowDown = 0;

    while norm(CurrentPoint-EndPoint)>1
        num = num+1;
        PointsSequence(1,num) = tcurrent;
        PointsSequence(2,num) = theta1;
        PointsSequence(3,num) = theta2;
        PointsSequence(4,num) = theta3;
        
        [vtheta2,vtheta3] = GetCurrentvtheta3(jointAngle,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3);
        x3 = vtheta3^2/(2*amaxtheta3)+vtheta3*tinterval/2;
%         x2 = vtheta2^2/(2*amaxtheta2)+vtheta2*tinterval/2;
        
        if abs(x3-(jointAngleEnd(3)-jointAngle(3)))<1
            flagSlowDown = 1;
        end
%         Chazhi = [Chazhi jointAngleEnd(3)-jointAngle(3)];
%         X3 = [X3;x3];
%         X2 = [X2;x2];
        
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
        
        if flagSlowDown == 1
            break; %之后就开始减速 以两个关节都能承受的加速度进行减速，只不过最后可能会超出末位置 如果是以最大值进行减速，则正好到目标点，否则会超过目标点。要尽可能使加速度达到最大！
        end
    end
%     figure
%     plot(Chazhi)
%     plot(X3)
%     hold on 
%     plot(X2)

    if flagSlowDown == 1
        while abs(vtheta3)>0.1
            k2 = theta2*pi/180;
            k3 = theta3*pi/180;
            acurrenttheta2 = amaxtheta2;
            acurrenttheta3 = amaxtheta3;
            if amaxtheta2 < abs((2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2))*amaxtheta3)
                %说明amaxtheta2 带不动amaxtheta3
                acurrenttheta3 = acurrenttheta2/((2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)));
                acurrenttheta3 = abs(acurrenttheta3);
                disp('要注意铲斗不是精确到目标位置!要想精确，首先提高加速度约束的大小');
            end
            if vtheta3 < 0
                vtheta3 = vtheta3 + acurrenttheta3*tinterval;
            else
                vtheta3 = vtheta3 - acurrenttheta3*tinterval;
            end
            d_k3 = vtheta3*pi/180;
            d_k2 = -(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)); %这是通过求雅可比矩阵得到的对应关系
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
        end
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
    enddistance = norm(CurrentPoint-EndPoint) %这是误差，经过少量测试，误差不会超过2cm
end

function thetaout = legalizAnger(theta) %把角度限制在(-180,180]
    thetaout = theta;
    while theta>180 
        thetaout = theta-360;
    end
    while theta<=-180
        thetaout = theta+360;
    end
end

function [vtheta2,vtheta3] = GetCurrentvtheta3(CurrentJointAngle,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3)%输出满足加速度限制的当前时刻速度值
    amaxtheta2 = amaxtheta2*pi/180;
    amaxtheta3 = amaxtheta3*pi/180;
    fuhao = 1;
    lastd_k2 = vtheta2Last*pi/180;
    lastd_k3 = vtheta3Last*pi/180;
    flagonce = 0;
    xishu = 0.3; %每次迭代的缩放系数
    k_yuzhi = 0.9; %先设置为允许的留有一定裕度的最大值
   
%      jointAngle = InverseKinematicsPos2Angle(CurrentPoint);
    theta1 = CurrentJointAngle(1);
    theta2 = CurrentJointAngle(2);
    theta3 = CurrentJointAngle(3);
    [pos1,pos2] = ForwardKinematics([theta1 theta2 theta3 0]);
    CurrentPoint = pos1(1:3,4);
    CurrentPoint = CurrentPoint';
    
    [Vtheta2range,Vtheta3range] = LinearDiggingGetRange(Vmaxtheta2,Vmaxtheta3,theta2,theta3);
    
    theta1 = theta1*pi/180;
    theta2 = theta2*pi/180;
    theta3 = theta3*pi/180;
    k1 = theta1; k2 = theta2; k3 = theta3;
    
    lasta_k2 = 0;
    lasta_k3 = 0;
    while 1
        if flagonce == 0
            if vtheta3Last<0
                vtheta3 = -k_yuzhi*Vtheta3range; %先设置为允许的留有一定裕度的最大值
            else
                vtheta3 = k_yuzhi*Vtheta3range;
            end
            flagonce = 1;
        else
            if abs(vtheta3)<abs(vtheta3Last)
                if Vtheta3range>=abs(vtheta3Last)
                    upper = abs(vtheta3Last);
                else
                    upper = Vtheta3range;
                end
                vtheta3tmp = abs(vtheta3) + xishu*(upper-abs(vtheta3));
            else
                vtheta3tmp = abs(vtheta3) - xishu*(abs(vtheta3)-abs(vtheta3Last));
            end
            if vtheta3<0
               vtheta3 = -vtheta3tmp;
            else
               vtheta3 = vtheta3tmp;
            end
        end

        d_k3 = fuhao * vtheta3 * pi/180;
        d_k2 = fuhao * (-(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)));

        Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
        Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
        if dot(EndPoint-CurrentPoint,[Vx,Vy,0])<0
            fuhao = -fuhao;
            d_k3 = -d_k3;
            d_k2 = -d_k2;
            Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
            Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
            if dot(EndPoint-CurrentPoint,[Vx,Vy,0])<0
                error('');
            end
        end
%         CurrentPoint = CurrentPoint+tinterval*[Vx Vy 0];
        a_k2 = abs((d_k2-lastd_k2)/tinterval);
        a_k3 = abs((d_k3-lastd_k3)/tinterval);
        if abs(a_k2-lasta_k2)<0.001 && abs(a_k3-lasta_k3)<0.000001
            error('需要重新修改相关参数，这组参数无法满足加速度约束');
            break;
        end
        lasta_k2 = a_k2;
        lasta_k3 = a_k3;
        if a_k2<=amaxtheta2 && a_k3<=amaxtheta3
            vtheta3 = d_k3*180/pi;
            vtheta2 = d_k2*180/pi;
            break;
        end
    end
end

function [Vtheta2,Vtheta3] = LinearDiggingGetRange(Vmaxtheta2,Vmaxtheta3,theta2,theta3)% 输出每个theta2 theta3对应的满足角速度约束的角速度的绝对值的最大值 只有这样的角速度才能满足轨迹是直线，且直线平行于地面
 %要保证输入Vmax是正值
    Vmaxtheta2 = Vmaxtheta2*pi/180;
    Vmaxtheta3 = Vmaxtheta3*pi/180;
    theta2 = theta2*pi/180;
    theta3 = theta3*pi/180;
    
    d_k3 = Vmaxtheta3;
    k2 = theta2;
    k3 = theta3;
    Vtheta2tmp = -(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)); %这是通过求雅可比矩阵得到的对应关系
    d_k2 = Vmaxtheta2;
    Vtheta3tmp = -((2109*cos(k2 + k3) + 4600*cos(k2))/(2109*cos(k2 + k3)))*d_k2;
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

function Combination = CombineAngleSequences(PointsAngleSequence) %把每个独立的角度序列组合在一起 必须保证待组合的序列长度相同且时间跨度相同
    Combination = PointsAngleSequence{1}(1,:);
    for i = 1:size(PointsAngleSequence,2)
        Combination = [Combination;PointsAngleSequence{i}(2,:)];
    end
end

function matrixtool = MatrixTool(tool)
	matrixtool(1,1) = 1.0;
	matrixtool(1,2) = 0.0;
	matrixtool(1,3) = 0.0;
	matrixtool(1,4) = tool;

	matrixtool(2,1) = 0.0;
	matrixtool(2,2) = 1.0;
	matrixtool(2,3) = 0.0;
	matrixtool(2,4) = 0.0;

	matrixtool(3,1) = 0.0;
	matrixtool(3,2) = 0.0;
	matrixtool(3,3) = 1.0;
	matrixtool(3,4) = 0.0;

	matrixtool(4,1) = 0.0;
	matrixtool(4,2) = 0.0;
	matrixtool(4,3) = 0.0;
	matrixtool(4,4) = 1.0;
end

function TransMatrix = MatrixT(alpha, a, theta, d )
	TransMatrix(1,1) = cos(theta);
	TransMatrix(1,2) = -sin(theta);
	TransMatrix(1,3) = 0.0;
	TransMatrix(1,4) = a;

	TransMatrix(2,1) = sin(theta)*cos(alpha);
	TransMatrix(2,2) = cos(theta)*cos(alpha);
	TransMatrix(2,3) = -sin(alpha);
	TransMatrix(2,4) = -sin(alpha)*d;

	TransMatrix(3,1) = sin(theta)*sin(alpha);
	TransMatrix(3,2) = cos(theta)*sin(alpha);
	TransMatrix(3,3) = cos(alpha);
	TransMatrix(3,4) = cos(alpha)*d;

	TransMatrix(4,1) = 0.0;
	TransMatrix(4,2) = 0.0;
	TransMatrix(4,3) = 0.0;
	TransMatrix(4,4) = 1.0;
end

% //position1为铲斗旋转中心点坐标，position2为铲齿最末端点坐标
% void ForwardKinematics(double *jointangle, double *position1, double *position2)
function [position1,position2] = ForwardKinematics(jointangle)
% 	double m[4], a[4], theta[4], d[4], tool;
% 	double m_matrix1[16], m_matrix2[16], m_matrix3[16], m_matrix4[16], m_matrix5[16];
% 	double m_matrixtemp1[16], m_matrixtemp2[16], m_matrixtemp3[16], m_matrixtemp4[16];
% 	double m_dRPY[3];
    ZERO = 10^-6;
    M_PI = pi;
    M_PI_2 = pi/2;
	m(1) = 0.0*M_PI / 180.0;
	m(2) = 90.0*M_PI / 180.0;
	m(3) = 0.0*M_PI / 180.0;
	m(4) = 0.0*M_PI / 180.0;

	a(1) = 0.0;
	a(2) = 12.0;
	a(3) = 460.0;
	a(4) = 210.9;

	d(1) = 57.9;
	d(2) = 13.7;%//13.7
	d(3) = 0.0;
	d(4) = 0.0;
	tool = 123.5;

	theta(1) = jointangle(1) * M_PI / 180.0;
	theta(2) = jointangle(2) * M_PI / 180.0;
	theta(3) = jointangle(3) * M_PI / 180.0;
	theta(4) = jointangle(4) * M_PI / 180.0;

	m_matrix1 = MatrixT(m(1), a(1), theta(1), d(1));
	m_matrix2 = MatrixT(m(2), a(2), theta(2), d(2));
	m_matrix3 = MatrixT(m(3), a(3), theta(3), d(3));
	m_matrix4 = MatrixT(m(4), a(4), theta(4), d(4));
	m_matrix5 = MatrixTool(tool);

    m_matrixtemp1 = m_matrix1*m_matrix2;
    m_matrixtemp2 = m_matrixtemp1*m_matrix3;
    m_matrixtemp3 = m_matrixtemp2*m_matrix4;
    m_matrixtemp4 = m_matrixtemp3*m_matrix5;
	

% 	//欧拉角A、B、C begin
	m_dRPY(2) = mathAtan2(-m_matrixtemp4(3,1), sqrt(m_matrixtemp4(1,1) * m_matrixtemp4(1,1) + m_matrixtemp4(2,1) * m_matrixtemp4(2,1)));%//欧拉角B

    if ((m_dRPY(2)<M_PI_2 + ZERO) && (m_dRPY(2)>M_PI_2 - ZERO))
        m_dRPY(3) = mathAtan2(m_matrixtemp4(1,2), m_matrixtemp4(2,2));
		m_dRPY(1) = 0.0;
    else
        if ((m_dRPY(2)<-M_PI_2 + ZERO) && (m_dRPY(2)>-M_PI_2 - ZERO))
            m_dRPY(3) = -mathAtan2(m_matrixtemp4(1,2), m_matrixtemp4(2,2));%//欧拉角A
            m_dRPY(1) = 0.0;%//欧拉角C
        else
            m_dRPY(3) = mathAtan2(m_matrixtemp4(2,1) / cos(m_dRPY(2)), m_matrixtemp4(1,1) / cos(m_dRPY(2)));%//欧拉角A
            m_dRPY(1) = mathAtan2(m_matrixtemp4(3,2) / cos(m_dRPY(2)), m_matrixtemp4(3,3) / cos(m_dRPY(2)));%//欧拉角C
        end
    end

% 	//欧拉角A、B、C end

% 	//printf("\nEuler%f %f %f\n", m_dRPY[0] * 180.0 / M_PI, m_dRPY[1] * 180.0 / M_PI, m_dRPY[2] * 180.0 / M_PI);

    position1 = m_matrixtemp3;%//铲斗旋转中心坐标
    position2 = m_matrixtemp4;%//末端铲齿坐标
end

function result = mathAtan2(y,x)
    ZERO = 10^-6;
    if (abs(y) < ZERO)
		y = 0.0;
    end
    if (abs(x) < ZERO)
        x = 0.0;
    end
    result = atan2(y, x);
end

function result = pow(a,b)
    result = a^b;
end

function jointAngle = InverseKinematicsPos2Angle(position)%输入为列向量，输出为前三个角的角度 为横向量
    ZERO = 10^-6;
    M_PI = pi;
%     nx, ny, nz;
%     ox, oy, oz;
% 	ax, ay, az;
% 	px, py, pz;
%     m[4], a[4], theta[4], d[4], tool;
% 	mTempAngleOne[2];
    m(1) = 0.0*M_PI / 180.0;
	m(2) = 90.0*M_PI / 180.0;
	m(3) = 0.0*M_PI / 180.0;
	m(4) = 0.0*M_PI / 180.0;
    
    a(1) = 0.0;
	a(2) = 12.0;
	a(3) = 460.0;
	a(4) = 210.9;
	tool = 123.5;

	d(1) = 57.9;
	d(2) = 13.7;%//13.7
	d(3) = 0.0;
	d(4) = 0.0;
    
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

function jointAngle = InverseKinematics(position)
    ZERO = 10^-6;
    M_PI = pi;
%     nx, ny, nz;
%     ox, oy, oz;
% 	ax, ay, az;
% 	px, py, pz;
%     m[4], a[4], theta[4], d[4], tool;
% 	mTempAngleOne[2];
    m(1) = 0.0*M_PI / 180.0;
	m(2) = 90.0*M_PI / 180.0;
	m(3) = 0.0*M_PI / 180.0;
	m(4) = 0.0*M_PI / 180.0;
    
    a(1) = 0.0;
	a(2) = 12.0;
	a(3) = 460.0;
	a(4) = 210.9;
	tool = 123.5;

	d(1) = 57.9;
	d(2) = 13.7;%//13.7
	d(3) = 0.0;
	d(4) = 0.0;
    
    nx = position(1,1);
	ny = position(2,1);
	nz = position(3,1);

	ox = position(1,2);
	oy = position(2,2);
	oz = position(3,2);

	ax = position(1,3);
	ay = position(2,3);
	az = position(3,3);

	px = position(1,4);
	py = position(2,4);
	pz = position(3,4);

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
	tempfour1 = -(cos(jointAngle(1))*cos(jointAngle(2) + jointAngle(3))*ox + sin(jointAngle(1))*cos(jointAngle(2) + jointAngle(3))*oy + sin(jointAngle(2) + jointAngle(3))*oz);
	tempfour2 = -cos(jointAngle(1))*sin(jointAngle(2) + jointAngle(3))*ox - sin(jointAngle(1))*sin(jointAngle(2) + jointAngle(3))*oy + cos(jointAngle(2) + jointAngle(3))*oz;
	jointAngle(4) = mathAtan2(tempfour1, tempfour2);

    for i=1:4
        jointAngle(i) = jointAngle(i) * 180.0 / M_PI;
    end
end

function [PointsSequence,success] = ManipulatorPlanningJointSpace(theta0,thetaf,tf,AMAX,VMAX,SampleTime) %默认初始末点速度为0，加速度为0;
%-180,180 角度体系
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
        amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
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
        amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
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
            if PointsSequence(2,i)>180
                PointsSequence(2,i) = -180 + (PointsSequence(2,i)-180);
            end
        end
    end
    success = 1;
end

function [Sequenceout,YES] = BFS(MapRelation,beginnum,endnum)
    Sequenceout = [];
    Sequence = [];
    YES = 0;
    if size(MapRelation{beginnum},2)==1 || size(MapRelation{endnum},2)==1
        return;
    end
    index = 1:1:size(MapRelation,1);
    isvisited = zeros(1,size(MapRelation,1));
    Queue = [];
    Queue = beginnum;
    index(2,MapRelation{beginnum}(2:end)) = beginnum;
    isvisited(beginnum) = 1;
%     prePointer = beginnum;
    
    while isempty(Queue)==0 
        temp = find ( isvisited(MapRelation{Queue(1)}(2:end))==0 );
        add = MapRelation{Queue(1)}(2:end);
        Queue = [Queue add(temp)];
        isvisited(add(temp))=1;

        index(2,add(temp)) = Queue(1);
        if Queue(1) == endnum
            prePointer = index(2,endnum);
            Sequence = [Sequence prePointer];
            while prePointer~=beginnum
                prePointer = index(2,prePointer);
                Sequence = [Sequence prePointer];
            end
            break;
        end
        Queue(1) = [];
    end
    if isempty(Sequence) == 1
        YES = 0;
    else
        YES = 1;
%         Sequenceout = beginnum;
        for i = size(Sequence,2):-1:1
            Sequenceout = [Sequenceout Sequence(i)];
        end
        Sequenceout = [Sequenceout endnum];
    end
end

function PointsSequence = ManipulatorPlanningcartesian(BeginPoint,EndPoint) %笛卡尔坐标系的规划
    PointsSequence = [];
    if IsDirectReachable(BeginPoint,EndPoint) == 1
        PointsSequence = [BeginPoint;EndPoint];
    else
        AllPoints = [BeginPoint;EndPoint];
        if IsDirectReachable(BeginPoint,EndPoint)==1
            MapRelation = [{[1 2]};{[2 1]}];
        else
            MapRelation = [{1};{2}];
        end
        
        while 1
            Point = RandGenratePointInWorkSpace(1);
            AllPoints = [AllPoints;Point];
%             figure(5)
%             hold on
%             plot3( Point(1) , Point(2)  ,Point(3),'o');
            MapRelation = [MapRelation;size(MapRelation,1)+1];
            for i=size(MapRelation,1)-1:-1:1
                i
                d = 1.5*norm(BeginPoint - EndPoint);
                if norm(AllPoints(end,:)-AllPoints(i,:))<d
                    if IsDirectReachable(AllPoints(end,:),AllPoints(i,:))==1
                        MapRelation{end} = [MapRelation{end} i];
                        MapRelation{i} = [MapRelation{i} size(MapRelation,1)];
                    end
                end
            end
            [Sequence,YES] = BFS(MapRelation,1,2);
            if YES == 1
                PointsSequence = AllPoints(Sequence,:);
                break;
            end
        end
        
        
        
        
        
%         MapRelation = 1;
%         Point = RandGenratePointInWorkSpace(1);
%         AllPoints = [AllPoints;Point];
%         MapRelation = [MapRelation;2];
%         for i = size(AllPoints,1)-1:-1:1
%             if IsDirectReachable(AllPoints(end,:),AllPoints(i,:))==1
%                 
%             end
%         end
%         while ~(IsDirectReachable(Point,BeginPoint)==1 && IsDirectReachable(Point,EndPoint)==1)
% %             a = [IsDirectReachable(Point,BeginPoint) IsDirectReachable(Point,EndPoint)]
%             Point = RandGenratePointInWorkSpace(1);
%             AllPoints = [AllPoints;Point];
%         end
%         AllPoints = [AllPoints;EndPoint];
%         PointsSequence = [BeginPoint; Point; EndPoint];
    end
end

function YESDirectReachable = IsDirectReachable(P0,P1) %YESDirectReachable = 0 时不可直达，1时可直达
    d = 13.7;
    global yudu;
    syms x y z t
    x = P0(1) + ( P1(1)-P0(1) ) * t;
    y = P0(2) + ( P1(2)-P0(2) ) * t;
    z = P0(3) + ( P1(3)-P0(3) ) * t;

    %z<=459.8 && z>=165.25
    % ( (-0.3377*z^2+810.5*z-90990)/(z-74.85) )^2 = x^2+y^2-d^2
    Fup = ( (-0.3377*z^2+810.5*z-90990)/(z-74.85) + yudu )^2 - (x^2+y^2-d^2);

    %z<165.25 && z>=-274.405
    % ( 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370 )^2 = x^2+y^2-d^2
    Fmid = ( 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370+yudu )^2 - (x^2+y^2-d^2);

    %z<-274.405 && z>=-441.0118
    % ( -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3 )^2 = x^2+y^2-d^2
    Fdown = ( -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3+yudu )^2 - (x^2+y^2-d^2);

    equationSet = []; %要求解的方程
    equationSet_trange = []; %对应的t的取值范围
    if (P0(3) == P1(3))
        P0Z = P0(3);
        if IsInUp(P0Z)
            equationSet = [equationSet;Fup];
        end
        if IsInMid(P0Z)
            equationSet = [equationSet;Fmid];
        end
        if IsInDown(P0Z)
            equationSet = [equationSet;Fdown];
        end
        trange = [];
        trange(1,1) = 0;
        trange(1,2) = 1;
        equationSet_trange = [equationSet_trange;trange];
    else
        z0 = P0(3);
        z1 = P1(3);

        Az0 = IsInUp(z0);
        Bz0 = IsInMid(z0);
        Cz0 = IsInDown(z0);
        Az1 = IsInUp(z1);
        Bz1 = IsInMid(z1);
        Cz1 = IsInDown(z1);
        BorderLines = [-441.0118 -274.405 165.25 459.8];

        if Az0 && Az1
            equationSet = [equationSet;Fup];
            equationSet_trange = [equationSet_trange;[0 1]];
        end
        if Az0 && Bz1
            equationSet = [equationSet;Fup];
            equationSet_trange = [equationSet_trange;[0 Return_tValue(z0,z1,165.25)]];
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,165.25) 1]];
        end
        if Az0 && Cz1
            equationSet = [equationSet;Fup];
            equationSet_trange = [equationSet_trange;[0 Return_tValue(z0,z1,165.25)]];
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,165.25) Return_tValue(z0,z1,-274.405)]];
            equationSet = [equationSet;Fdown];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,-274.405) 1]];
        end

        if Bz0 && Az1
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[0 Return_tValue(z0,z1,165.25)]];
            equationSet = [equationSet;Fup];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,165.25) 1]];
        end
        if Bz0 && Bz1
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[0 1]];
        end
        if Bz0 && Cz1
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[0 Return_tValue(z0,z1,-274.405)]];
            equationSet = [equationSet;Fdown];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,-274.405) 1]];
        end

        if Cz0 && Az1
            equationSet = [equationSet;Fdown];
            equationSet_trange = [equationSet_trange;[0 Return_tValue(z0,z1,-274.405)]];
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,-274.405) Return_tValue(z0,z1,165.25) ]];
            equationSet = [equationSet;Fup];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,165.25) 1]];
        end
        if Cz0 && Bz1
            equationSet = [equationSet;Fdown];
            equationSet_trange = [equationSet_trange;[0 Return_tValue(z0,z1,-274.405)]];
            equationSet = [equationSet;Fmid];
            equationSet_trange = [equationSet_trange;[Return_tValue(z0,z1,-274.405) 1]];
        end
        if Cz0 && Cz1
            equationSet = [equationSet;Fdown];
            equationSet_trange = [equationSet_trange;[0 1]];
        end
    end


    equationSet ; %要求解的方程
    equationSet_trange ; %对应的t的取值范围
    
    interval = 0.01;
    
    for i = 1:size(equationSet,1)
%         result1 = SolveEqualZero(equationSet(i),equationSet_trange(i,:));
        result2 = SolveEquavpasolve(equationSet(i),equationSet_trange(i,:));
        result3 = SolveEquaMuller(equationSet(i),equationSet_trange(i,:));
        
        if isempty(result2)==1 && result3 == 1
            figure(7)
            P0 
            P1
            result2
            result3
            plot([equationSet_trange(i,1):interval:equationSet_trange(i,2)],subs(equationSet(i),[equationSet_trange(i,1):interval:equationSet_trange(i,2)]),'-');
            disp('s');
             result3 = SolveEquaMuller(equationSet(i),equationSet_trange(i,:));
        end
        if isempty(result2)==0 && result3 == 0
            figure(7)
            P0 
            P1
            result2
            result3
            plot([equationSet_trange(i,1):interval:equationSet_trange(i,2)],subs(equationSet(i),[equationSet_trange(i,1):interval:equationSet_trange(i,2)]),'-');
            disp('s');
        end
        if size(result2,1)==3
            figure(7)
            result2
            result3
            plot([equationSet_trange(i,1):interval:equationSet_trange(i,2)],subs(equationSet(i),[equationSet_trange(i,1):interval:equationSet_trange(i,2)]),'-');
            result3 = SolveEquaMuller(-equationSet(i),equationSet_trange(i,:));
        end
        
        if result3 == 1 %说明有交点
            YESDirectReachable = 0;
            return;
        end
    end
    YESDirectReachable = 1;
end

function YES = SolveEquaMuller(equation,range)%用抛物线法求是否有交点 存在性判断
    YES = IsExistZero(equation,range);
    if YES == 1
        return;
    end
    Xk = [range(1) (range(1)+range(2))*(1/2) range(2)];
%     range
    yuzhi = 10^-3;
%     result2 = SolveEquavpasolve(equation,range)
    result = [];
    while 1
        xk = Xk(3);
        xk_1 = Xk(2);
        xk_2 = Xk(1);

        fxk = subs(equation,xk);
        fxk_1 = subs(equation,xk_1);
        fxk_2 = subs(equation,xk_2);

        omiga = DiffQuotFir(equation,xk,xk_1) + DiffQuotSec(equation,xk,xk_1,xk_2)*(xk-xk_1);
    %     fxk + DiffQuotFir(equation,xk,xk_1)*(x-xk) + DiffQuotSec(xk,xk_1,xk_2)*(x-xk)*(x-xk_1); 
        xkplus1ONE = xk - (2*fxk)/(omiga + ( omiga^2-4*fxk*DiffQuotSec(equation,xk,xk_1,xk_2) )^0.5);
        xkplus1TWO = xk - (2*fxk)/(omiga - ( omiga^2-4*fxk*DiffQuotSec(equation,xk,xk_1,xk_2) )^0.5);

        
        if omiga >=0
            xkplus1 = xkplus1ONE;
        else
            xkplus1 = xkplus1TWO;
        end
        xkplus1 = double(xkplus1);
        Xk(1) = Xk(2);
        Xk(2) = Xk(3);
        Xk(3) = xkplus1;
%         Xk
        if abs(imag(xkplus1))>0.1
            break;
        end
%         if ~isreal(xkplus1)
%             break;
%         end
        if xkplus1<range(1)-0.01 || xkplus1>range(2)+0.01
            break;
        end
        if abs(Xk(3)-Xk(2))<yuzhi 
            result = Xk(3);
            break;
        end
    end
    
    if isempty(result)==0
        if result < range(1) || result >range(2) || abs(imag(result))>0.001
            YES = 0;
        else
            YES = 1;
        end
    else
        YES = 0;
    end
%     result 
end

function DQresult = DiffQuotFir(equation,xi,xj) %一阶差商计算公式
    fxi = subs(equation,xi);
    fxj = subs(equation,xj);
    DQresult = (fxi-fxj)/(xi-xj);
end

function DQresult = DiffQuotSec(equation,xi,xj,xk) %二阶差商公式
    DQresult = (DiffQuotFir(equation,xi,xj)-DiffQuotFir(equation,xj,xk))/(xi-xk);
end

function YES = IsExistZero(equation,range) %判断与横轴交点的存在性 如果返回1，则肯定有交点，如果返回0，则不确定有没有交点。
    DuanDian1 = subs(equation,range(1));
    DuanDian2 = subs(equation,range(2));
    yuzhi = 10^-3;
    if DuanDian1>yuzhi
        Pan1 = 1;
    else
        if DuanDian1<-yuzhi
            Pan1 = -1;
        else
            Pan1 = 0;
        end
    end
    
    if DuanDian2>yuzhi
        Pan2 = 1;
    else
        if DuanDian2<-yuzhi
            Pan2 = -1;
        else
            Pan2 = 0;
        end
    end
    
    if Pan1 * Pan2 <= 0
        YES = 1;
    else
%         result = SolveEqualZero(equation,range);
%         if isempty(result) == 1
%             YES = 0;
%         else
%             YES = 1;
%         end
        YES = 0;
    end
end

function result = SolveEqualZero(equation,range) %输出与横轴交点的具体值，精度有可能是不够的。
    SampleNum = 20;
    interval = (range(2)-range(1))/SampleNum;
    result = [];
    yuzhi = 10^-3;
    for i=range(1):interval:range(2)-interval
        pre = subs(equation,i);
        after = subs(equation,i + interval);
        if (pre<-yuzhi && after>=-yuzhi) || (pre>=yuzhi && after<yuzhi) ...
                || (pre>=-yuzhi && pre<=yuzhi)
            result = [result;i];
        end
    end
end

function result = SolveEquavpasolve(equation,range)
    syms t;
    result = vpasolve(equation==0,t,range);
    result = double(result);
end

function PositionNum = InWhichPosition(BorderLines,ztest) %BorderLines是从低到高
    if ztest<BorderLines(1)
        PositionNum = 0;
        return;
    end
    if ztest>BorderLines(end)
        PositionNum = 2*size(BorderLines,2);
        return;
    end
    for i =1:size(BorderLines,2)
        if (ztest == BorderLines(i))
            PositionNum = 2*i-1;
            return;
        end
    end
    for i=1:size(BorderLines,2)-1
        if ztest>BorderLines(i) && ztest<BorderLines(i+1)
            PositionNum = 2 * i;
            return;
        end
    end
end

function t = Return_tValue(z0,z1,ztest) %必须保证z0 z1 有高度差 从z0 指向z1
    if z0 == z1
        erro('必须保证有高度差');
    else
        t = (ztest - z0)/(z1-z0);
    end
end

function YES = IsInUp(z)%判断是否在最上边的那一层工作空间
    if z<=459.8 && z>=165.25
        YES = 1;
    else
        YES = 0;
    end
end

function YES = IsInMid(z)
    if z<165.25 && z>=-274.405
        YES = 1;
    else
        YES = 0;
    end
end

function YES = IsInDown(z)
    if z<-274.405 && z>=-441.0118
        YES = 1;
    else
        YES = 0;
    end
end

function SortedData = SortData(Data)
    [a,sortindex] = sort(Data(:,2));
    SortedData = [];
    for i=1:size(sortindex,1)
        SortedData = [SortedData;Data(sortindex(i),:)];
    end
end

function EdgeSet = EdgeExtraction(SortedData)
    EdgeSet = [];
    windowheight = 340;
    flag = 0;
    for i=1:size(SortedData,1)
        down = (i-1)*windowheight+1;
        up = i*windowheight;
        if i*windowheight > size(SortedData,1)
            up = size(SortedData,1);
            flag = 1;
        end
        NotSortData = SortedData(down:up,:);
        [a,sortindex] = sort(NotSortData(:,1));
        if isempty(sortindex)==1
            break;
        end
        EdgeSet = [EdgeSet;NotSortData(sortindex(1),:)];
        EdgeSet = [EdgeSet;NotSortData(sortindex(end),:)];
        if flag==1
            break;
        end
    end
end

function result = ToLeftTest(PointA,PointB,PointC)
    Area = PointA(1) * PointB(2) - PointA(2) * PointB(1) + PointB(1) * PointC(2) ...
		- PointB(2) * PointC(1) + PointC(1) * PointA(2) - PointC(2) * PointA(1);
    yuzhi = 1^-10;
    if Area > yuzhi
        result = 1;
    end
    if Area < -yuzhi
        result = -1;
    end
    if Area<yuzhi && Area>-yuzhi
        result = 0;
    end
end

function RandNumber = RandGenerateNumber(a,b,Num)
    RandNumber = a + (b-a).*rand(Num,1);
end

function PointsSet = RandGenratePointInWorkSpace(num)
    global yudu;
    if num == 0
        PointSet = [];
        return ;
    end
    r = RandGenerateNumber(-441.0118,459.8,num);
    PointsSet = [];
    for i = 1:size(r,1)
        z = r(i);
        if z<=459.8 && z>=165.25
            up = -0.0009554*z^2+0.1704*z+662.7 - yudu;
            down = (-0.3377*z^2+810.5*z-90990)/(z-74.85) + yudu;
            
        end
        if z<165.25 && z>=-274.405
            up = -0.000818*z^2+0.0902*z+671.8 - yudu;
            down = 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370 + yudu;
            
        end
        if z<-274.405 && z>=-441.0118
            up = 2.453*10^-5*z^3+0.02358*z^2+8.212*z+1572 - yudu;
            down = -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3 + yudu;
            
        end
%         up = down      ;
%         down = up;
        theta = RandGenerateNumber(0,2*pi,1);
        y0 = RandGenerateNumber(down,up,1);
        PointsSet = [PointsSet;[13.7 * cos(theta) - y0 * sin(theta) , 13.7 * sin(theta) + y0 * cos(theta) , z]];
    end
end

function [PointA,PointB] = RandGenratePointLineParallelground() %生成两个点，这两个点在一条平行于地面的直线上，且与挖机臂共面 
    global yudu;
    r = RandGenerateNumber(-441.0118,459.8,1);
    
    z = r;
    if z<=459.8 && z>=165.25
        up = -0.0009554*z^2+0.1704*z+662.7 - yudu;
        down = (-0.3377*z^2+810.5*z-90990)/(z-74.85) + yudu;

    end
    if z<165.25 && z>=-274.405
        up = -0.000818*z^2+0.0902*z+671.8 - yudu;
        down = 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370 + yudu;

    end
    if z<-274.405 && z>=-441.0118
        up = 2.453*10^-5*z^3+0.02358*z^2+8.212*z+1572 - yudu;
        down = -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3 + yudu;

    end

    theta = RandGenerateNumber(0,2*pi,1);
    
    y01 = RandGenerateNumber(down,down+(up-down)/10,1);
    y02 = RandGenerateNumber(up-(up-down)/10,up,1);
    PointA = [13.7 * cos(theta) - y01 * sin(theta) , 13.7 * sin(theta) + y01 * cos(theta) , z];
    PointB = [13.7 * cos(theta) - y02 * sin(theta) , 13.7 * sin(theta) + y02 * cos(theta) , z];
end

function [PointA,PointB] = RandGenratePointDirectLine()%生成两个点，这两个点在一直线上，与挖机臂共面 但这俩直线不一定是直线可达的
    global yudu;
    num = 2;
    if num == 0
        PointSet = [];
        return ;
    end
    r = RandGenerateNumber(-441.0118,459.8,num);
    theta = RandGenerateNumber(0,2*pi,1);
    PointsSet = [];
    for i = 1:size(r,1)
        z = r(i);
        if z<=459.8 && z>=165.25
            up = -0.0009554*z^2+0.1704*z+662.7 - yudu;
            down = (-0.3377*z^2+810.5*z-90990)/(z-74.85) + yudu;
            
        end
        if z<165.25 && z>=-274.405
            up = -0.000818*z^2+0.0902*z+671.8 - yudu;
            down = 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370 + yudu;
            
        end
        if z<-274.405 && z>=-441.0118
            up = 2.453*10^-5*z^3+0.02358*z^2+8.212*z+1572 - yudu;
            down = -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3 + yudu;
            
        end
        y0 = RandGenerateNumber(down,up,1);
        PointsSet = [PointsSet;[13.7 * cos(theta) - y0 * sin(theta) , 13.7 * sin(theta) + y0 * cos(theta) , z]];
    end
    PointA = PointsSet(1,:);
    PointB = PointsSet(2,:);
end

function PointsSet = GenratePointInner(num)%生成分布在工作空间内壁的点
    global yudu;
    if num == 0
        PointSet = [];
        return ;
    end
    r = RandGenerateNumber(-441.0118,459.8,num);
    PointsSet = [];
    for i = 1:size(r,1)
        z = r(i);
        if z<=459.8 && z>=165.25
            up = -0.0009554*z^2+0.1704*z+662.7 - yudu;
            down = (-0.3377*z^2+810.5*z-90990)/(z-74.85) + yudu;
            
        end
        if z<165.25 && z>=-274.405
            up = -0.000818*z^2+0.0902*z+671.8 - yudu;
            down = 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370 + yudu;
            
        end
        if z<-274.405 && z>=-441.0118
            up = 2.453*10^-5*z^3+0.02358*z^2+8.212*z+1572 - yudu;
            down = -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3 + yudu;
            
        end
        up = down      ;
%         down = up;
        theta = RandGenerateNumber(0,2*pi,1);
        y0 = RandGenerateNumber(down,up,1);
        PointsSet = [PointsSet;[13.7 * cos(theta) - y0 * sin(theta) , 13.7 * sin(theta) + y0 * cos(theta) , z]];
    end
end

function PointsSet = GenratePointOuter(num) %生成分布在工作空间外壁的点
    global yudu;
    if num == 0
        PointSet = [];
        return ;
    end
    r = RandGenerateNumber(-441.0118,459.8,num);
    PointsSet = [];
    for i = 1:size(r,1)
        z = r(i);
        if z<=459.8 && z>=165.25
            up = -0.0009554*z^2+0.1704*z+662.7 - yudu;
            down = (-0.3377*z^2+810.5*z-90990)/(z-74.85) + yudu;
            
        end
        if z<165.25 && z>=-274.405
            up = -0.000818*z^2+0.0902*z+671.8 - yudu;
            down = 2.897*10^-6*z^3-0.001479*z^2+0.1196*z+370 + yudu;
            
        end
        if z<-274.405 && z>=-441.0118
            up = 2.453*10^-5*z^3+0.02358*z^2+8.212*z+1572 - yudu;
            down = -3.011*10^-5*z^3-0.02679*z^2-8.276*z-721.3 + yudu;
            
        end
%         up = down      ;
        down = up;
        theta = RandGenerateNumber(0,2*pi,1);
        y0 = RandGenerateNumber(down,up,1);
        PointsSet = [PointsSet;[13.7 * cos(theta) - y0 * sin(theta) , 13.7 * sin(theta) + y0 * cos(theta) , z]];
    end
end









