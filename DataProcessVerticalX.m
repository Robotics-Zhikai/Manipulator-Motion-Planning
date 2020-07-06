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

for times=1:10
    %中间空洞的曲面方程
    d = 13.7;
    yudu = 8; %安全裕度
    syms x y z t
    P0 = RandGenratePointInWorkSpace(1,yudu);
    P1 = RandGenratePointInWorkSpace(1,yudu);
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
    figure(7)
    interval = 0.01;
    times
    for i = 1:size(equationSet,1)
%         plot([equationSet_trange(i,1):interval:equationSet_trange(i,2)],subs(equationSet(i),[equationSet_trange(i,1):interval:equationSet_trange(i,2)]),'-');

        result1 = SolveEqualZero(equationSet(i),equationSet_trange(i,:));
        result2 = SolveEquavpasolve(equationSet(i),equationSet_trange(i,:));
        YES = IsExistZero(equationSet(i),equationSet_trange(i,:));

%         hold on;
%         plot(result,subs(equationSet(i),result),'o');
%         hold on;
    end
    
%     pause(0.1);
end



figure
PointsSet = RandGenratePointInWorkSpace(5050,yudu);
plot3(PointsSet(:,1),PointsSet(:,2),PointsSet(:,3),'.');
xlabel('x');
ylabel('y');
zlabel('z');







function result = SolveEquaMuller(equation,range)%用抛物线法求交点
    Xk = [range(1) (range(1)+range(2))/2 range(2)];
    fxk = subs(equation,Xk(3));
    fxkminus1 = subs(equation,Xk(2));
    fxkminus2 = subs(equation,Xk(1));
    fxk + ((fxk-fxkminus1)/(Xk(3)-Xk(2)))*(x-Xk(3)) + 
      
end

function DQresult = DiffQuot(equation,xi,xj) %一阶差商计算公式
    fxi = subs(equation,xi);
    fxj = subs(equation,xj);
    DQresult = (fxi-fxj)/(xi-xj);
end

function YES = IsExistZero(equation,range) %判断与横轴交点的存在性 
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
        result = SolveEqualZero(equation,range);
        if isempty(result) == 1
            YES = 0;
        else
            YES = 1;
        end
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

function PointsSet = RandGenratePointInWorkSpace(num,yudu)
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












