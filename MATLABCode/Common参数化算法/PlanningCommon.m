clc
close all
% VisualizationExcavator(result);
clear all


GlobalDeclarationCommon





TwoInnerTangentPoints = FindInnerEdgeTangentPoints();
InnerEdgeDown = GetInnerEdgeOfPlaneWorkSpaceDown(0.5);
InnerEdgeUp = GetInnerEdgeOfPlaneWorkSpaceUp(0.5);
% InnerEdgeDown = [InnerEdgeDown;[250 -221]];
% InnerEdgeDown = [InnerEdgeDown;[200 -300]];
% InnerEdgeDown = [InnerEdgeDown;[370 150]];
% InnerEdgeDown = [InnerEdgeDown;[370 -60]];
% InnerEdgeDown = [InnerEdgeDown;[150 200]];
% InnerCH = [InnerEdgeDown zeros(size(InnerEdgeDown,1),1)];
% InnerCH = GetCHGrahamScan(InnerCH);
% figure
% plot(InnerCH(:,1),InnerCH(:,2),'-');
% hold on 
% plot(InnerEdgeDown(:,1),InnerEdgeDown(:,2),'.');




%%
[PointA,BeginAngelBucketWithGround,PointB,EndAngelBucketWithGround,angleA,angleB] = RandGenratePointDirectLineBucketTip([200 250]);
[Theta4Range,YES] = groundAngleRangeTOtheta4Range(angleA(1),angleA(2),angleA(3),[BeginAngelBucketWithGround,BeginAngelBucketWithGround]);
[Theta4Range1,YES1] = groundAngleRangeTOtheta4Range(angleB(1),angleB(2),angleB(3),[EndAngelBucketWithGround,EndAngelBucketWithGround]);
if abs(Theta4Range{1}-angleA(4))>0.001 || abs(Theta4Range1{1}-angleB(4))>0.001
    error('�ϱ����������ĳ����߼���������');
end
VisualBuckettipOuterEdge(110);
PlotSingularPointsOfBucketTip(110,PointA,PointB);

% PointA = [370.5466  430.4401 -357.6222];
% PointB = [383.6440  446.4183 -121.4106];
% BeginAngelBucketWithGround = -73.0732;
% EndAngelBucketWithGround = -109.0813;

% PointA = [-26.7292  510.7350    5.2501];
% PointB = [-10.9153  311.1723  -90.0148];
% BeginAngelBucketWithGround =  -77.0072;
% EndAngelBucketWithGround =  -147.7394;

% PointA = [221.1790 -518.2097 -465.9836];
% PointB = [229.1122 -535.6121 -244.8862];
% BeginAngelBucketWithGround = -72.6830;
% EndAngelBucketWithGround = -88.5871;

% PointA = [ -654.0350  -43.0796 -158.1275];
% PointB = [-484.1271  -28.3157 -335.6824];
% BeginAngelBucketWithGround = -93.2128;
% EndAngelBucketWithGround = -82.0676;

% PointA = [681.1443  -21.8567  172.4116];
% PointB = [ 546.0533  -20.2392  -29.5721];
% BeginAngelBucketWithGround = -21.2425;
% EndAngelBucketWithGround =  -111.4202;

% PointA = [556.1216 -240.4642  108.7025];
% PointB = [663.6957 -284.1187  299.7914];
% BeginAngelBucketWithGround = -93.6690;
% EndAngelBucketWithGround = -21.8327;

% PointB = [398.1105 -203.8291 -197.4437];
% PointA = [550.8013 -276.1908 -343.4893];
% EndAngelBucketWithGround = -157.2472;
% BeginAngelBucketWithGround = -67.9361;
%���û��� �嵽һ���û�취���� ��Ϊ�������޶�

figure
result = InverseKinematicsBucketTip(PointA',BeginAngelBucketWithGround);
PlotTheta1234(result(1),result(2),result(3),result(4));
figure
result = InverseKinematicsBucketTip(PointB',EndAngelBucketWithGround);
PlotTheta1234(result(1),result(2),result(3),result(4));


Sequence = BucketTipLinearPlanningROBOTICSTOOL(PointA,PointB,BeginAngelBucketWithGround,EndAngelBucketWithGround,5,5,5,25,25,25);
%�����װ�����ӻ����� 20200820

figure
reducedSeqplot = [];
for i=1:ceil(size(Sequence,2)/20):size(Sequence,2)
    reducedSeqplot = [reducedSeqplot Sequence(2:5,i)];
end
PeterCorkePlotRobot(reducedSeqplot');




BucketTipLinearPlanning(PointA,PointB,BeginAngelBucketWithGround,EndAngelBucketWithGround,150,150,150,25,25,25);
% (BeginPoint,EndPoint,Begin_Bucket_WithGround,End_Bucket_WithGround,Vtheta2Max,Vtheta3Max,Vtheta4Max,atheta2max,atheta3max,atheta4max)



%%
%���ǲ�����ת���ĵĹ滮
% PointA = [-32.1682 -588.1967  218.8374];
% PointB = [-30.6171 -538.7770  260.6377];
% 
% PointA = [543.8354 -235.0792 -175.7771];
% PointB = [  411.5576 -181.4958   56.1149];

PointA = [-247.1427 -517.7705 -265.1999];
PointB = [ -131.8404 -260.5854 -415.8218]; %�������ݺ���֣�Ŀ��λ�õ�����

% PointA  =[-297.1962 -539.3766 -150.4083];
% PointB = [-196.6604 -346.9060   60.3847];
% 
% PointA = [86.5212  276.6359 -244.3818];
% PointB = [149.2692  516.5239  -51.7284];

PointA = [437.0117   76.3013    5.7303];
PointB = [348.9690   58.1109 -216.2471];

PointA = [-266.3541 -497.8007 -285.9217];
PointB = [-260.9417 -487.0669    4.5837];

PointA = [ -113.8933 -593.2268 -154.3792];
PointB = [ -101.7682 -521.2970  119.7108];

PointA = [-508.0082  290.3112  130.1380];
PointB = [-361.0164  210.8165 -170.4685];
[PointA,PointB] = RandGenratePointDirectLine([300 350]);

% figure
% plot3(PointA(1),PointA(2),PointA(3),'o');
% hold on
% plot3(PointB(1),PointB(2),PointB(3),'o');
% hold on 
% view([-129 47])



AngleSequencetest = [0;90;theta2Range(1);theta3Range(1);theta4Range(1)];
VisualizationExcavator(AngleSequencetest);



resultPoints = RandGenerateStableBucketPoints(10,[-30,-30]);
figure
plot(resultPoints(:,2),resultPoints(:,3),'.');
axis([100 700 -500 500])


timethis = tic;
[PointsSequence,theta4sequence] = LinearDigPlanningRandomLine(PointA,PointB,theta4Range(1),theta4Range(2),150,150,150,15,25,25,InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints);
toc(timethis)
result = CombineTheta123AndTheta4(PointsSequence,theta4sequence);
VisualizationExcavator(result);

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
plot3(PointA(1),PointA(2),PointA(3),'o');
hold on
plot3(PointB(1),PointB(2),PointB(3),'o');
hold on 
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

function PlotSingularPointsOfBucketTip(lidu,StartPoint,EndPoint)
    GlobalDeclarationCommon
    k1=40;
    liduthis = lidu;
    x0 = StartPoint(1);
    y0 = StartPoint(2);
    z0 = StartPoint(3);
    
    x1 = EndPoint(1);
    y1 = EndPoint(2);
    z1 = EndPoint(3);
    for k2 = theta2Range(1):liduthis:theta2Range(2)
        for k3 = theta3Range(1):liduthis:theta3Range(2)
            for k4 = theta4Range(1):liduthis:theta4Range(2)
                
                JacoboMatrix = GetvOmiga50_JacoboMatrix(k1,k2,k3,k4);
                a11 = JacoboMatrix(1,1); a12 = JacoboMatrix(1,2); a13 = JacoboMatrix(1,3); a14 = JacoboMatrix(1,4);
                a21 = JacoboMatrix(2,1); a22 = JacoboMatrix(2,2); a23 = JacoboMatrix(2,3); a24 = JacoboMatrix(2,4);
                a31 = JacoboMatrix(3,1); a32 = JacoboMatrix(3,2); a33 = JacoboMatrix(3,3); a34 = JacoboMatrix(3,4);
                a51 = JacoboMatrix(5,1); a52 = JacoboMatrix(5,2); a53 = JacoboMatrix(5,3); a54 = JacoboMatrix(5,4);
%                 rank(JacoboMatrix(1:6,2:4))
%                 JacoboMatrix
%                 a32 = (z1-z0)/(x1-x0)*a12; a33 = (z1-z0)/(x1-x0)*a13; a34 = (z1-z0)/(x1-x0)*a14;
                detofthis = a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32;
%                 detofnew = (a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52)
                detofnew = (a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52)
                if detofnew==0
                    detofthis
                    [position1,position2] = ForwardKinematics([k1;k2;k3;k4]);
                    plot(position2(2,4),position2(3,4),'ko');
                    hold on 
                    pause(0.1);
                end
            end
        end
    end

end

function VisualBuckettipOuterEdge(lidu)
    GlobalDeclarationCommon
    figure
    liduthis = lidu;
    k1= 90;
    for k2 = theta2Range(1):liduthis:theta2Range(2)
        for k3 = theta3Range(1):theta3Range(2)-theta3Range(1):theta3Range(2)
            for k4 = theta4Range(1):liduthis:theta4Range(2)
                [position1,position2] = ForwardKinematics([k1;k2;k3;k4]);
                plot(position2(2,4),position2(3,4),'r.');
                hold on 
                pause(0.1);
            end
            plot(position1(2,4),position1(3,4),'b.');
            hold on 
        end
    end
    for k2 = theta2Range(1):theta2Range(2)-theta2Range(1):theta2Range(2)
        for k3 = theta3Range(1):liduthis:theta3Range(2)
            for k4 = theta4Range(1):liduthis:theta4Range(2)
                [position1,position2] = ForwardKinematics([k1;k2;k3;k4]);
                plot(position2(2,4),position2(3,4),'r.');
                hold on 
                pause(0.1);
            end
            plot(position1(2,4),position1(3,4),'b.');
            hold on 
        end
    end
end

function[PointsSequence,theta4sequence] = LinearDigPlanningRandomLine(StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4,InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints)%��������ֱ��ֱ���ھ�
%Ϊ�˽������pid���ƺ��ܾ�ȷ����Ŀ��λ�õ�����
    begindistance = norm(StartPoint-EndPoint)
    if begindistance<=1
        jointAngle = InverseKinematicsPos2Angle(StartPoint);
        PointsSequence = [0;jointAngle(1);jointAngle(2);jointAngle(3)];
        theta4sequence = [0;theta4begin];
        disp('������������̫���ˣ�С��1����');
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
    
    [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(kp,ki,kd,StartPoint,EndPointtmp,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4,InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints);
    if DirectReachable==0
        error('��������㲻��ֱ�ӵ�ֱ�߿ɴ������һ�㺯��403');
        return;
    end
    
    lasterrorthis = norm(EndPoint-endPosition)/norm(StartPoint-EndPoint);
    lastdirection = 0;
    countTimes = 0;
    Sumerro = 0;
    

    
    countDirectReachable0 = 0;
    countOscillation = 0;%��¼�񵴵Ĵ���
    while norm(EndPoint-endPosition)>1 %�����ֵ���Ļ� ���Ƚ��ͣ������ٶȻ������� �ڵ����㷨�о�ʱ�������ŵͣ�����Ϊ1  ʵ��Ӧ�ÿɸ��������ʵ�����
        PointsSequenceStorage = PointsSequence;
        theta4sequenceStorage = theta4sequence;
        endPositionStorage = endPosition;
        DirectReachableStorage = DirectReachable;
%         [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(1.1,0.002,0.65,StartPoint,EndPointtmp,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4);
        [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(kp,ki,kd,StartPoint,EndPointtmp,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4,InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints);
        
        if DirectReachable==0
            countDirectReachable0 = countDirectReachable0 + 1;
            EndPointtmp = StartPoint + (1-0.1) * (EndPoint-StartPoint);
            while IsDirectReachablePlane(InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints,StartPoint,EndPointtmp)==0
                EndPointtmp = StartPoint + (1-0.1) * (EndPointtmp-StartPoint);
            end
%             kiScale = 0;
            endPosition = endPositionStorage;
            
            if countDirectReachable0 == 2
                %˵����һ��̫�����߽��ˣ���������ȥҲûɶ�ã�
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
                    %˵�����ˣ������޸�pid��������
                    countOscillation = countOscillation+1;
                    if countOscillation==2 %˵���޸�PID�������κ������񵴣���ô���˳���û��Ҫ�����ˣ���ʱĩ�˾���Ŀ��λ�õ���һ�������趨��Ҫ��
                        disp('�����񵴣�');
                        return;
                    end 
%                     ki = 0;
%                     kiScale = 0;
                    ki = ki/2;
                    
                    kiScale = kiScale/4;
                    
                    Sumerro = 0;
                    lasterrorthis = 0.5;%���п�����0.5
                    
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
            errorthis = norm(EndPoint-endPosition)/norm(StartPoint-EndPoint);
            
%             if errorthis< norm(StartPoint-EndPoint)/2
%                 kiScale = 0.00015;%����þ���С������һ��С�ľͻ�����
%             else
%                 kiScale = 0.00015;
%             end
            kiScale = 0.018;
            %     kpScale = 0.001;
            kpScale = 0.135; %��������һ��֮��Ĳ���
            kdScale = 0.01;
%             kdScale = 0; %���d����Ϊ0�Ļ���������Щ��������ٶȻἫ����
            
            Sumerro = Sumerro+errorthis;
            sumI = kiScale*Sumerro
            xishu = kpScale*errorthis + kiScale*Sumerro - kdScale*(abs(errorthis-lasterrorthis));
            xishu = Limit2range(xishu,[0,0.1])
            lasterrorthis = errorthis;
            
            EndPointtmp = StartPoint + (1+direction*xishu) * (EndPointtmp-StartPoint);
        end
        
%         plot3(EndPointtmp(1),EndPointtmp(2),EndPointtmp(3),'.');
%         hold on 
%         pause(0.1);
        
        direction*norm(EndPoint-endPosition)
    end
    [pos1,pos2] = ForwardKinematics([PointsSequence(2,end) PointsSequence(3,end) PointsSequence(4,end) 0]);
    CurrentPoint = pos1(1:3,4);
    CurrentPoint = CurrentPoint';
    enddistance = norm(CurrentPoint-EndPoint)
end

function [PointsSequence,theta4sequence,endPosition,DirectReachable] = LinearDigPlanningRandomLinesub(kp,ki,kd,StartPoint,EndPoint,theta4begin,theta4end,Vmaxtheta2,Vmaxtheta3,Vmaxtheta4,amaxtheta2,amaxtheta3,amaxtheta4,InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints)%��������ֱ��ֱ���ھ���Ӻ���
  %PointsSequence �洢theta1 theta2 theta3 ��ʱ��仯������
    if IsDirectReachablePlane(InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints,StartPoint,EndPoint) == 0
        PointsSequence = [];
        theta4sequence = [];
        DirectReachable = 0;
        endPosition = [];
        disp('��������㲻��ֱ�ӵ�ֱ�߿ɴ������һ�㺯��');
        return;
    end
    GlobalDeclarationCommon
    
    DirectReachable = 1;
    PointsSequence = [];
%     tinterval = 0.01; %�����˶�ʱ����0.01��
    angleinterval = 1; %ÿ�ιؽ��˶�1��
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
%             if abs(mean(Queue)-Queue(1))<0.1 �����ȡֵ������ʱ���й�ϵ    ������ʱ��Ϊ0.02�����������
            if abs(mean(Queue)-Queue(1))<0.1
                break;
%                 disp('');
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
%         k_xishuV = 1.1*errovalue + 0.002*Sumerrovalue - 0.65*derrovalue; %�޸����pid�Ĳ����ﵽ���õ�Ч��
        k_xishuV = kp*errovalue + ki*Sumerrovalue - kd*derrovalue;
        k_xishuV = Limit2range(k_xishuV,[0,1]);
%         plot(num,k_xishuV,'.');
%         hold on 
%         pause(0.1);
        Lasterrovalue = errovalue;
        
        [vtheta2,vtheta3] = GetCurrentvtheta3RandomLine(k_xishuV,CurrentPoint,jointAngle,StartPoint,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3);
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
%             break; %֮��Ϳ�ʼ���� �������ؽڶ��ܳ��ܵļ��ٶȽ��м��٣�ֻ���������ܻᳬ��ĩλ�� ����������ֵ���м��٣������õ�Ŀ��㣬����ᳬ��Ŀ��㡣Ҫ������ʹ���ٶȴﵽ���
%         end
    end
    
    %����ʱ��Ҫ������Ϊ�ٶ�̫�����˳�����ʱû�е���Ŀ�ĵأ�Ҫ������Ϊ�Ը��ٶȵ�����Ŀ��λ�ã�����ʱ������ֽ�һ������
    %�����pid��ԭ�������Ҫ���㷨�Ե���Ŀ�ĵ���ĩ�ٶ�Ϊ0�˳����������±ߵļ��ٹ���

%     if flagSlowDown == 0 %�������
%         flagSlowDown = 1;
%         EndPoint = StartPoint + 2*(EndPoint-StartPoint);
%         x1 = EndPoint(1);
%         y1 = EndPoint(2);
%         z1 = EndPoint(3);
%     end
    
%     if flagSlowDown == 1
   
    acurrenttheta3 = amaxtheta3;
    while abs(vtheta3)>(acurrenttheta3*tinterval) %������ٽṹ������ ��Ϊpid��������ٶȣ��л�����Ļ����������ٶ�Ϊ0
        k1 = theta1*pi/180;
        k2 = theta2*pi/180;
        k3 = theta3*pi/180;
        acurrenttheta2 = amaxtheta2;
        acurrenttheta3 = amaxtheta3;
        if x0 ~= x1
            tmpkxishu = kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1);
            if amaxtheta2 < abs(tmpkxishu)*amaxtheta3
                %˵��amaxtheta2 ������amaxtheta3
                acurrenttheta3 = acurrenttheta2/tmpkxishu;
                acurrenttheta3 = abs(acurrenttheta3);
                disp('Ҫע��������Ǿ�ȷ��Ŀ��λ��!Ҫ�뾫ȷ��������߼��ٶ�Լ���Ĵ�С');
            end
        else
            if y0~=y1
                tmpkxishu = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1);
                if amaxtheta2 <abs(tmpkxishu)*amaxtheta3
                    acurrenttheta3 = acurrenttheta2/tmpkxishu;
                    acurrenttheta3 = abs(acurrenttheta3);
                    disp('Ҫע��������Ǿ�ȷ��Ŀ��λ��!Ҫ�뾫ȷ��������߼��ٶ�Լ���Ĵ�С');
                end
            else
                disp('Ҫ��ɵ�ֱ���Ǵ�ֱ�ڵ����')
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
                disp('Ҫ��ɵ�ֱ���Ǵ�ֱ�ڵ����')
            end
        end
%             d_k2 = -(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)); %����ͨ�����ſɱȾ���õ��Ķ�Ӧ��ϵ
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
    
    %������ֻҪ�������滮ʱ���ڰѲ����ڶ���Ԥ��λ�ü���
    
    success = 0;
    k_tf = 1.1;
    tf = PointsSequence(1,end)/k_tf;
    while success == 0
        tf = k_tf*tf;
        [theta4sequence,success] = ManipulatorPlanningJointSpace(theta4begin,theta4end,tf,amaxtheta4,Vmaxtheta4,tinterval);
        [PointsSequence1,success1] = ManipulatorPlanningJointSpaceMethod2test(theta4begin,theta4end,tf,amaxtheta4,Vmaxtheta4,tinterval);
        if sum(sum(theta4sequence-PointsSequence1))~=0
            erro('�����߼������⣡');
        end
    end
%     enddistance = norm(CurrentPoint-EndPoint); %�����������������ԣ����ᳬ��2cm
    endPosition = CurrentPoint;
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


function [vtheta2,vtheta3] = GetCurrentvtheta3RandomLine(k_xishu,CurrentPointTMP,CurrentJointAngle,StartPoint,EndPoint,tinterval,vtheta2Last,vtheta3Last,Vmaxtheta2,Vmaxtheta3,amaxtheta2,amaxtheta3)%���������ٶ����Ƶĵ�ǰʱ���ٶ�ֵ
    GlobalDeclarationCommon
    amaxtheta2 = amaxtheta2*pi/180;
    amaxtheta3 = amaxtheta3*pi/180;
    fuhao = 1;
    lastd_k2 = vtheta2Last*pi/180;
    lastd_k3 = vtheta3Last*pi/180;
    flagonce = 0;
    xishu = 0.3; %ÿ�ε���������ϵ��
    k_yuzhi = 0.9; %������Ϊ���������һ��ԣ�ȵ����ֵ
    
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
%     [pos1,pos2] = ForwardKinematics([theta1 theta2 theta3 0]);
%     CurrentPoint = pos1(1:3,4);
%     CurrentPoint = CurrentPoint';
%     if norm(CurrentPointTMP-CurrentPoint)>0.001
%         disp('');
%     end
    CurrentPoint = CurrentPointTMP;
    
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
        error('��Ҫ�����޸���ز�������������޷�������ٶ�Լ��');
        return;
    end
    
    if huduVtheta3range>=absvtheta3Upper %���������ٶ����Ƶ�Vtheta3�����½�
        Fita_Vtheta3Upper = absvtheta3Upper;
        Fita_Vtheta3Lower = absvtheta3Lower;
    else
        Fita_Vtheta3Upper = huduVtheta3range;
        Fita_Vtheta3Lower = absvtheta3Lower;
    end
    if huduVtheta2range>=absvtheta2Upper %���������ٶ����Ƶ�Vtheta3�����½�
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
    
    Fita_Vtheta3 = [Fita_Vtheta3Lower Fita_Vtheta3Upper];%���ǻ���
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
        Vx = d_k2*((sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cos(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + cos(k4)*cosm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) - (cosm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + sinm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cosm3*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2) + sin(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)) - d_k3*((a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cos(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + cos(k4)*cosm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2)) - (a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cosm3*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2) + sin(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1)) + a3*sinm3*(cosm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + sinm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2)));
        Vy = - d_k2*((cosm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + sinm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (cos(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) - sin(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + cosm3*sin(k4)*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2) + (sin(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) + cos(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) - cos(k4)*cosm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3)) - d_k3*((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) - sin(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + cosm3*sin(k4)*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) + cos(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) - cos(k4)*cosm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))) + a3*sinm3*(cosm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + sinm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))));
        Vz = d_k2*((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)) + d_k3*((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) + a3*sinm3*(sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1))));
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
            Vx = d_k2*((sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cos(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + cos(k4)*cosm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) - (cosm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + sinm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cosm3*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2) + sin(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)) - d_k3*((a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cos(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + cos(k4)*cosm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2)) - (a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cosm3*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2) + sin(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1)) + a3*sinm3*(cosm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + sinm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2)));
            Vy = - d_k2*((cosm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + sinm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (cos(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) - sin(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + cosm3*sin(k4)*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2) + (sin(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) + cos(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) - cos(k4)*cosm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3)) - d_k3*((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) - sin(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + cosm3*sin(k4)*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) + cos(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) - cos(k4)*cosm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))) + a3*sinm3*(cosm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + sinm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))));
            Vz = d_k2*((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)) + d_k3*((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) + a3*sinm3*(sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1))));
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
        error('��Ҫ�����޸���ز�������������޷�������ٶ�Լ��');
    end
    vtheta3intersectiontmp = [];
    for i=1:size(vtheta2intersection,2)
        d_k2 = vtheta2intersection(i)*pi/180;
        if x0~=x1
            Vtheta3tmp = 1/kd_k2_divide_d_k3_x1NEx0(k1,k2,k3,x0,z0,x1,z1)*d_k2;
%             Vtheta3tmp = (- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1))));
        else
            if y0~=y1
                Vtheta3tmp = 1/kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1)*d_k2;
%                 Vtheta3tmp = (- (sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1)) - 1)*d_k2/(-(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*sin(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(y0 - y1))));
            else
                disp('˵��ֻ��z���򣬴�ʱֻ��һ��vtheta2 vthata3�Ľ�')
            end
        end
        vtheta3intersectiontmp(1,i) = Vtheta3tmp*180/pi;
    end
    
    if isempty(GetIntersection(vtheta3intersectiontmp,vtheta3intersection))==1
        erro('�����߼�����');
    end
    
    vtheta2reliablerange = vtheta2intersection; %�ɿ���ȡֵ��Χ
    vtheta3reliablerange = vtheta3intersectiontmp;
    
    if isempty(vtheta2reliablerange)==1 || isempty(vtheta3reliablerange)==1
        erro('�����߼�����');
    end
    
%     vtheta3 = vtheta3reliablerange(end);
%     vtheta2 = vtheta2reliablerange(end);
%     if abs(vtheta2) <=0.00001 && abs(vtheta3) <= 0.00001
%         vtheta3 = vtheta3reliablerange(1);
%         vtheta2 = vtheta2reliablerange(1);
%     end

%     k_xishu = 0.9; %k_xishuȡֵ��ΧΪ[0,1] k_xishuԽ��ƽ���ٶ�Խ��
    if abs(vtheta3reliablerange(end))>=abs(vtheta3reliablerange(1))
        vtheta3 = vtheta3reliablerange(1) + (vtheta3reliablerange(end)-vtheta3reliablerange(1))*(k_xishu);
        vtheta2 = vtheta2reliablerange(1) + (vtheta2reliablerange(end)-vtheta2reliablerange(1))*(k_xishu);
    else
        vtheta3 = vtheta3reliablerange(1) + (vtheta3reliablerange(end)-vtheta3reliablerange(1))*(1-k_xishu);
        vtheta2 = vtheta2reliablerange(1) + (vtheta2reliablerange(end)-vtheta2reliablerange(1))*(1-k_xishu);
    end
end


function [Vtheta2,Vtheta3] = LinearDiggingGetRangeRandomLine(Vmaxtheta2,Vmaxtheta3,theta1,theta2,theta3,StartPoint,EndPoint)
     %Ҫ��֤����Vmax����ֵ
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
            disp('˵��ֻ��z���򣬴�ʱֻ��һ��vtheta2 vthata3�Ľ�')
        end
    end
    %     Vtheta2tmp = -(2109*d_k3*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)); %����ͨ�����ſɱȾ���õ��Ķ�Ӧ��ϵ
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
    k4 = theta4Range(2) * pi / 180; %���������ȡ�� �����ս��û��Ӱ��
    result = -(- ((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) + a3*sinm3*(sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1))))/((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)) - ((z0 - z1)*((a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cos(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + cos(k4)*cosm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2)) - (a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cosm3*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2) + sin(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1)) + a3*sinm3*(cosm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + sinm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))))/((x0 - x1)*((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2))))/((((z0 - z1)*((sin(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cos(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + cos(k4)*cosm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) - (cosm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1) + sinm3*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) - cosm2*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + sin(k1)*sin(k3)*sinm1*sinm2) - cosm3*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cosm1*sin(k1)*sin(k2)) + cos(k3)*cosm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) - cos(k3)*sin(k1)*sinm1*sinm2) + sin(k4)*sinm3*(sinm2*(cos(k1)*sin(k2) + cos(k2)*cosm1*sin(k1)) + cosm2*sin(k1)*sinm1))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)))/((x0 - x1)*((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2))) - 1));
end

function result = kd_k2_divide_d_k3_y1NEy0(k1,k2,k3,y0,z0,y1,z1)
    GlobalDeclarationCommon
    k4 = theta4Range(2) * pi / 180; %���������ȡ�� �����ս��û��Ӱ��
    result = -((- ((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0))) + a3*sinm3*(sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1))))/((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)) - ((z0 - z1)*((a3*cosm3*sin(k4) + d4*cos(k4)*sinm3)*(cos(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) - sin(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + cosm3*sin(k4)*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))) - (a3*cos(k4)*cosm3 - d4*sin(k4)*sinm3)*(sin(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) + cos(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) - cos(k4)*cosm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1))) + a3*sinm3*(cosm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + sinm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))))/((y0 - y1)*((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2)))))/((- ((z0 - z1)*((cosm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + sinm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (cos(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) - sin(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) + cosm3*sin(k4)*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2) + (sin(k4)*(sin(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) - cos(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cosm2*sin(k3)*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)) + cos(k4)*sinm3*(sinm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1) - cosm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1)) - cos(k4)*cosm3*(sin(k3)*(cos(k2)*cosm0*sin(k1) - sin(k2)*sinm0*sinm1 + cos(k1)*cosm0*cosm1*sin(k2)) + cos(k3)*sinm2*(cosm1*sinm0 + cos(k1)*cosm0*sinm1) + cos(k3)*cosm2*(cosm0*sin(k1)*sin(k2) + cos(k2)*sinm0*sinm1 - cos(k1)*cos(k2)*cosm0*cosm1)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3)))/((y0 - y1)*((sinm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cosm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)))*(a2*cosm3*sinm2 + a3*cosm2*sinm3 + a2*cos(k3)*cosm2*sinm3 + a3*cos(k3)*cosm3*sinm2 - d3*sin(k3)*sinm2*sinm3) + (sin(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) + cos(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) - cos(k4)*cosm3*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cosm2*sin(k3)*sin(k4) - a3*cos(k4)*cosm2*cosm3 + a2*cos(k4)*sinm2*sinm3 + d3*cos(k3)*sin(k4)*sinm2 + d4*cos(k4)*sin(k3)*sinm2 + d4*cosm2*sin(k4)*sinm3 + a3*cos(k3)*cos(k4)*sinm2*sinm3 + d3*cos(k4)*cosm3*sin(k3)*sinm2 + d4*cos(k3)*cosm3*sin(k4)*sinm2 - a2*cos(k3)*cos(k4)*cosm2*cosm3) + (cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + sin(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) + cosm2*sin(k3)*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)) - sin(k4)*sinm3*(sinm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0) - cosm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1)) + cosm3*sin(k4)*(cos(k3)*sinm2*(cosm0*cosm1 - cos(k1)*sinm0*sinm1) - sin(k3)*(cos(k2)*sin(k1)*sinm0 + cosm0*sin(k2)*sinm1 + cos(k1)*cosm1*sin(k2)*sinm0) + cos(k3)*cosm2*(cos(k2)*cosm0*sinm1 - sin(k1)*sin(k2)*sinm0 + cos(k1)*cos(k2)*cosm1*sinm0)))*(a2*cos(k4)*cosm2*sin(k3) + a3*cosm2*cosm3*sin(k4) + d3*cos(k3)*cos(k4)*sinm2 + d4*cos(k4)*cosm2*sinm3 - a2*sin(k4)*sinm2*sinm3 - d4*sin(k3)*sin(k4)*sinm2 + a2*cos(k3)*cosm2*cosm3*sin(k4) + d4*cos(k3)*cos(k4)*cosm3*sinm2 - a3*cos(k3)*sin(k4)*sinm2*sinm3 - d3*cosm3*sin(k3)*sin(k4)*sinm2))) - 1));
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


function blockcodeA = LocateWhereBlock(TwoInnerTangentPoints,PointA)
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

function YES = IsDirectReachablePlaneSub(CH,waitforprocess,anotherPoint) %�ж�ֱ�������һЩ����ж� һ��Բ��ĵ㣬����������
    num = 0;
    for i=1:size(CH,1)
        if norm(CH(i,:)-waitforprocess)<0.001
            num = i;
            break;
        end
    end
    if num == 0
        error('�߼������ˣ���');
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

function YES = ProcessPointABandblock(InnerEdgeUp,InnerEdgeDown,waitforprocess,anotherPoint,blockwaitforprocess) %�������IsDirectReachablePlaneSub����Ӧ��ͬ����״
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

function YES = IsDirectReachablePlane(InnerEdgeUp,InnerEdgeDown,TwoInnerTangentPoints,PointA,PointB) %��ά��е��ƽ��Ŀɴ����ж�
    GlobalDeclarationCommon
    YES = [];
    jointAngle1 = InverseKinematicsPos2Angle(PointA);
    jointAngle2 = InverseKinematicsPos2Angle(PointB);
    if abs(jointAngle1(1)-jointAngle2(1))>0.001
        error('�޷��ж����ֲ��ͻ�е����ͬһƽ��Ŀɴ���');
        return;
    end
    yuzhithis = 0;
    theta2rangetmp = [theta2Range(1)-yuzhithis theta2Range(2)+yuzhithis];
    theta3rangetmp = [theta3Range(1)-yuzhithis theta3Range(2)+yuzhithis];
    if isempty(GetIntersection(jointAngle1(1),theta1Range)) || isempty(GetIntersection(jointAngle1(2),theta2rangetmp)) || isempty(GetIntersection(jointAngle1(3),theta3rangetmp)) ...
          || isempty(GetIntersection(jointAngle2(1),theta1Range)) || isempty(GetIntersection(jointAngle2(2),theta2rangetmp)) || isempty(GetIntersection(jointAngle2(3),theta3rangetmp))
%         error('��һ�㴦�ڹ����ռ�֮�⣡');
        disp('��һ�㴦�ڹ����ռ�֮�⣡');
        YES = 0;
        return;
    end
    
    [A,B,C] = GetLineABC(TwoInnerTangentPoints(1,:),TwoInnerTangentPoints(2,:));
    
    
%     z=(-A*y-C)/B;
%     y=(-B*z-C)/A;
%     PointALocatey = (PointA(1)^2+PointA(2)^2-(sqrt(a1^2+d2^2))^2)^(1/2) ;
    bthis = sqrt(a1^2+d2^2);
    athis=norm(PointA(1:2));
    cthis = (2*bthis*cosinetheta+sqrt(4*bthis^2*cosinetheta^2-4*(bthis^2-athis^2)))/2;
    PointALocatey = cthis;
    yCriticalA = (-B*PointA(3)-C)/A;
%     PointBLocatey = (PointB(1)^2+PointB(2)^2-(sqrt(a1^2+d2^2))^2)^(1/2) ;
    bthis = sqrt(a1^2+d2^2);
    athis=norm(PointB(1:2));
    cthis = (2*bthis*cosinetheta+sqrt(4*bthis^2*cosinetheta^2-4*(bthis^2-athis^2)))/2;
    PointBLocatey = cthis;
    yCriticalB = (-B*PointB(3)-C)/A;
    
    FakePointA = [0 PointALocatey PointA(3)];
    FakePointB = [0 PointBLocatey PointB(3)];
    if PointALocatey>=yCriticalA && PointBLocatey>=yCriticalB
        YES = 1;
    else
        if PointALocatey<=yCriticalA && PointBLocatey<=yCriticalB
            blockcodeA = LocateWhereBlock(TwoInnerTangentPoints,FakePointA);
            blockcodeB = LocateWhereBlock(TwoInnerTangentPoints,FakePointB);
            if blockcodeA==0||blockcodeB==0
                error('�����߼�����');
            end
            if blockcodeA ~= blockcodeB
                YES = 0;
            else
                waitforprocess = FakePointA(2:3);
                anotherPoint = FakePointB(2:3);
                blockwaitforprocess = blockcodeA;
                YES = ProcessPointABandblock(InnerEdgeUp,InnerEdgeDown,waitforprocess,anotherPoint,blockwaitforprocess);
            end
        else
            blockcodeA = 0;
            blockcodeB = 0;
            if PointALocatey<=yCriticalA
                blockcodeA = LocateWhereBlock(TwoInnerTangentPoints,FakePointA);
            else
                blockcodeB = LocateWhereBlock(TwoInnerTangentPoints,FakePointB);
            end
            if blockcodeA==0 && blockcodeB==0
                error('�����߼�����');
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
            YES = ProcessPointABandblock(InnerEdgeUp,InnerEdgeDown,waitforprocess,anotherPoint,blockwaitforprocess);
        end
    end
    if isempty(YES)==1
        error('�����߼�����');
    end
end

function [PointsSequence,success] = ManipulatorPlanningJointSpace(theta0,thetaf,tf,AMAX,VMAX,SampleTime) %Ĭ�ϳ�ʼĩ���ٶ�Ϊ0�����ٶ�Ϊ0;
%-180,180 �Ƕ���ϵ
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
    k = 0.1; %k=ta/tb,ȡֵ��ΧΪ[0,0.5]
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
            disp('1VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
            return;
        end
        if a_Uppertemp>=AMAX
            if AMAX<a_LowerBound
                disp('���ٶȲ�����Ҫ���޸�tf����theta0 thetaf���߸ı�AMAX');
                return;
            end
            a_UpperBound = AMAX;
        else
            if a_Uppertemp<a_LowerBound
                disp('2VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
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
            disp('1VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
            return;
        end
        if a_Uppertemp>=AMAX
            if AMAX<a_LowerBound
                disp('���ٶȲ�����Ҫ���޸�tf����theta0 thetaf���߸ı�AMAX');
                return;
            end
            a_UpperBound = AMAX;
        else
            if a_Uppertemp<a_LowerBound
                disp('2VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
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

function [PointsSequence,success] = ManipulatorPlanningJointSpaceMethod2test(theta0,thetaf,tf,AMAX,VMAX,SampleTime) %����תC++�Ĵ������
    PointsSequence = [];
    for i=1:tf/SampleTime+1
        t = (i-1)*SampleTime;
        [PointCurrentTime,success] = ManipulatorPlanningJointSpaceSub(theta0,thetaf,tf,AMAX,VMAX,t,0.1,1);
        if success == 1
            PointsSequence = [PointsSequence [t;PointCurrentTime]];
        else
            PointsSequence = [];
            return;
        end
    end
end

function [PointCurrentTime,success] = ManipulatorPlanningJointSpaceSub(theta0,thetaf,tf,AMAX,VMAX,t,k,k_amax) %t������[0,tf]������ k=ta/tb,ȡֵ��ΧΪ[0,0.5] k_amaxȡֵ��ΧΪ[0,1]
%-180,180 �Ƕ���ϵ
    theta0 = legalizAnger(theta0);
    thetaf = legalizAnger(thetaf);
    
    success = 0;
    PointCurrentTime = theta0;
    if t<0
%         disp("t����С��0��");
        return;
    end
    if t>tf
%         disp('t���ܴ���tf��');
        return;
    end
    
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
   
%     k = 0.1; %k=ta/tb,ȡֵ��ΧΪ[0,0.5]
    if theta0 == thetaf
        PointCurrentTime = theta0;
        success = 1;
        return;
    end
    if (theta0<thetaf)
        a_LowerBound = 4*(theta0-thetaf)/( (k-1)*tf^2 );
        a_Uppertemp = VMAX^2 / ( (1-k)*(theta0-thetaf+tf*VMAX) );
        if a_Uppertemp<a_LowerBound
%             disp('1VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
            return;
        end
        if a_Uppertemp>=AMAX
            if AMAX<a_LowerBound
%                 disp('���ٶȲ�����Ҫ���޸�tf����theta0 thetaf���߸ı�AMAX');
                return;
            end
            a_UpperBound = AMAX;
        else
            if a_Uppertemp<a_LowerBound
%                 disp('2VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
                return;
            end
            a_UpperBound = a_Uppertemp;
        end
%         amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
        amax = a_LowerBound + k_amax * (a_UpperBound-a_LowerBound);
%         amax = a_UpperBound;
        tb = -((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*tf^2 - amax*k*tf^2))^(1/2) - amax*tf + amax*k*tf)/(2*(amax - amax*k));
        ta = k*tb;
        
%             PointsSequence(1,i) = (i-1)*SampleTime;
%             t = PointsSequence(1,i);
        if t>=0&&t<ta
            PointCurrentTime = (amax*t^3)/(6*k*tb) + theta0;

        end
        if t>=ta && t<tb-ta
            PointCurrentTime = theta0 + (amax*t*(t - k*tb))/2 + (amax*k^2*tb^2)/6;

        end
        if t>=tb-ta && t<tb
            PointCurrentTime = theta0 + (amax*t^2)/(2*k) + (amax*k^2*tb^2)/6 + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*t*k^2 - 6*t*k + 3*t))/(6*k) - (amax*t^3)/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        if t>=tb && t<tf-tb
            PointCurrentTime = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 + amax*(tb - k*tb)*(t - tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        if t>=tf-tb && t<tf-tb+ta
            PointCurrentTime = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k) - (amax*(t + tb - tf)*(6*k^2*tb^2 - 6*k*tb^2 + t^2 + 2*t*tb - 2*t*tf + tb^2 - 2*tb*tf + tf^2))/(6*k*tb);

        end
        if t>=tf-tb+ta && t<tf-ta
            PointCurrentTime = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - (amax*(t + tb - tf - k*tb)*(t - tb - tf + 2*k*tb))/2 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        if t>=tf-ta && t<=tf
            PointCurrentTime = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/3 - amax*(2*tb - tf)*(tb - k*tb) + amax*(tb - k*tb)*(tb - 2*k*tb) - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) + (amax*(t^3 - 3*t^2*tf + 3*t*tf^2 - tf^3))/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        
    else
        thetatemp = thetaf;
        thetaf = theta0;
        theta0 = thetatemp;
        a_LowerBound = 4*(theta0-thetaf)/( (k-1)*tf^2 );
        a_Uppertemp = VMAX^2 / ( (1-k)*(theta0-thetaf+tf*VMAX) );
        if a_Uppertemp<a_LowerBound
%             disp('1VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
            return;
        end
        if a_Uppertemp>=AMAX
            if AMAX<a_LowerBound
%                 disp('���ٶȲ�����Ҫ���޸�tf����theta0 thetaf���߸ı�AMAX');
                return;
            end
            a_UpperBound = AMAX;
        else
            if a_Uppertemp<a_LowerBound
%                 disp('2VMAX������Ҫ���޸�tf����theta0 thetaf���߸ı�VMAX');
                return;
            end
            a_UpperBound = a_Uppertemp;
        end
%         amax = RandGenerateNumber(a_LowerBound,a_UpperBound,1);
%         amax = a_UpperBound;
        amax = a_LowerBound + k_amax * (a_UpperBound-a_LowerBound);
        tb = -((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*tf^2 - amax*k*tf^2))^(1/2) - amax*tf + amax*k*tf)/(2*(amax - amax*k));
        ta = k*tb;
%         for i=1:tf/SampleTime+1
%             PointsSequence(1,i) = (i-1)*SampleTime;
%             t = PointsSequence(1,i);
        t = 2*(tf/2)-t;
        if t>=0&&t<ta
            PointCurrentTime = (amax*t^3)/(6*k*tb) + theta0;

        end
        if t>=ta && t<tb-ta
            PointCurrentTime = theta0 + (amax*t*(t - k*tb))/2 + (amax*k^2*tb^2)/6;

        end
        if t>=tb-ta && t<tb
            PointCurrentTime = theta0 + (amax*t^2)/(2*k) + (amax*k^2*tb^2)/6 + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*t*k^2 - 6*t*k + 3*t))/(6*k) - (amax*t^3)/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        if t>=tb && t<tf-tb
            PointCurrentTime = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 + amax*(tb - k*tb)*(t - tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        if t>=tf-tb && t<tf-tb+ta
            PointCurrentTime = theta0 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k) - (amax*(t + tb - tf)*(6*k^2*tb^2 - 6*k*tb^2 + t^2 + 2*t*tb - 2*t*tf + tb^2 - 2*tb*tf + tf^2))/(6*k*tb);

        end
        if t>=tf-tb+ta && t<tf-ta
            PointCurrentTime = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/6 - (amax*(t + tb - tf - k*tb)*(t - tb - tf + 2*k*tb))/2 - amax*(2*tb - tf)*(tb - k*tb) + (amax*(tb - k*tb)*(tb - 2*k*tb))/2 - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
        if t>=tf-ta && t<=tf
            PointCurrentTime = theta0 - (amax*((tf - tb + k*tb)^2 - 2*tb*tf - 6*k*tb^2 + 2*tb*(tf - tb + k*tb) - 2*tf*(tf - tb + k*tb) + tb^2 + tf^2 + 6*k^2*tb^2))/6 + (amax*tb^2)/(3*k) + (amax*k^2*tb^2)/3 - amax*(2*tb - tf)*(tb - k*tb) + amax*(tb - k*tb)*(tb - 2*k*tb) - (amax*tb*(6*tb*k^2 - 6*tb*k + 3*tb))/(6*k) + (amax*(t^3 - 3*t^2*tf + 3*t*tf^2 - tf^3))/(6*k*tb) - (amax*tb^2*(7*k^3 - 12*k^2 + 6*k - 1))/(6*k);

        end
%         end
    end
    if flagNeedXG==1
        PointCurrentTime = legalizAnger(PointCurrentTime);
%         if PointCurrentTime>180
%             PointCurrentTime = -180 + (PointCurrentTime-180);
%         end
    end
    success = 1;
end





function TWOPOINTS = FindInnerEdgeTangentPoints()
    lidu = 0.5;
    OuterEdge = GetOuterEdgeOfPlaneWorkSpace(lidu);
    InnerEdge = GetInnerEdgeOfPlaneWorkSpace(lidu);

    figure
    plot(OuterEdge(:,1),OuterEdge(:,2),'.');
    hold on 
    plot(InnerEdge(:,1),InnerEdge(:,2),'.');
    hold on 
    axis([100 700 -500 500])

    InnerCH = [InnerEdge zeros(size(InnerEdge,1),1)];
    InnerCH = GetCHGrahamScan(InnerCH);
    plot(InnerCH(:,1),InnerCH(:,2),'-');
    hold on

    %�ҵ����е��Ǹ�ֱ�ߵ�����
    TWOPOINTS = [];
    lastdistance = norm(InnerCH(1,:)-InnerCH(2,:));
    for i=2:size(InnerCH,1)-1
        currentdistance = norm(InnerCH(i,:)-InnerCH(i+1,:));
        if norm(currentdistance-lastdistance)>10*lastdistance && i~=size(InnerCH,1)-1
            TWOPOINTS = [TWOPOINTS;[InnerCH(i,:);InnerCH(i+1,:)]];
        end
        lastdistance = currentdistance;
    end
    if size(TWOPOINTS,1)~=2
        error('���ڲ�͹�����ܰ������ַ�����');
    end
    plot(TWOPOINTS(:,1),TWOPOINTS(:,2),'o');
    hold on 
    TWOPOINTS(:,3) = [];

    [A,B,C] = GetLineABC(TWOPOINTS(1,:),TWOPOINTS(2,:));
    y=[140,690];
    z=(-A*y-C)/B;
    plot(y,z,'-');
    hold on
    
    if TWOPOINTS(1,2)>TWOPOINTS(2,2)
        tmpswap = TWOPOINTS(2,2);
        TWOPOINTS(2,2) = TWOPOINTS(1,2);
        TWOPOINTS(1,2) = tmpswap;
    end
end

function EdgePoints = GetOuterEdgeOfPlaneWorkSpace(lidu)
    GlobalDeclarationCommon
    EdgePoints = [];
    k3 = theta3Range(2);
%     lidu = 0.1;
    for k2=theta2Range(1):lidu:theta2Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end

    k2=theta2Range(1);
    for k3 = theta3Range(1):lidu:theta3Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
end


function EdgePoints = GetInnerEdgeOfPlaneWorkSpace(lidu) %�õ�ƽ����ڲ��Ե y-zϵ 
    GlobalDeclarationCommon
    EdgePoints = [];
    k3 = theta3Range(1);
%     lidu = 0.1;
    for k2=theta2Range(1):lidu:theta2Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end

    for k3 = theta3Range(1):lidu:theta3Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
%     for k2=theta2Range(1):5:theta2Range(2)
%         for k3 = theta3Range(1):5:theta3Range(2)
%             [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
%             plot(position1(2,4),position1(3,4),'.');
%             hold on
%             pause(0.1);
%         end
%     end

end

function EdgePoints = GetInnerEdgeOfPlaneWorkSpaceDown(lidu) %�õ�ƽ����ڲ��Ե y-zϵ �±���һ�� 
    GlobalDeclarationCommon
    EdgePoints = [];
    k3 = theta3Range(1);
%     lidu = 0.1;
    for k2=theta2Range(1):lidu:theta2Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
end

function EdgePoints = GetInnerEdgeOfPlaneWorkSpaceUp(lidu) %�õ�ƽ����ڲ��Ե y-zϵ �ϱ���һ�� 
    GlobalDeclarationCommon
    EdgePoints = [];
    k2 = theta2Range(2);
    for k3 = theta3Range(1):lidu:theta3Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
end

function [PointA,PointB] = RandGenratePointLineParallelground(TwoInnerTangentPoints,InnerEdgeDown,InnerEdgeUp) %���������㣬����������һ��ƽ���ڵ����ֱ���ϣ������ڻ��۹��� 
    [A,B,C] = GetLineABC(TwoInnerTangentPoints(1,:),TwoInnerTangentPoints(2,:));
    %     z=(-A*y-C)/B;
%     y=(-B*z-C)/A;
    
end

function [PointA,PointB] = RandGenratePointDirectLine(Range)%���������㣬����������һֱ���ϣ����ڻ��۹��� ������ֱ�߲�һ����ֱ�߿ɴ�� ������Range��Χ�� ���ǲ�����ת����
    distance = 2*Range(2);
    GlobalDeclarationCommon
    while (distance<Range(1) || distance>Range(2))
        
        theta1 = RandGenerateNumber(theta1Range(1),theta1Range(2),1);

        theta2A = RandGenerateNumber(theta2Range(1),theta2Range(2),1);
        theta3A = RandGenerateNumber(theta3Range(1),theta3Range(2),1);
        theta4A = theta4Range(1);

        theta2B = RandGenerateNumber(theta2Range(1),theta2Range(2),1);
        theta3B = RandGenerateNumber(theta3Range(1),theta3Range(2),1);
        theta4B = theta4Range(1);

        angleA = [theta1 theta2A theta3A theta4A];
        angleB = [theta1 theta2B theta3B theta4B];

        [position1A,~] = ForwardKinematics(angleA);
        [position1B,~] = ForwardKinematics(angleB);

        PointA = position1A(1:3,4)';
        PointB = position1B(1:3,4)';
        
        distance = norm(PointA-PointB);
    end
    
end






function resultPoints = RandGenerateStableBucketPoints(num,BucketStableRange) %�����е�۹���ĵѿ�������ϵ���������num���㣬��num�������theta4ʹ�ò����������ﲻ©�������ﲻ©��ǰ���ǲ��������ļн�����BucketStableRange
    BucketStableRange(1) = legalizAnger(BucketStableRange(1));
    BucketStableRange(2) = legalizAnger(BucketStableRange(2));
    count = 0;
    resultPoints = [];
    while count<num
        Points = RandGeneratePointsCoplanarWithManipulator(90,1);
        jointAngle = InverseKinematicsPos2Angle(Points);
%         jointAngle
        [~,YES] = groundAngleRangeTOtheta4Range(jointAngle(1),jointAngle(2),jointAngle(3),BucketStableRange);
        if YES == 1
            resultPoints = [resultPoints;Points];
            count = count+1
        end
    end
end

function PointsSet = RandGeneratePointsCoplanarWithManipulator(theta1,num)%����num�����е�۹���ĵ� ������theta1Ϊһ����ֵ)
    GlobalDeclarationCommon
    PointsSet = [];
    for i =1:num
        theta2A = RandGenerateNumber(theta2Range(1),theta2Range(2),1);
        theta3A = RandGenerateNumber(theta3Range(1),theta3Range(2),1);
        theta4A = theta4Range(1);
        angleA = [theta1 theta2A theta3A theta4A];
        [position1A,~] = ForwardKinematics(angleA);
        PointA = position1A(1:3,4)';
        PointsSet = [PointsSet;PointA];
    end
end


function result = CombineTheta123AndTheta4(Theta123,Theta4) %ʱ��������һ��
    if size(Theta123,2)>size(Theta4,2)
        tmp = Theta4(2,end);
        for i=size(Theta4,2)+1:size(Theta123,2)
            Theta4(1,i) = Theta123(1,i);
            Theta4(2,i) = tmp;
        end
    else
        if size(Theta123,2)<size(Theta4,2)
            tmp = Theta123(2:4,end);
            for i=size(Theta123,2)+1:size(Theta4,2)
                Theta123(1,i) = Theta4(1,i);
                Theta123(2:4,i) = tmp;
            end
        end
    end
    
    if size(Theta123,2)~=size(Theta4,2)
        error('�����߼�����');
    end
    result  =[];
    for i=1:size(Theta123,2)
        if norm(Theta123(1,i)-Theta4(1,i))<0.001
            result = [result,[Theta123(1,i);Theta123(2:4,i);Theta4(2,i)]];
        else
            error('�����߼�����');
        end
    end
end


function VisualizationExcavator(AngleSequence)
    %AngleSequence�ĵ�һ�б�����ʱ�䣬��2 3 4 5 �����ĸ��Ƕ�
    figure
    for i=1:10:size(AngleSequence,2)
%         [withground,YES] = groundAngleRangeTOtheta4Range(AngleSequence(2,i),AngleSequence(3,i),AngleSequence(4,i),[AngleSequence(5,i),AngleSequence(5,i)])
        PlotTheta1234(AngleSequence(2,i),AngleSequence(3,i),AngleSequence(4,i),AngleSequence(5,i));
        pause(0.1);
        if i+10<size(AngleSequence,2)
            clf
        end
    end
end

function JacoboSimplify11 = Getv50_Jacobo11(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify11 = cos(m0)*(tool*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + a3*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) - d4*sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - a1*cos(m0)*sin(k1) + d2*cos(m1)*sin(m0) - d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - d4*cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + d2*cos(k1)*cos(m0)*sin(m1)) - sin(m0)*(tool*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + d2*cos(m0)*cos(m1) - d4*sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + a1*sin(k1)*sin(m0) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - d4*cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - d2*cos(k1)*sin(m0)*sin(m1));

end

function JacoboSimplify12 = Getv50_Jacobo12(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify12 = - (cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))*(tool*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) - d4*sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - d4*cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) - (cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))*(a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) - a3*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - tool*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + d4*sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) + d4*cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)));

end

function JacoboSimplify13 = Getv50_Jacobo13(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify13 = (sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)))*(tool*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - d4*sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - d4*cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))) - (sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))*(tool*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + a3*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - d4*sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - d4*cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))));

end

function JacoboSimplify14 = Getv50_Jacobo14(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify14 = tool*(cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) - tool*(sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))));

end

function JacoboSimplify21 = Getv50_Jacobo21(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify21 = cos(m0)*(a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + tool*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a1*cos(k1) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d2*sin(k1)*sin(m1) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1));

end

function JacoboSimplify22 = Getv50_Jacobo22(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify22 = (cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))*(a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + tool*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1)) - sin(k1)*sin(m1)*(tool*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) - d4*sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - d4*cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)));

end

function JacoboSimplify23 = Getv50_Jacobo23(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify23 = - (sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))*(tool*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)))) + a3*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - d4*sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - d4*cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))) - (sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)))*(tool*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)));

end

function JacoboSimplify24 = Getv50_Jacobo24(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify24 = - tool*(sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))))*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) - tool*(cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(cos(k4)*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - sin(k4)*sin(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + cos(m3)*sin(k4)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))));

end

function JacoboSimplify31 = Getv50_Jacobo31(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify31 = sin(m0)*(a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + tool*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a1*cos(k1) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d2*sin(k1)*sin(m1) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1));

end

function JacoboSimplify32 = Getv50_Jacobo32(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify32 = (cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))*(a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + tool*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1)) + sin(k1)*sin(m1)*(a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) - a3*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - tool*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + d4*sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) + d4*cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)));

end

function JacoboSimplify33 = Getv50_Jacobo33(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify33 = - (sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))*(tool*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) + a3*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - d4*sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - d4*cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)))) - (sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)))*(tool*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1))) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)));

end

function JacoboSimplify34 = Getv50_Jacobo34(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify34 = - tool*(cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)))*(cos(k4)*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) - sin(k4)*sin(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + cos(m3)*sin(k4)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)))) - tool*(cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))))*(cos(k4)*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) - cos(m3)*sin(k4)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + sin(k4)*sin(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)));

end

function JacoboSimplify41 = GetOmiga50_Jacobo41(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify41 = 0;
end

function JacoboSimplify42 = GetOmiga50_Jacobo42(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify42 = sin(k1)*sin(m1);
    
end

function JacoboSimplify43 = GetOmiga50_Jacobo43(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify43 = sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1);
    
end

function JacoboSimplify44 = GetOmiga50_Jacobo44(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify44 = cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2));

end

function JacoboSimplify51 = GetOmiga50_Jacobo51(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify51 = -sin(m0);
    
end

function JacoboSimplify52 = GetOmiga50_Jacobo52(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify52 = - cos(m1)*sin(m0) - cos(k1)*cos(m0)*sin(m1);
    
end

function JacoboSimplify53 = GetOmiga50_Jacobo53(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify53 = sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1));

end

function JacoboSimplify54 = GetOmiga50_Jacobo54(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify54 = cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) + sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)));

end

function JacoboSimplify61 = GetOmiga50_Jacobo61(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify61 = cos(m0);
    
end

function JacoboSimplify62 = GetOmiga50_Jacobo62(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify62 = cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1);
    
end

function JacoboSimplify63 = GetOmiga50_Jacobo63(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify63 = cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0));

end

function JacoboSimplify64 = GetOmiga50_Jacobo64(k1,k2,k3,k4)
    GlobalDeclarationCommon
    JacoboSimplify64 = - sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) - cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)));

end








function JacoboMatrix = GetvOmiga50_JacoboMatrix(theta1,theta2,theta3,theta4)
%�����и���ʱ��ĵ㣬����4-6����theta1�����������ǲ���ġ�
    k1 = DegToRad(theta1);
    k2 = DegToRad(theta2);
    k3 = DegToRad(theta3);
    k4 = DegToRad(theta4);
    
    J11 = Getv50_Jacobo11(k1,k2,k3,k4);
    J12 = Getv50_Jacobo12(k1,k2,k3,k4);
    J13 = Getv50_Jacobo13(k1,k2,k3,k4);
    J14 = Getv50_Jacobo14(k1,k2,k3,k4);
    J21 = Getv50_Jacobo21(k1,k2,k3,k4);
    J22 = Getv50_Jacobo22(k1,k2,k3,k4);
    J23 = Getv50_Jacobo23(k1,k2,k3,k4);
    J24 = Getv50_Jacobo24(k1,k2,k3,k4);
    J31 = Getv50_Jacobo31(k1,k2,k3,k4);
    J32 = Getv50_Jacobo32(k1,k2,k3,k4);
    J33 = Getv50_Jacobo33(k1,k2,k3,k4);
    J34 = Getv50_Jacobo34(k1,k2,k3,k4);
    J41 = GetOmiga50_Jacobo41(k1,k2,k3,k4);
    J42 = GetOmiga50_Jacobo42(k1,k2,k3,k4);
    J43 = GetOmiga50_Jacobo43(k1,k2,k3,k4);
    J44 = GetOmiga50_Jacobo44(k1,k2,k3,k4);
    J51 = GetOmiga50_Jacobo51(k1,k2,k3,k4);
    J52 = GetOmiga50_Jacobo52(k1,k2,k3,k4);
    J53 = GetOmiga50_Jacobo53(k1,k2,k3,k4);
    J54 = GetOmiga50_Jacobo54(k1,k2,k3,k4);
    J61 = GetOmiga50_Jacobo61(k1,k2,k3,k4);
    J62 = GetOmiga50_Jacobo62(k1,k2,k3,k4);
    J63 = GetOmiga50_Jacobo63(k1,k2,k3,k4);
    J64 = GetOmiga50_Jacobo64(k1,k2,k3,k4);
    JacoboMatrix=[J11,J12,J13,J14;J21,J22,J23,J24;J31,J32,J33,J34;J41,J42,J43,J44;J51,J52,J53,J54;J61,J62,J63,J64];
end

function [vtheta2,vtheta3,vtheta4] = GetCurrentvthetaBucketTip(JacoboMatrix,BeginPoint,EndPoint,lastvtheta2,lastvtheta3,lastvtheta4,Vtheta2Max,Vtheta3Max,Vtheta4Max,atheta2max,atheta3max,atheta4max)
   %20200807������������ǲ����õ�
    GlobalDeclarationCommon
    vectorbe = EndPoint-BeginPoint;
    xn = vectorbe(1)/norm(vectorbe);
    yn = vectorbe(2)/norm(vectorbe);
    
    zn = vectorbe(3)/norm(vectorbe);
    x0 = BeginPoint(1);
    y0 = BeginPoint(2);
    z0 = BeginPoint(3);
    
    x1 = EndPoint(1);
    y1 = EndPoint(2);
    z1 = EndPoint(3);
    
    %     a11 = JacoboMatrix(1,1); 
    a12 = JacoboMatrix(1,2); a13 = JacoboMatrix(1,3); a14 = JacoboMatrix(1,4);
%     a21 = JacoboMatrix(2,1); 
    a22 = JacoboMatrix(2,2); a23 = JacoboMatrix(2,3); a24 = JacoboMatrix(2,4);
%     a31 = JacoboMatrix(3,1); 
    a32 = JacoboMatrix(3,2); a33 = JacoboMatrix(3,3); a34 = JacoboMatrix(3,4);
%     a32 = (z1-z0)/(x1-x0)*a12; a33 = (z1-z0)/(x1-x0)*a13; a34 = (z1-z0)/(x1-x0)*a14;
    

    currentvtheta2Range = [lastvtheta2-atheta2max*tinterval,lastvtheta2+atheta2max*tinterval];
    currentvtheta2Range = GetIntersection(currentvtheta2Range,[-Vtheta2Max,Vtheta2Max]);
    
    currentvtheta3Range = [lastvtheta3-atheta3max*tinterval,lastvtheta3+atheta3max*tinterval];
    currentvtheta3Range = GetIntersection(currentvtheta3Range,[-Vtheta3Max,Vtheta3Max]);
    
    currentvtheta4Range = [lastvtheta4-atheta4max*tinterval,lastvtheta4+atheta4max*tinterval];
    currentvtheta4Range = GetIntersection(currentvtheta4Range,[-Vtheta4Max,Vtheta4Max]);
    
    if isempty(currentvtheta2Range)==1 || isempty(currentvtheta3Range)==1 || isempty(currentvtheta4Range)==1
        error('��������ˣ����ܱ�֤�ٶȵ�������');
    end
    
    currentvtheta2Range(1) = currentvtheta2Range(1)*pi/180; currentvtheta2Range(2) = currentvtheta2Range(2)*pi/180;
    currentvtheta3Range(1) = currentvtheta3Range(1)*pi/180; currentvtheta3Range(2) = currentvtheta3Range(2)*pi/180;
    currentvtheta4Range(1) = currentvtheta4Range(1)*pi/180; currentvtheta4Range(2) = currentvtheta4Range(2)*pi/180;
    
    currentvtheta2Range = Sortqujian(currentvtheta2Range);
    currentvtheta3Range = Sortqujian(currentvtheta3Range);
    currentvtheta4Range = Sortqujian(currentvtheta4Range);
    
    vtheta4FitableRange = [];
    
%     vtheta2 = ((a13*a24 - a14*a23)/(a12*a23 - a13*a22))*vtheta4 + ((a23*xn - a13*yn)/(a12*a23 - a13*a22))*k;
%     k = ((a12*a23 - a13*a22)/(a23*xn - a13*yn))*vtheta2 + (-(a13*a24 - a14*a23)/(a23*xn - a13*yn))*vtheta4;
    k_xishu1 = ((a12*a23 - a13*a22)/(a23*xn - a13*yn));
    k_xishu2 = (-(a13*a24 - a14*a23)/(a23*xn - a13*yn));
    %     vtheta3 = (-(a12*a24 - a14*a22)/(a12*a23 - a13*a22))*vtheta4 + (-(a22*xn - a12*yn)/(a12*a23 - a13*a22))*k;
%     k=(-(a12*a23 - a13*a22)/(a22*xn - a12*yn))*vtheta3 + (-(a12*a24 - a14*a22)/(a22*xn - a12*yn))*vtheta4;
    k_xishu3 = (-(a12*a23 - a13*a22)/(a22*xn - a12*yn));
    k_xishu4 = (-(a12*a24 - a14*a22)/(a22*xn - a12*yn));
    
    if (a23*xn - a13*yn)==0 || (a23*xn - a13*yn)==0 || (a22*xn - a12*yn)==0 || (a22*xn - a12*yn)==0
        error('��ʱ�㷨���ܴ������������������˼��һ��');
    end

    if k_xishu1>=0 && k_xishu3>=0
        LA = currentvtheta2Range(1);
        LB = currentvtheta2Range(2);
        LC = currentvtheta3Range(1);
        LD = currentvtheta3Range(2);
    else
        if k_xishu1>=0 && k_xishu3<0
            LA = currentvtheta2Range(1);
            LB = currentvtheta2Range(2);
            LC = currentvtheta3Range(2);
            LD = currentvtheta3Range(1);
        else
            if k_xishu1<0&&k_xishu3>=0
                LA = currentvtheta2Range(2);
                LB = currentvtheta2Range(1);
                LC = currentvtheta3Range(1);
                LD = currentvtheta3Range(2);
            else
                LA = currentvtheta2Range(2);
                LB = currentvtheta2Range(1);
                LC = currentvtheta3Range(2);
                LD = currentvtheta3Range(1);
            end
        end
    end
    
    if k_xishu4>k_xishu2
%         tmp1 = [(k_xishu1*LB-k_xishu3*LC)/(k_xishu4-k_xishu2) currentvtheta4Range(2)];
%         tmp2 = [currentvtheta4Range(1) (k_xishu1*LA-k_xishu3*LD)/(k_xishu4-k_xishu2)];
        vtheta4FitableRange = [(k_xishu1*LA-k_xishu3*LD)/(k_xishu4-k_xishu2) (k_xishu1*LB-k_xishu3*LC)/(k_xishu4-k_xishu2)];
    else
        if k_xishu4<k_xishu2
%             tmp1 = [currentvtheta4Range(1) (k_xishu1*LB-k_xishu3*LC)/(k_xishu4-k_xishu2)];
%             tmp2 = [(k_xishu1*LA-k_xishu3*LD)/(k_xishu4-k_xishu2) currentvtheta4Range(2)];
            vtheta4FitableRange = [(k_xishu1*LB-k_xishu3*LC)/(k_xishu4-k_xishu2) (k_xishu1*LA-k_xishu3*LD)/(k_xishu4-k_xishu2)];
        else
            if k_xishu1*LB-k_xishu3*LC<0 || k_xishu1*LA-k_xishu3*LD>0 %˵��vtheta4������ȡֵ������ʹ��k�н�
                error('vtheta4������ȡֵ������ʹ��k�н�,�ٶ�����ٶ�Լ�����õ�����');
            else
                %��ʱ˵�����е�vtheta4����ʹ��k�н�
                vtheta4FitableRange = currentvtheta4Range;
            end
        end
    end
    if vtheta4FitableRange(1)>vtheta4FitableRange(2)
        error('�����߼��д�');
    end
    vtheta4FitableRange = GetIntersection(currentvtheta4Range,vtheta4FitableRange);
    if isempty(vtheta4FitableRange)==1
        error('vtheta4������ȡֵ������ʹ��k�н�,�ٶ�����ٶ�Լ�����õ�����');
    end
    vtheta4 = max(vtheta4FitableRange);
    
    tmp1 = ((a12*a23 - a13*a22)/(a23*xn - a13*yn))*currentvtheta2Range(1) + (-(a13*a24 - a14*a23)/(a23*xn - a13*yn))*vtheta4;
    tmp2 = ((a12*a23 - a13*a22)/(a23*xn - a13*yn))*currentvtheta2Range(2) + (-(a13*a24 - a14*a23)/(a23*xn - a13*yn))*vtheta4;
    if tmp2<0 && tmp1<0
        error('vtheta4��ȡֵ���ԣ�ʹ�ò���vtheta2ȡ���٣����������żȶ�������');
    else
        if tmp1<0
            tmp1 = 0;
        end
        if tmp2<0
            tmp2 = 0;
        end
    end
    k_vtheta2Range = [min([tmp1,tmp2]) max([tmp1,tmp2])];
    

    tmp1 = (-(a12*a23 - a13*a22)/(a22*xn - a12*yn))*currentvtheta3Range(1) + (-(a12*a24 - a14*a22)/(a22*xn - a12*yn))*vtheta4;
    tmp2 = (-(a12*a23 - a13*a22)/(a22*xn - a12*yn))*currentvtheta3Range(2) + (-(a12*a24 - a14*a22)/(a22*xn - a12*yn))*vtheta4;
    if tmp2<0 && tmp1<0
        error('vtheta4��ȡֵ���ԣ�ʹ�ò���vtheta3ȡ���٣����������żȶ�������');
    else
        if tmp1<0
            tmp1 = 0;
        end
        if tmp2<0
            tmp2 = 0;
        end
    end
    k_vtheta3Range = [min([tmp1,tmp2]) max([tmp1,tmp2])];
    
    k_Range = GetIntersection(k_vtheta2Range,k_vtheta3Range);
    k_Rangetmp = [min(k_Range) max(k_Range)];
    k_Range = k_Rangetmp;
    %���ˣ�Ҫ������vtheta4ֵ�²���ֱ���˶��ҳ���ָ���ķ���k��ֵӦ������k_Range��Χ
    
    
    %Ȼ���ٸ���k��ȡֵ��Χ����һ������ȡk��ֵ����֪vtheta4����vtheta2 vtheta3 ����� 
    
    vtheta2L = ((a13*a24 - a14*a23)/(a12*a23 - a13*a22))*vtheta4 + ((a23*xn - a13*yn)/(a12*a23 - a13*a22))*k_Range(1);
    vtheta2U = ((a13*a24 - a14*a23)/(a12*a23 - a13*a22))*vtheta4 + ((a23*xn - a13*yn)/(a12*a23 - a13*a22))*k_Range(2);
    testrange = [min([vtheta2L vtheta2U]),max([vtheta2L vtheta2U])];
    test = GetIntersection(testrange,currentvtheta2Range);
    if norm(test-testrange)>0.001
        error('�߼�����');
    end
    
    vtheta3L = (-(a12*a24 - a14*a22)/(a12*a23 - a13*a22))*vtheta4 + (-(a22*xn - a12*yn)/(a12*a23 - a13*a22))*k_Range(1);
    vtheta3U = (-(a12*a24 - a14*a22)/(a12*a23 - a13*a22))*vtheta4 + (-(a22*xn - a12*yn)/(a12*a23 - a13*a22))*k_Range(2);
    testrange = [min([vtheta3L vtheta3U]),max([vtheta3L vtheta3U])];
    test = GetIntersection(testrange,currentvtheta3Range);
    if norm(test-testrange)>0.001
        error('�߼�����');
    end
    %�ϱߵ���δ������쳣������
    
    k_power = 1;
    k = k_Range(1) + k_power*(k_Range(2)-k_Range(1));
    
    vtheta2 = ((a13*a24 - a14*a23)/(a12*a23 - a13*a22))*vtheta4 + ((a23*xn - a13*yn)/(a12*a23 - a13*a22))*k;
    vtheta3 = (-(a12*a24 - a14*a22)/(a12*a23 - a13*a22))*vtheta4 + (-(a22*xn - a12*yn)/(a12*a23 - a13*a22))*k;
    
    vxtmp = a12*vtheta2+a13*vtheta3+a14*vtheta4;
    vytmp = a22*vtheta2+a23*vtheta3+a24*vtheta4;
    vztmp = a32*vtheta2+a33*vtheta3+a34*vtheta4;
    cross([vxtmp vytmp vztmp],[xn yn zn]) %����Ҳ���С���������
    
    
    vtheta2 = vtheta2*180/pi;
    vtheta3 = vtheta3*180/pi;
    vtheta4 = vtheta4*180/pi;
    
%     figure
%     for vtheta2=currentvtheta2Range(1):(currentvtheta2Range(2)-currentvtheta2Range(1))/20:currentvtheta2Range(2)
%         for vtheta4=currentvtheta4Range(1):(currentvtheta4Range(2)-currentvtheta4Range(1))/20:currentvtheta4Range(2)
%             k = ((a12*a23 - a13*a22)/(a23*xn - a13*yn))*vtheta2 + (-(a13*a24 - a14*a23)/(a23*xn - a13*yn))*vtheta4;
%             plot3(vtheta2,vtheta4,k,'r.');
%             hold on 
%         end
%     end
end

function [omigayFitableRange,k_xishu] = GetCurrentvthetaBucketTipVxyOmigay(currentvtheta2Range,currentvtheta3Range,currentvtheta4Range,JacoboMatrix,BeginPoint,EndPoint)
 %�ò���ĩ��Vxy���ٶȺͲ����Ľ��ٶ�omigayΪ����
    GlobalDeclarationCommon
    vectorbe = EndPoint-BeginPoint;
    xn = vectorbe(1)/norm(vectorbe);
    yn = vectorbe(2)/norm(vectorbe);
    
    zn = vectorbe(3)/norm(vectorbe);
    x0 = BeginPoint(1);
    y0 = BeginPoint(2);
    z0 = BeginPoint(3);
    
    x1 = EndPoint(1);
    y1 = EndPoint(2);
    z1 = EndPoint(3);
    
    %     a11 = JacoboMatrix(1,1); 
    a12 = JacoboMatrix(1,2); a13 = JacoboMatrix(1,3); a14 = JacoboMatrix(1,4);
%     a21 = JacoboMatrix(2,1); 
    a22 = JacoboMatrix(2,2); a23 = JacoboMatrix(2,3); a24 = JacoboMatrix(2,4);
%     a31 = JacoboMatrix(3,1); 
    a32 = JacoboMatrix(3,2); a33 = JacoboMatrix(3,3); a34 = JacoboMatrix(3,4);
%     a32 = (z1-z0)/(x1-x0)*a12; a33 = (z1-z0)/(x1-x0)*a13; a34 = (z1-z0)/(x1-x0)*a14;
    a42 = JacoboMatrix(4,2); a43 = JacoboMatrix(4,3); a44 = JacoboMatrix(4,4);
    a52 = JacoboMatrix(5,2); a53 = JacoboMatrix(5,3); a54 = JacoboMatrix(5,4);
    a62 = JacoboMatrix(6,2); a63 = JacoboMatrix(6,3); a64 = JacoboMatrix(6,4);
    


    
    omigayLimitRange = [-10,10]; %��ʱ�����ó�����������֮����ܻ�� ��λ�ǻ�������-10��10һ�����һ��������
%     vtheta2 = ((a23*a54*xn - a24*a53*xn - a13*a54*yn + a14*a53*yn)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52))*k + ((a13*a24 - a14*a23)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52))*omigay;
%     k = ((a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52)/(a23*a54*xn - a24*a53*xn - a13*a54*yn + a14*a53*yn))*vtheta2 + (-(a13*a24 - a14*a23)/(a23*a54*xn - a24*a53*xn - a13*a54*yn + a14*a53*yn))*omigay;
    k_xishu1 = ((a23*a54*xn - a24*a53*xn - a13*a54*yn + a14*a53*yn)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52));
    k_xishu2 = ((a13*a24 - a14*a23)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52));
                                    
%     vtheta3 = (-(a22*a54*xn - a24*a52*xn - a12*a54*yn + a14*a52*yn)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52))*k + (-(a12*a24 - a14*a22)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52))*omigay;
%     k = (-(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52)/(a22*a54*xn - a24*a52*xn - a12*a54*yn + a14*a52*yn))*vtheta3 + (-(a12*a24 - a14*a22)/(a22*a54*xn - a24*a52*xn - a12*a54*yn + a14*a52*yn))*omigay;
    k_xishu3 =(-(a22*a54*xn - a24*a52*xn - a12*a54*yn + a14*a52*yn)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52));
    k_xishu4 =(-(a12*a24 - a14*a22)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52));
    
%     vtheta4 = ((a22*a53*xn - a23*a52*xn - a12*a53*yn + a13*a52*yn)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52))*k + ((a12*a23 - a13*a22)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52))*omigay;
%     k = ((a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52)/(a22*a53*xn - a23*a52*xn - a12*a53*yn + a13*a52*yn))*vtheta4 + (-(a12*a23 - a13*a22)/(a22*a53*xn - a23*a52*xn - a12*a53*yn + a13*a52*yn))*omigay;
    k_xishu5 = ((a22*a53*xn - a23*a52*xn - a12*a53*yn + a13*a52*yn)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52));
    k_xishu6 = ((a12*a23 - a13*a22)/(a12*a23*a54 - a12*a24*a53 - a13*a22*a54 + a13*a24*a52 + a14*a22*a53 - a14*a23*a52));
    
    k_xishu = [k_xishu1,k_xishu2,k_xishu3,k_xishu4,k_xishu5,k_xishu6];
    omigayFitableRange = [];
      
end

function k_xishu = Getk_xishuBucketTipVxzOmigay(CurrentPoint,JacoboMatrix,EndPoint,BeginPoint)
 %�ò���ĩ��Vxy���ٶȺͲ����Ľ��ٶ�omigayΪ����
    GlobalDeclarationCommon
%     vectorbe = EndPoint-CurrentPoint;
    vectorbe = EndPoint-BeginPoint;
    
    xn = vectorbe(1)/norm(vectorbe);
    yn = vectorbe(2)/norm(vectorbe);
    zn = vectorbe(3)/norm(vectorbe);

    
    %     a11 = JacoboMatrix(1,1); 
    a12 = JacoboMatrix(1,2); a13 = JacoboMatrix(1,3); a14 = JacoboMatrix(1,4);
%     a21 = JacoboMatrix(2,1); 
    a22 = JacoboMatrix(2,2); a23 = JacoboMatrix(2,3); a24 = JacoboMatrix(2,4);
%     a31 = JacoboMatrix(3,1); 
    a32 = JacoboMatrix(3,2); a33 = JacoboMatrix(3,3); a34 = JacoboMatrix(3,4);
%     a32 = (z1-z0)/(x1-x0)*a12; a33 = (z1-z0)/(x1-x0)*a13; a34 = (z1-z0)/(x1-x0)*a14;
    a42 = JacoboMatrix(4,2); a43 = JacoboMatrix(4,3); a44 = JacoboMatrix(4,4);
    a52 = JacoboMatrix(5,2); a53 = JacoboMatrix(5,3); a54 = JacoboMatrix(5,4);
    a62 = JacoboMatrix(6,2); a63 = JacoboMatrix(6,3); a64 = JacoboMatrix(6,4);
    

%     vtheta2 = ((a33*a54*xn - a34*a53*xn - a13*a54*zn + a14*a53*zn)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52))*k + ((a13*a34 - a14*a33)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52))*omigay;
%     k = ((a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52)/(a33*a54*xn - a34*a53*xn - a13*a54*zn + a14*a53*zn))*vtheta2 + (-(a13*a34 - a14*a33)/(a33*a54*xn - a34*a53*xn - a13*a54*zn + a14*a53*zn))*omigay;
    k_xishu1 = ((a33*a54*xn - a34*a53*xn - a13*a54*zn + a14*a53*zn)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52));
    k_xishu2 = ((a13*a34 - a14*a33)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52));

%     vtheta3 = (-(a32*a54*xn - a34*a52*xn - a12*a54*zn + a14*a52*zn)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52))*k + (-(a12*a34 - a14*a32)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52))*omigay;
%     k = (-(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52)/(a32*a54*xn - a34*a52*xn - a12*a54*zn + a14*a52*zn))*vtheta3 + (-(a12*a34 - a14*a32)/(a32*a54*xn - a34*a52*xn - a12*a54*zn + a14*a52*zn))*omigay;
    k_xishu3 = (-(a32*a54*xn - a34*a52*xn - a12*a54*zn + a14*a52*zn)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52));
    k_xishu4 = (-(a12*a34 - a14*a32)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52));
    
%     vtheta4 = ((a32*a53*xn - a33*a52*xn - a12*a53*zn + a13*a52*zn)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52))*k + ((a12*a33 - a13*a32)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52))*omigay;
%     k = ((a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52)/(a32*a53*xn - a33*a52*xn - a12*a53*zn + a13*a52*zn))*vtheta4 + (-(a12*a33 - a13*a32)/(a32*a53*xn - a33*a52*xn - a12*a53*zn + a13*a52*zn))*omigay;
    k_xishu5 = ((a32*a53*xn - a33*a52*xn - a12*a53*zn + a13*a52*zn)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52));
    k_xishu6 = ((a12*a33 - a13*a32)/(a12*a33*a54 - a12*a34*a53 - a13*a32*a54 + a13*a34*a52 + a14*a32*a53 - a14*a33*a52));
    
    k_xishu = [k_xishu1,k_xishu2,k_xishu3,k_xishu4,k_xishu5,k_xishu6];
end

% function k_FitableRange = GetkRangeFromOmigay(omigay,k_xishu,currentvtheta2Range,currentvtheta3Range,currentvtheta4Range)
%     
% 
%     ktmp1 = [k_xishu(1)*currentvtheta2Range(1)+k_xishu(2)*omigay k_xishu(1)*currentvtheta2Range(2)+k_xishu(2)*omigay];
%     ktmp2 = [k_xishu(3)*currentvtheta3Range(1)+k_xishu(4)*omigay k_xishu(3)*currentvtheta3Range(2)+k_xishu(4)*omigay];
%     ktmp3 = [k_xishu(5)*currentvtheta4Range(1)+k_xishu(6)*omigay k_xishu(5)*currentvtheta4Range(2)+k_xishu(6)*omigay];
%     
%     k_FitableRange = GetThreeQujianIntersection(ktmp1,ktmp2,ktmp3);
%     
%     if isempty(k_FitableRange)==1
%         error('�����߼�����,���������һ�����϶��Ǿ�����һ���ļ��飬�ܹ��ҳ�һ����ģ�');
%     end
% end

% function [x,y] = GetTwoLineJiaodian(k1,b1,k2,b2)
%     if k1==k2
%         error('��������齻����޽���');
%     end
%     x = (b2-b1)/(k1-k2);
%     y = k1*x+b1;
% end

function [vtheta2,vtheta3,vtheta4,kthistime,omigay] = GetCurrentvthetaBucketTipvtheta234komiga(omigayDirection,k_power,omiga_power,CurrentPoint,JacoboMatrix,BeginPoint,EndPoint,lastvtheta2,lastvtheta3,lastvtheta4,Vtheta2Max,Vtheta3Max,Vtheta4Max,atheta2max,atheta3max,atheta4max)
    GlobalDeclarationCommon

    currentvtheta2Range = [lastvtheta2-atheta2max*tinterval,lastvtheta2+atheta2max*tinterval];
    currentvtheta2Range = GetIntersection(currentvtheta2Range,[-Vtheta2Max,Vtheta2Max]);
    
    currentvtheta3Range = [lastvtheta3-atheta3max*tinterval,lastvtheta3+atheta3max*tinterval];
    currentvtheta3Range = GetIntersection(currentvtheta3Range,[-Vtheta3Max,Vtheta3Max]);
    
    currentvtheta4Range = [lastvtheta4-atheta4max*tinterval,lastvtheta4+atheta4max*tinterval];
    currentvtheta4Range = GetIntersection(currentvtheta4Range,[-Vtheta4Max,Vtheta4Max]);
    
    if isempty(currentvtheta2Range)==1 || isempty(currentvtheta3Range)==1 || isempty(currentvtheta4Range)==1
        error('��������ˣ����ܱ�֤�ٶȵ�������');
    end
    
    currentvtheta2Range(1) = currentvtheta2Range(1)*pi/180; currentvtheta2Range(2) = currentvtheta2Range(2)*pi/180;
    currentvtheta3Range(1) = currentvtheta3Range(1)*pi/180; currentvtheta3Range(2) = currentvtheta3Range(2)*pi/180;
    currentvtheta4Range(1) = currentvtheta4Range(1)*pi/180; currentvtheta4Range(2) = currentvtheta4Range(2)*pi/180;
    
    currentvtheta2Range = Sortqujian(currentvtheta2Range);
    currentvtheta3Range = Sortqujian(currentvtheta3Range);
    currentvtheta4Range = Sortqujian(currentvtheta4Range);

%     [omigayFitableRangeA,k_xishuA] = GetCurrentvthetaBucketTipVxyOmigay(currentvtheta2Range,currentvtheta3Range,currentvtheta4Range,JacoboMatrix,BeginPoint,EndPoint);
    k_xishuB = Getk_xishuBucketTipVxzOmigay(CurrentPoint,JacoboMatrix,EndPoint,BeginPoint);
    
    
    %��ɽ����� ����Ϊk ����Ϊomigay ���㼸���㽻��
    kt = 1000; %����һ�����������ܴﵽ��kֵ k=1000ʱ��ζ��һ��ĩ����10��ÿ��
    ktdown = 0;
    Points2 = [ktdown (currentvtheta2Range(1)-k_xishuB(1)*ktdown)/k_xishuB(2) 0;ktdown (currentvtheta2Range(2)-k_xishuB(1)*ktdown)/k_xishuB(2) 0;kt (currentvtheta2Range(1)-k_xishuB(1)*kt)/k_xishuB(2) 0;kt (currentvtheta2Range(2)-k_xishuB(1)*kt)/k_xishuB(2) 0];
    Points3 = [ktdown (currentvtheta3Range(1)-k_xishuB(3)*ktdown)/k_xishuB(4) 0;ktdown (currentvtheta3Range(2)-k_xishuB(3)*ktdown)/k_xishuB(4) 0;kt (currentvtheta3Range(1)-k_xishuB(3)*kt)/k_xishuB(4) 0;kt (currentvtheta3Range(2)-k_xishuB(3)*kt)/k_xishuB(4) 0];
    Points4 = [ktdown (currentvtheta4Range(1)-k_xishuB(5)*ktdown)/k_xishuB(6) 0;ktdown (currentvtheta4Range(2)-k_xishuB(5)*ktdown)/k_xishuB(6) 0;kt (currentvtheta4Range(1)-k_xishuB(5)*kt)/k_xishuB(6) 0;kt (currentvtheta4Range(2)-k_xishuB(5)*kt)/k_xishuB(6) 0];

    CommonPoints1 = GetConvexHullIntersection(Points2,Points3);
    CommonPoints = GetConvexHullIntersection(CommonPoints1,Points4); %20200814 �������bug 
    
%  figure
%     PlotConvexHull(Points2,'r.','r-');
%     PlotConvexHull(Points3,'g.','g-');
%     PlotConvexHull(Points4,'b.','b-');
%     PlotConvexHull(CommonPoints,'ko','k-');

    
    if omigayDirection>0
        ConstrainOmigay = [ktdown 0 0; ktdown 100 0;kt 0 0;kt 100 0];
    else
        if omigayDirection < 0
            ConstrainOmigay = [ktdown 0 0; ktdown -100 0;kt 0 0;kt -100 0];
        else
            ConstrainOmigay = [ktdown 0 0; ktdown 0 0;kt 0 0;kt 0 0];
        end
    end
    CommonPoints = GetConvexHullIntersection(CommonPoints,ConstrainOmigay);
    
%     [min(CommonPoints(:,2)),max(CommonPoints(:,2))]
    if isempty(CommonPoints)==1
            figure
        PlotConvexHull(Points2,'r.','r-');
        PlotConvexHull(Points3,'g.','g-');
        PlotConvexHull(Points4,'b.','b-');
        PlotConvexHull(CommonPoints,'ko','k-');
        error('���õĲ���û�취�õ�����Լ���Ľ�');
    end
    
    
    %�ڿ��н⼯������ѡ��һ���н�
    kthistimeRange = [min(CommonPoints(:,1)),max(CommonPoints(:,1))];
%     k_power = 0.9; ��Χ��(0-1)
    kthistime = kthistimeRange(1) +k_power*(kthistimeRange(2)-kthistimeRange(1));
    omigayRangetmp = GetConvexHullIntersection(CommonPoints,[kthistime 10 0;kthistime -10 0]);%��omigayһ�����Բ�����ȡ��ֵ-10 10
    if size(omigayRangetmp,1)==2
        omigayRange = [min(omigayRangetmp(:,2)),max(omigayRangetmp(:,2))];
    else
        if size(omigayRangetmp,1)==1
            omigayRange = omigayRangetmp(2);
        else
            error('�����߼�����');
        end
    end
%     omiga_power = 0.9; ��Χ��(0-1)
    if size(omigayRange,2)==1
        omigay = omigayRange;
    else
        if abs(omigayRange(end))>=abs(omigayRange(1))
            omigay = omigayRange(1) + (omigayRange(end)-omigayRange(1))*(omiga_power);
        else
            omigay = omigayRange(1) + (omigayRange(end)-omigayRange(1))*(1-omiga_power);
        end
    end
%     omigayRange
%     
%     vtheta2 = k_xishuA(1)*kthistime+k_xishuA(2)*omigay;
%     vtheta3 = k_xishuA(3)*kthistime+k_xishuA(4)*omigay;
%     vtheta4 = k_xishuA(5)*kthistime+k_xishuA(6)*omigay;
%     
    %����ٶ�
    vtheta2 = k_xishuB(1)*kthistime+k_xishuB(2)*omigay;
    vtheta3 = k_xishuB(3)*kthistime+k_xishuB(4)*omigay;
    vtheta4 = k_xishuB(5)*kthistime+k_xishuB(6)*omigay; %���㼸���㽻��
    
    if isempty(GetIntersection(currentvtheta2Range,vtheta2)) || isempty(GetIntersection(currentvtheta3Range,vtheta3)) || isempty(GetIntersection(currentvtheta4Range,vtheta4))
        error('k��omigay���õĲ���,�����ٶȳ�����Լ��')
    end
    
    vtheta2 = vtheta2*180/pi;
    vtheta3 = vtheta3*180/pi;
    vtheta4 = vtheta4*180/pi;
end

function [Intersection,YES] = GetThreeQujianIntersection(qujian1,qujian2,qujian3)
%�õ���������Ľ����������������û�н�������ôYES=0��intersectionΪ�գ�����Ϊ1��intersectionΪʵ�ʵĽ���
%     if qujian1(1)>qujian1(2)
%         tmp = qujian1(1);
%         qujian1(1) = qujian1(2);
%         qujian1(2) = tmp;
%     end
%     if qujian2(1)>qujian2(2)
%         tmp = qujian2(1);
%         qujian2(1) = qujian2(2);
%         qujian2(2) = tmp;
%     end
%     if qujian3(1)>qujian3(2)
%         tmp = qujian3(1);
%         qujian3(1) = qujian3(2);
%         qujian3(2) = tmp;
%     end
    tmp1 = GetIntersection(qujian1,qujian2);
    tmp2 = GetIntersection(qujian1,qujian3);
    Intersection = GetIntersection(tmp1,tmp2);
    if isempty(Intersection)==1
        YES = 0;
    else
        YES = 1;
    end
end

function Sortedqujian = Sortqujian(qujian)
    if qujian(1)>qujian(2)
        tmp = qujian(1);
        qujian(1)=qujian(2);
        qujian(2) = tmp;
        Sortedqujian = qujian;
    else
        Sortedqujian = qujian;
    end
end

% function Limitqujian2Rnage(qujian,LIMIT)
% %��һ���䣬�������Ӽ���LIMIT��
%     
% end

function [vtheta2,vtheta3,vtheta4]=Getv50k_2_vtheta(JacoboMatrix,BeginPoint,EndPoint,k)
%����ǻ���������
    

    
    Vx = k*xn;
    Vy = k*yn;
    Vz = k*zn;
    

    
%     vtheta2 = (a23*a34 - a24*a33)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32)*Vx+(-(a13*a34 - a14*a33)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vy+((a13*a24 - a14*a23)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vz;
%     vtheta3 = (-(a22*a34 - a24*a32)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vx+((a12*a34 - a14*a32)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vy+(-(a12*a24 - a14*a22)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vz;
%     vtheta4 = ((a22*a33 - a23*a32)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vx+(-(a12*a33 - a13*a32)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vy+((a12*a23 - a13*a22)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32))*Vz;

                                                                                                         
    vtheta2 = (a23*a34*k*xn - a24*a33*k*xn - a13*a34*k*yn + a14*a33*k*yn + a13*a24*k*zn - a14*a23*k*zn)/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32);
    vtheta3 = -(k*(a22*a34*xn - a24*a32*xn - a12*a34*yn + a14*a32*yn + a12*a24*zn - a14*a22*zn))/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32);
    vtheta4 = (k*(a22*a33*xn - a23*a32*xn - a12*a33*yn + a13*a32*yn + a12*a23*zn - a13*a22*zn))/(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32);
    
    
    
%     vtheta3up = (JacoboMatrix(2,2)-JacoboMatrix(2,4)/JacoboMatrix(3,4)*JacoboMatrix(3,2))*(k*xn-JacoboMatrix(1,4)/JacoboMatrix(3,4)*k*zn) - (JacoboMatrix(1,2)-JacoboMatrix(1,4)/JacoboMatrix(3,4)*JacoboMatrix(3,2))*(k*yn-JacoboMatrix(2,4)/JacoboMatrix(3,4)*k*zn);
%     vtheta3down = (JacoboMatrix(1,2)-JacoboMatrix(1,4)/JacoboMatrix(3,4)*JacoboMatrix(3,2))*(JacoboMatrix(2,4)/JacoboMatrix(3,4)*JacoboMatrix(3,3)-JacoboMatrix(2,3))-(JacoboMatrix(1,4)/JacoboMatrix(3,4)*JacoboMatrix(3,3)-JacoboMatrix(1,3))*(JacoboMatrix(2,2)-JacoboMatrix(2,4)/JacoboMatrix(3,4)*JacoboMatrix(3,2));
%     vtheta3 = vtheta3up/vtheta3down;
    
%     vtheta2 = ((JacoboMatrix(2,4)/JacoboMatrix(3,4)*JacoboMatrix(3,3)-JacoboMatrix(2,3))*vthete3 + k*yn-JacoboMatrix(2,4)/JacoboMatrix(3,4)*k*zn)/(JacoboMatrix(2,2)-JacoboMatrix(2,4)/JacoboMatrix(3,4)*JacoboMatrix(3,2));
%     vtheta4 = (k*zn-JacoboMatrix(3,2)*vtheta2-JacoboMatrix(3,3)*vtheta3)/JacoboMatrix(3,4);
end

function [vtheta2Slope,vtheta3Slope,vtheta4Slope]=Getv50_vthetaSlope(JacoboMatrix,BeginPoint,EndPoint)
    [vtheta2_0,vtheta3_0,vtheta4_0]=Getv50k_2_vtheta(JacoboMatrix,BeginPoint,EndPoint,0);
    if vtheta2_0~=0||vtheta3_0~=0||vtheta4_0~=0
        error('�ϱߺ�����ķ����ִ����');
    end
    [vtheta2_1,vtheta3_1,vtheta4_1]=Getv50k_2_vtheta(JacoboMatrix,BeginPoint,EndPoint,1);
    vtheta2Slope = vtheta2_1-vtheta2_0;
    vtheta3Slope = vtheta3_1-vtheta3_0;
    vtheta4Slope = vtheta4_1-vtheta4_0;
end

function k_Range=GetRangeOfv50_k(JacoboMatrix,BeginPoint,EndPoint,Vtheta2Max,Vtheta3Max,Vtheta4Max)
%����ĽǶ��Ƕ�
%����ñ�֤����ĩ�˵��ٶȷ���ʼ��ָ��Ŀ��λ��
%Ҳ����˵ÿһʱ�̣�Ҫ������ؽڵ��ٶ�Լ�������뱣֤k�������Χ��
    Vtheta2Max = DegToRad(Vtheta2Max);
    Vtheta3Max = DegToRad(Vtheta3Max);
    Vtheta4Max = DegToRad(Vtheta4Max);
    
    [vtheta2Slope,vtheta3Slope,vtheta4Slope]=Getv50_vthetaSlope(JacoboMatrix,BeginPoint,EndPoint);
    Vtheta2_kMax = Vtheta2Max/vtheta2Slope;
    Vtheta3_kMax = Vtheta3Max/vtheta3Slope;
    Vtheta4_kMax = Vtheta4Max/vtheta4Slope;
    Vtheta2_kRange = [0,Vtheta2_kMax];
    Vtheta3_kRange = [0,Vtheta3_kMax];
    Vtheta4_kRange = [0,Vtheta4_kMax];
    k_Range = GetIntersection(Vtheta2_kRange,Vtheta3_kRange);
    k_Range = GetIntersection(k_Range,Vtheta4_kRange);
    if isempty(k_Range)==1
        error('ֱ���н�ʱ�ٶ�Լ�����ܱ����㣡');
    end
end

function BucketTipLinearPlanning(BeginPoint,EndPoint,Begin_Bucket_WithGround,End_Bucket_WithGround,Vtheta2Max,Vtheta3Max,Vtheta4Max,atheta2max,atheta3max,atheta4max)
%������ֱ�߹滮
    GlobalDeclarationCommon
    CurrentPoint = BeginPoint;
    jointAngle = InverseKinematicsBucketTip(CurrentPoint',Begin_Bucket_WithGround);
    figure
    PlotTheta1234(jointAngle(1),jointAngle(2),jointAngle(3),jointAngle(4));
    hold on;
    plot3(BeginPoint(1),BeginPoint(2),BeginPoint(3),'ro');
    hold on ;
    plot3(EndPoint(1),EndPoint(2),EndPoint(3),'ko');
    hold on;
    
    if End_Bucket_WithGround>Begin_Bucket_WithGround
        omigayDirection = 1;
    else
        if End_Bucket_WithGround<Begin_Bucket_WithGround
            omigayDirection = -1;
        else
            omigayDirection = 0;
        end
    end
    Lastchazhi = abs(Begin_Bucket_WithGround-End_Bucket_WithGround);

    
    [DesiredTheta4Begin,YES] = groundAngleRangeTOtheta4Range(jointAngle(1),jointAngle(2),jointAngle(3),[Begin_Bucket_WithGround Begin_Bucket_WithGround]);
    [DesiredTheta4End,YES] = groundAngleRangeTOtheta4Range(jointAngle(1),jointAngle(2),jointAngle(3),[End_Bucket_WithGround End_Bucket_WithGround]);
    
    lastvtheta2 = 0;
    lastvtheta3 = 0;
    lastvtheta4 = 0;
    figure
    i=1;
    
    SumerrorDistance = 0;
    LasterrorDistance = 1;
    kpdis = 0.99;
    kidis = 0.092;
%     kidis = 0;
    kddis = 0.018;
    
    SumerrorOmiga = 0;
    LasterrorOmiga = 1;
    kpOmiga = 1.95;
    kiOmiga = 0.0005;
    kdOmiga = 0.868;
    
    DecelerationFLAG = 0;
    DecelerationANGLEFLAG = 0;
    
    kthistime = 2;
    omigay = 0;
%     while kthistime>1 || abs(omigay)>0.005 
    while kthistime>1 || abs(omigay)>0.05 || norm(CurrentPoint-EndPoint)>1 || norm(jointAngle(4)-DesiredTheta4End{1})>1
%     while norm(CurrentPoint-EndPoint)>1 
%     while 1 
        JacoboMatrix = GetvOmiga50_JacoboMatrix(jointAngle(1),jointAngle(2),jointAngle(3),jointAngle(4));
        
        errorDistance = norm(CurrentPoint-EndPoint)/norm(BeginPoint-EndPoint);
        SumerrorDistance = SumerrorDistance + errorDistance*tinterval;
        DerivaDistance = abs(LasterrorDistance-errorDistance)/tinterval;
        LasterrorDistance = errorDistance;
        k_power = kpdis * errorDistance + kidis * SumerrorDistance - kddis * DerivaDistance;
        k_power = Limit2range(k_power,[0.00001,0.6]);
%         k_power = 0.5;
%         if norm(CurrentPoint-EndPoint)<(norm(BeginPoint-EndPoint)/3)*kthistime*tinterval && DecelerationFLAG==0
        if norm(CurrentPoint-EndPoint)<kthistime*tinterval && DecelerationFLAG==0
            DecelerationFLAG = 1;
        end
        if DecelerationFLAG == 1
            k_power = 0.0001;
        end

        
        AnglewithGround = GetAngleOfBucketWithGround(jointAngle(1),jointAngle(2),jointAngle(3),jointAngle(4));
        errorOmiga = norm(AnglewithGround-End_Bucket_WithGround)/norm(Begin_Bucket_WithGround-End_Bucket_WithGround);
        SumerrorOmiga = SumerrorOmiga + errorOmiga*tinterval;
        DerivaOmiga = abs(LasterrorOmiga-errorOmiga)/tinterval;
        LasterrorOmiga = errorOmiga;
        omiga_power = kpOmiga * errorOmiga + kiOmiga * SumerrorOmiga - kdOmiga * DerivaOmiga;
        omiga_power = Limit2range(omiga_power,[0.00001,0.5]);
%         omiga_power= 0.2;
%         omigayDirection = 0;
        if norm(AnglewithGround-End_Bucket_WithGround)<10*abs(omigay)*tinterval && DecelerationANGLEFLAG==0
            DecelerationANGLEFLAG =1;
        end
        if DecelerationANGLEFLAG == 1
            omiga_power = 0.00001;
%             omigayDirection = 0;
        end
        
        Nowchazhi = abs(AnglewithGround-End_Bucket_WithGround);
        if Nowchazhi>Lastchazhi
            omigayDirection = -omigayDirection;
        end
        Lastchazhi = Nowchazhi;
%         k_power = 0.999;
%         omigayDirection = 0;
%         omiga_power = 0.999;
        [vtheta2,vtheta3,vtheta4,kthistime,omigay] = GetCurrentvthetaBucketTipvtheta234komiga(omigayDirection,k_power,omiga_power,CurrentPoint,JacoboMatrix,BeginPoint,EndPoint,lastvtheta2,lastvtheta3,lastvtheta4,Vtheta2Max,Vtheta3Max,Vtheta4Max,atheta2max,atheta3max,atheta4max);
        lastvtheta2 = vtheta2;
        lastvtheta3 = vtheta3;
        lastvtheta4 = vtheta4;
        
        if (mod(i,22222222222)==0)
            subplot(171)
            plot(i,vtheta2,'.');
            title('vtheta2');
            hold on 
            subplot(172)
            plot(i,vtheta3,'.');
            title('vtheta3');
            hold on 
            subplot(173)
            plot(i,vtheta4,'.');
            title('vtheta4');
            hold on 
            subplot(174)
            plot(i,kthistime,'.');
            title('k');
            hold on 
            subplot(175)
            plot(i,omigay,'.');
            title('omigay');
            hold on 
            subplot(176)
            plot(i,norm(CurrentPoint-EndPoint),'.');
            title('Distance');
            hold on 
            subplot(177)
            plot(i,norm(AnglewithGround-End_Bucket_WithGround),'.');
            title('angle4');
            hold on 
            pause(0.1);
        end
        i=i+1;
%         if i==245
%             disp('');
%         end

        %         k_Range=GetRangeOfv50_k(JacoboMatrix,BeginPoint,EndPoint,Vtheta2Max,Vtheta3Max,Vtheta4Max);
%         [vtheta2,vtheta3,vtheta4]= Getv50k_2_vtheta(JacoboMatrix,BeginPoint,EndPoint,k_Range(2));

        jointAngle(2) = jointAngle(2) + vtheta2*tinterval;
        jointAngle(3) = jointAngle(3) + vtheta3*tinterval;
        jointAngle(4) = jointAngle(4) + vtheta4*tinterval;
        
        if IsAnglesInLimitRange(jointAngle)==0
            error('��ƵĹ滮�㷨ʹ�ýǶȳ�������������')
        end

        
        [~,pos2] = ForwardKinematics(jointAngle);
        CurrentPoint = pos2(1:3,4)';
        [Begin_Bucket_WithGround AnglewithGround End_Bucket_WithGround]
        norm(CurrentPoint-EndPoint)
%         CurrentPoint-BeginPoint
        if (mod(i,12222222220)==0)
            
            PlotTheta1234(jointAngle(1),jointAngle(2),jointAngle(3),jointAngle(4));
            
%             hold on 
%             plot3(CurrentPoint(1),CurrentPoint(2),CurrentPoint(3),'.');
%             hold on
            pause(0.1);
%             clf
        end

    end
    
end

function Sequence = BucketTipLinearPlanningROBOTICSTOOL(BeginPoint,EndPoint,Begin_Bucket_WithGround,End_Bucket_WithGround,Vtheta2Max,Vtheta3Max,Vtheta4Max,atheta2max,atheta3max,atheta4max)
    GlobalDeclarationCommon
    result1 = InverseKinematicsBucketTip(BeginPoint',Begin_Bucket_WithGround) ;
    result2 = InverseKinematicsBucketTip(EndPoint',End_Bucket_WithGround) ;
    [~,Matrixbegin] = ForwardKinematics(result1);
    [~,Matrixend] = ForwardKinematics(result2);

    Tsequence = ctraj(Matrixbegin,Matrixend,200);
    posStore = [];
    jointangleSeq = [];
    for i=1:size(Tsequence,3)
        tform = Tsequence(:,:,i);
        jointangle = InverseKinematics(tform);
        if IsAnglesInLimitRange(jointangle) == 0
            norm(BeginPoint-EndPoint)
            error('��ƵĹ滮�㷨ʹ�ýǶȳ������������� ���߽Ƕ�̫����')
%             disp('');
        end
        posStore = [posStore;tform(1:3,4)'];
        jointangleSeq(i,:) = jointangle;
    end
    
    
    djointangleSeq = [0 2*Vtheta2Max 2*Vtheta3Max 2*Vtheta4Max];
    ddjointangleSeq = [0 2*atheta2max 2*atheta3max 2*atheta4max];
    Leftbeishu = 0;
    Rightbeishu = 4000; %����ζ��40�������8cm һ��2mm ���������϶��Ǿ�ֹ����
    timesBEISHU = (Leftbeishu+Rightbeishu)/2;
    FLAGWHILE = 0;
    while FLAGWHILE<2
        djointangleSeq = [];
        for i=1:size(jointangleSeq,1)-1
            djointangleSeq(i,:) = (jointangleSeq(i+1,:)-jointangleSeq(i,:))/(timesBEISHU*tinterval);
        end
        ddjointangleSeq=[];
        for i=1:size(djointangleSeq,1)-1
            ddjointangleSeq(i,:) = (djointangleSeq(i+1,:)-djointangleSeq(i,:))/(timesBEISHU*tinterval);
        end
        dposStore = [];
        for i=1:size(posStore,1)-1
            dposStore(i,:) = norm(posStore(i+1,:)-posStore(i,:))/(timesBEISHU*tinterval);
        end
        ddposStore = [];
        for i=1:size(dposStore,1)-1
            ddposStore(i,:) = (dposStore(i+1,:)-dposStore(i,:))/(timesBEISHU*tinterval);
        end
        
        if Rightbeishu-Leftbeishu<=1 %�������ַ����ȵ�0.1 ���õ�timesBEISHU���ٽ�ȡֵ
            FLAGWHILE = FLAGWHILE+1;
            timesBEISHU = Rightbeishu;
            timesBEISHU = ceil(timesBEISHU);
            if max(abs(djointangleSeq(:,2)))>Vtheta2Max==1 || max(abs(djointangleSeq(:,3)))>Vtheta3Max==1 || max(abs(djointangleSeq(:,4)))>Vtheta4Max==1 ...
                || max(abs(ddjointangleSeq(:,2)))>atheta2max==1||max(abs(ddjointangleSeq(:,3)))>atheta3max==1||max(abs(ddjointangleSeq(:,4)))>atheta4max==1
                if FLAGWHILE==2
                    error('���õľ��ȳ���')
                end
            end
        else
            if max(abs(djointangleSeq(:,2)))>Vtheta2Max==1 || max(abs(djointangleSeq(:,3)))>Vtheta3Max==1 || max(abs(djointangleSeq(:,4)))>Vtheta4Max==1 ...
                || max(abs(ddjointangleSeq(:,2)))>atheta2max==1||max(abs(ddjointangleSeq(:,3)))>atheta3max==1||max(abs(ddjointangleSeq(:,4)))>atheta4max==1
                Leftbeishu = timesBEISHU;
            else
                Rightbeishu = timesBEISHU;
            end
            timesBEISHU = Leftbeishu + (Rightbeishu-Leftbeishu)/2;
        end
    end
    
    Sequence = []; %���շ��ش�ʱ����ĸ��Ƕ�
    for i=1:size(jointangleSeq,1)-1
        timethis = (i-1)*(timesBEISHU*tinterval);
        Sequence = [Sequence,[timethis;jointangleSeq(i,:)']];
        tmp = jointangleSeq(i+1,:)-jointangleSeq(i,:);
        tmp = tmp/timesBEISHU;
        for j=1:timesBEISHU-1
            Sequence = [Sequence,[timethis+j*tinterval;Sequence(2:5,end)+tmp']];
        end
    end
    timethis = (size(jointangleSeq,1)-1)*(timesBEISHU*tinterval);
    Sequence = [Sequence,[timethis;jointangleSeq(size(jointangleSeq,1),:)']];
    
    
    
    degq = jointangleSeq;
    degdq = djointangleSeq;
    degddq = ddjointangleSeq;
    figure
    subplot(531)
    plot(1:size(degdq,1),degdq(:,2),'-');
    title('vtheta2')
    subplot(532)
    plot(1:size(degdq,1),degdq(:,3),'-');
    title('vtheta3')
    subplot(533)
    plot(1:size(degdq,1),degdq(:,4),'-');
    title('vtheta4')

    subplot(534)
    plot(1:size(degddq,1),degddq(:,2),'-');
    title('atheta2')
    subplot(535)
    plot(1:size(degddq,1),degddq(:,3),'-');
    title('atheta3')
    subplot(536)
    plot(1:size(degddq,1),degddq(:,4),'-');
    title('atheta4')

    subplot(537)
    plot(1:size(degq,1),degq(:,2),'-');
    title('theta2')
    subplot(538)
    plot(1:size(degq,1),degq(:,3),'-');
    title('theta3')
    subplot(539)
    plot(1:size(degq,1),degq(:,4),'-');
    title('theta4')

    subplot(5,3,10)
    plot(1:size(dposStore,1),dposStore,'-');
    title('v')

    subplot(5,3,11)
    plot(1:size(ddposStore,1),ddposStore,'-');
    title('a')

    subplot(5,3,12)
    plot(posStore(:,1),posStore(:,2),'-');
    title('xy')

    subplot(5,3,13)
    plot(posStore(:,1),posStore(:,3),'-');
    title('xz')

    subplot(5,3,14)
    plot(posStore(:,2),posStore(:,3),'-');
    title('yz')
end
%%


%%

function deg = RadToDeg(rad)
    deg = rad*180/pi;
end

function rad = DegToRad(deg)
    rad = deg*pi/180;
end



function qiedian = GetQiedianOfCircle(ptCenter,ptOutside,dbRadious)
%ֻ�����ڶ�ά���
%  double r=dbRadious;
    r=dbRadious;
%  //1. ����ƽ�Ƶ�Բ��ptCenter��,��԰����������E
    E(1)= ptOutside(1)-ptCenter(1);
    E(2)= ptOutside(2)-ptCenter(2); %//ƽ�Ʊ任��E
 
%  //2. ��԰��OE�Ľ�������F, �൱��E�����ű任
    t= r / sqrt (E(1) * E(1) + E(2) * E(2)); % //�õ����ű���
    F(1)= E(1) * t;   F(2)= E(2) * t;  % //���ű任��F
 
%  //3. ��E��ת�任�Ƕ�a���е�G������cos(a)=r/OF=t, ����a=arccos(t);
    a=acos(t);  % //�õ���ת�Ƕ�
    G(1)=F(1)*cos(a) -F(2)*sin(a);
    G(2)=F(1)*sin(a) +F(2)*cos(a);   % //��ת�任��G
 
%  //4. ��Gƽ�Ƶ�ԭ���������µõ�������H
     H(1,1)=G(1)+ptCenter(1);
     H(1,2)=G(2)+ptCenter(2);             %//ƽ�Ʊ任��H
     
    a=-acos(t);  % //�õ���ת�Ƕ�
    G(1)=F(1)*cos(a) -F(2)*sin(a);
    G(2)=F(1)*sin(a) +F(2)*cos(a);   % //��ת�任��G
 
    H(2,1)=G(1)+ptCenter(1);
    H(2,2)=G(2)+ptCenter(2);             %//ƽ�Ʊ任��H
    
%  //5. ����H
    qiedian = H;

%  //6. ʵ��Ӧ�ù����У�ֻҪһ���м����E,����F,G,H���Բ��á�
end




