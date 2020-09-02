%%
% matlab 自带的工具箱

clc
clear
close all

GlobalDeclarationCommon

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.PositionLimits = [deg2rad(theta1Range(1)) deg2rad(theta1Range(2))];
jnt1.HomePosition = pi/4;
% [         cos(k1),        -sin(k1),        0,          a0]
% [ cos(m0)*sin(k1), cos(k1)*cos(m0), -sin(m0), -d1*sin(m0)]
% [ sin(k1)*sin(m0), cos(k1)*sin(m0),  cos(m0),  d1*cos(m0)]
% [               0,               0,        0,           1]
tform = trvec2tform([a0, -d1*sin(m0), d1*cos(m0)]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

robot = rigidBodyTree;
addBody(robot,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.PositionLimits =[deg2rad(theta2Range(1)) deg2rad(theta2Range(2))];
jnt2.HomePosition = deg2rad(theta2Range(1)); % User defined
tform2 = trvec2tform([a1,-d2*sin(m1),d2*cos(m1)])*eul2tform([0, 0, m1]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = deg2rad(theta3Range(1)); % User defined
jnt3.PositionLimits = [deg2rad(theta3Range(1)) deg2rad(theta3Range(2))];

tform3 = trvec2tform([a2,-d3*sin(m2),d3*cos(m2)]); % User defined
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.PositionLimits = [deg2rad(theta4Range(1)) deg2rad(theta4Range(2))];
jnt4.HomePosition = deg2rad(theta4Range(1)); % User defined
tform4 = trvec2tform([a3,-d4*sin(m3),d4*cos(m3)]); % User defined
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
addBody(robot,body4,'body3'); % Add body4 to body3

bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([tool, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);
addBody(robot,bodyEndEffector,'body4');

config = randomConfiguration(robot);
tform = getTransform(robot,config,'endeffector','base');
showdetails(robot)
% show(robot,config);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1]; %这个是固定的，不因待求的逆运动学变量改变而改变
initialguess = robot.homeConfiguration;

  tform =  [ 0.9320    0.3623   -0.0120  681.1443;
   -0.0112   -0.0043   -0.9999  -21.8567;
   -0.3623    0.9321    0.0000  172.4116;
         0         0         0    1.0000];
[configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
% show(robot,configSoln);


% 
% ans =
% 
%     2.5468
% 
% 
% ans =
% 
%     0.8809
% 
% 
% ans =
% 
%    -2.1178
% 
% 
% ans =
% 
%     0.4141
%     
%       -0.5633   -0.6072    0.5604 -371.5687
%     0.3811    0.4108    0.8282  267.9321
%    -0.7331    0.6802    0.0000  122.9043
%          0         0         0    1.0000
%第四列有小数点后第二位的差距 前三列有个别有小数点后一位的差距





%%
% %PeterCorke的工具箱
% clc
% clear
% close all
% GlobalDeclarationCommon
% %             theta   d         a        alpha     offset
ML1 =  Link([ pi/4,      d1,   a0,       m0,        0], 'modified');
ML2 =  Link([ deg2rad(theta2Range(1)),  d2, a1, m1,   0], 'modified');
ML3 =  Link([ deg2rad(theta3Range(1)),  d3, a2, m2,       0], 'modified');
ML4 =  Link([ deg2rad(theta4Range(1)),  d4, a3, m3,   0], 'modified');
ML5 =  Link([ 0,      0,        tool,       0 ,    0], 'modified');

ML1.qlim = [deg2rad(theta1Range(1)) deg2rad(theta1Range(2))];
ML2.qlim = [deg2rad(theta2Range(1)) deg2rad(theta2Range(2))];
ML3.qlim = [deg2rad(theta3Range(1)) deg2rad(theta3Range(2))];
ML4.qlim = [deg2rad(theta4Range(1)) deg2rad(theta4Range(2))];
ML5.qlim = [0 0];

robotPeterCorke=SerialLink([ML1 ML2 ML3 ML4 ML5],'name','Excavator');
robotPeterCorke2 = SerialLink([ML1 ML2 ML3 ML4],'name','Excavator');
robotPeterCorke3 = SerialLink([ML1 ML2 ML3 ],'name','Excavator');
robotPeterCorke4 = SerialLink([ML1 ML2],'name','Excavator');
robotPeterCorke5 = SerialLink([ML1 ],'name','Excavator');
% robotPeterCorke.teach();
% robotPeterCorke2.teach();
% robotPeterCorke3.teach();
% robotPeterCorke4.teach();
% robotPeterCorke5.teach();
% robot.teach();
% robot.plot([0,0,0,deg2rad(theta4Range(2)),0]);
% modmyt06 = robot.fkine([2.5468 0.8809 -2.1178 0.4141 0]);%不在意各关节的限制范围
%    -0.5634   -0.6072    0.5603    -371.6
%     0.3811    0.4108    0.8283     267.9
%    -0.7331    0.6802         0     122.9
%          0         0         0         1




PointA = [681.1443  -21.8567  172.4116];
PointB = [ 546.0533  -20.2392  -29.5721];
BeginAngelBucketWithGround = -21.2425;
EndAngelBucketWithGround =  -111.4202;

[PointA,BeginAngelBucketWithGround,PointB,EndAngelBucketWithGround,angleA,angleB] = RandGenratePointDirectLineBucketTip([200 250]);
[Theta4Range,YES] = groundAngleRangeTOtheta4Range(angleA(1),angleA(2),angleA(3),[BeginAngelBucketWithGround,BeginAngelBucketWithGround]);
[Theta4Range1,YES1] = groundAngleRangeTOtheta4Range(angleB(1),angleB(2),angleB(3),[EndAngelBucketWithGround,EndAngelBucketWithGround]);
if abs(Theta4Range{1}-angleA(4))>0.001 || abs(Theta4Range1{1}-angleB(4))>0.001
    error('上边两个函数的程序逻辑出了问题');
end


result1 = InverseKinematicsBucketTip(PointA',BeginAngelBucketWithGround) ;
result2 = InverseKinematicsBucketTip(PointB',EndAngelBucketWithGround) ;
[~,Matrixbegin] = ForwardKinematics(result1);
[~,Matrixend] = ForwardKinematics(result2);

T = ctraj(Matrixbegin,Matrixend,50);
posStore = [];
for i=1:size(T,3)
    i
    tform = T(:,:,i);
    
%     rad2deg(rotm2eul(tform(1:3,1:3)))
    
    
%     [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
%     angle = ([configSoln.JointPosition]); %弧度数
    configSoln = InverseKinematics(tform);
    angle = deg2rad(configSoln);
    
%     if norm(InverseKinematics(T(:,:,i))-rad2deg(angle))>0.001
%         InverseKinematics(T(:,:,i))
%         rad2deg(angle)
%     end
%     GetAngleOfBucketWithGround(rad2deg(angle(1)),rad2deg(angle(2)),rad2deg(angle(3)),rad2deg(angle(4)))
    
    [~,tmpmatrix] = ForwardKinematics(rad2deg(angle));
    posStore = [posStore;tmpmatrix(1:3,4)'];
    q(i,:) = [angle 0];
    if IsAnglesInLimitRange(rad2deg(angle)) == 0
        error('设计的规划算法使得角度超出了物理限制')
    end
end
robotPeterCorke.plot(q);
robotPeterCorke.teach()




q;
timesBEISHU = 8;
dq = [];
for i=1:size(q,1)-1
    dq(i,:) = (q(i+1,:)-q(i,:))/(timesBEISHU*tinterval);
end
ddq=[];
for i=1:size(dq,1)-1
    ddq(i,:) = (dq(i+1,:)-dq(i,:))/(timesBEISHU*tinterval);
end
dposStore = [];
for i=1:size(posStore,1)-1
    dposStore(i,:) = norm(posStore(i+1,:)-posStore(i,:))/(timesBEISHU*tinterval);
end
ddposStore = [];
for i=1:size(dposStore,1)-1
    ddposStore(i,:) = (dposStore(i+1,:)-dposStore(i,:))/(timesBEISHU*tinterval);
end


degq = rad2deg(q);
degdq = rad2deg(dq);
degddq = rad2deg(ddq);
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

