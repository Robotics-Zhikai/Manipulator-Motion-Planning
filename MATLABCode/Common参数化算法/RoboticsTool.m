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
jnt3.PositionLimits = [deg2rad(theta3Range(1)) deg2rad(theta3Range(2))];
jnt3.HomePosition = deg2rad(theta3Range(1)); % User defined
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
show(robot,config);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1]; %这个是固定的，不因待求的逆运动学变量改变而改变
initialguess = robot.homeConfiguration;
[configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
show(robot,configSoln);
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