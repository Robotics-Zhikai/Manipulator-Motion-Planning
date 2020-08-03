%GlobalDeclarationCommon
% global m0 m1 m2 m3 a0 a1 a2 a3 d1 d2 d3 d4 tool 
% global sinm0 cosm0 sinm1 cosm1 sinm2 cosm2 sinm3 cosm3
% global theta1Range theta2Range theta3Range theta4Range ZeroDefine
% global TwoInnerTangentPoints
% global InnerEdgeDown InnerEdgeUp

m0=0*pi/180;
m1=90*pi/180;
m2=0*pi/180;
m3=0*pi/180;
sinm0 = sin(m0);
cosm0 = cos(m0);
sinm1 = sin(m1);
cosm1 = cos(m1);
sinm2 = sin(m2);
cosm2 = cos(m2);
sinm3 = sin(m3);
cosm3 = cos(m3);


a0=0;
d3=0;
d4=0;
a1=12;
a2=460;
a3=210.9;
d1=57.9;
d2=13.7;
tool=123.5;

theta1Range = [-179.9999 180];
theta2Range = [-40 44];
theta3Range = [-130 -20];
theta4Range = [-100 30];
% theta4Range = [-100 150];
% theta4Range = [-179 179];

% theta1Range(1) = legalizAnger(theta1Range(1));
% theta1Range(2) = legalizAnger(theta1Range(2));
% theta2Range(1) = legalizAnger(theta2Range(1));
% theta2Range(2) = legalizAnger(theta2Range(2));
% theta3Range(1) = legalizAnger(theta3Range(1));
% theta3Range(2) = legalizAnger(theta3Range(2));
% theta4Range(1) = legalizAnger(theta4Range(1));
% theta4Range(2) = legalizAnger(theta4Range(2));


ZeroDefine = 10^-6;

tinterval = 0.01; % 离散的pid控制系统，采样时间过高了的话有可能会造成发散
