%% kinematics
clear
clc
close all
% syms m0 m1 m2 m3 a0 a1 a2 a3  d1 d2 d3 d4 tool 
syms k1 k2 k3 k4

%% init
m0=0*pi/180;
m1=90*pi/180;
m2=0*pi/180;
m3=0*pi/180;
%%
a0=0;
d3=0;
d4=0;
%%
a1=12;
a2=460;
a3=210.9;
d1=57.9;
d2=13.7;
tool=123.5;
%% init
% k1=0*pi/180;
% k2=0*pi/180;
% k3=0*pi/180;
% k4=10*pi/180;
%% calculate
A1=[cos(k1) -sin(k1) 0 a0;sin(k1)*cos(m0) cos(k1)*cos(m0) -sin(m0) -sin(m0)*d1;sin(k1)*sin(m0) cos(k1)*sin(m0) cos(m0) cos(m0)*d1;0 0 0 1];
R10 = A1(1:3,1:3);
P10 = A1(1:3,4);
A2=[cos(k2) -sin(k2) 0 a1;sin(k2)*cos(m1) cos(k2)*cos(m1) -sin(m1) -sin(m1)*d2;sin(k2)*sin(m1) cos(k2)*sin(m1) cos(m1) cos(m1)*d2;0 0 0 1];
R21 = A2(1:3,1:3);
P21 = A2(1:3,4);
A3=[cos(k3) -sin(k3) 0 a2;sin(k3)*cos(m2) cos(k3)*cos(m2) -sin(m2) -sin(m2)*d3;sin(k3)*sin(m2) cos(k3)*sin(m2) cos(m2) cos(m2)*d3;0 0 0 1];
R32 = A3(1:3,1:3);
P32 = A3(1:3,4);
A4=[cos(k4) -sin(k4) 0 a3;sin(k4)*cos(m3) cos(k4)*cos(m3) -sin(m3) -sin(m3)*d4;sin(k4)*sin(m3) cos(k4)*sin(m3) cos(m3) cos(m3)*d4;0 0 0 1];
R43 = A4(1:3,1:3);
P43 = A4(1:3,4);
A5=[1 0 0 tool;0 1 0 0;0 0 1 0;0 0 0 1];

T20 = simplify(A1*A2)
T30 = simplify(A1*A2*A3)
T40 = simplify(A1*A2*A3*A4)
T50 = simplify(A1*A2*A3*A4*A5)

T50(1:3,4)-T40(1:3,4) %这个向量的值就是铲斗刚体的方向向量 使得z等于0即可使铲斗始终平行于地面
T40(1,4)
T40(2,4)
T40(3,4)
T40(4,4)
% T1=A1*A2*A3*A4
% T2=A2*A3*A4
% T3=A3*A4

InvA1=[cos(k1) sin(k1) 0 0;-sin(k1) cos(k1) 0 0;0 0 1 -d1;0 0 0 1];
R01 = InvA1(1:3,1:3);
InvA2=[cos(k2) 0 sin(k2) -a1*cos(k2);-sin(k2) 0 cos(k2) a1*sin(k2);0 -1 0 -d2;0 0 0 1];
R12 = InvA2(1:3,1:3);
InvA3=[cos(k3) sin(k3) 0 -a2*cos(k3);-sin(k3) cos(k3) 0 a2*sin(k3);0 0 1 0;0 0 0 1];
R23 = InvA3(1:3,1:3);
InvA4=[cos(k4) sin(k4) 0 -a3*cos(k4);-sin(k4) cos(k4) 0 a3*sin(k4);0 0 1 0;0 0 0 1];
R34 = InvA4(1:3,1:3);
InvA3*InvA2*InvA1



syms d_k1 d_k2 d_k3 d_k4
Omiga11 = [0;0;d_k1]
% Omiga11 = [0;0;0] %假设不转 转时用上边一行
v11 = [0;0;0]
Omiga22 = R12*Omiga11 + [0;0;d_k2]
v22 = R12*(v11 + cross(Omiga11,P21))
Omiga33 = R23*Omiga22 + [0;0;d_k3]
v33 = R23*(v22 + cross(Omiga22,P32))
Omiga44 = R34*Omiga33 + [0;0;d_k4]
v44 = R34*(v33 + cross(Omiga33,P43))
R40 = R10*R21*R32*R43
R41 = R21*R32*R43
R41simp = simplify(R41)
R40simp = simplify(R40)
R40simp11 = R40simp(1,1)
R40simp12 = R40simp(1,2)
R40simp13 = R40simp(1,3)
R40simp21 = R40simp(2,1)
R40simp22 = R40simp(2,2)
R40simp23 = R40simp(2,3)
R40simp31 = R40simp(3,1)
R40simp32 = R40simp(3,2)
R40simp33 = R40simp(3,3)

v21 = R21*v22
omaga21 = R21*Omiga22
v41 = R41*v44
v40 = R40*v44
v40 = simplify(v40)
% v401simpNotconj = 230*d_k2*sin(k1 - k2) - (2109*d_k3*sin(k2 - k1 + k3))/20 - 230*d_k2*sin(k1 + k2) - (2109*d_k2*sin(k2 - k1 + k3))/20 - (7420384073534669*d_k2*sin(k1 + k2 + k3))/70368744177664 - (7420384073534669*d_k3*sin(k1 + k2 + k3))/70368744177664;
% v401simpNotconj = collect(v401simpNotconj,[d_k2,d_k3])
% v402simpNotconj = 230*d_k2*cos(k1 + k2) - (2109*d_k3*cos(k2 - k1 + k3))/20 - (2109*d_k2*cos(k2 - k1 + k3))/20 - 230*d_k2*cos(k1 - k2) + (7420384073534669*d_k2*cos(k1 + k2 + k3))/70368744177664 + (7420384073534669*d_k3*cos(k1 + k2 + k3))/70368744177664;
% v402simpNotconj = collect(v402simpNotconj,[d_k2,d_k3])
% v403simpNotconj = (2109*d_k2*cos(k2 + k3))/10 + (2109*d_k3*cos(k2 + k3))/10 + 460*d_k2*cos(k2);
% v403simpNotconj = collect(v403simpNotconj,[d_k2,d_k3])
% v40 = [v401simpNotconj;v402simpNotconj;v403simpNotconj]

% v401 = (230*sin(k1 - k2) - (7420384073534669*sin(k1 + k2 + k3))/70368744177664 - (2109*sin(k2 - k1 + k3))/20 - 230*sin(k1 + k2))*d_k2 + (- (7420384073534669*sin(k1 + k2 + k3))/70368744177664 - (2109*sin(k2 - k1 + k3))/20)*d_k3;
% v402 = ((7420384073534669*cos(k1 + k2 + k3))/70368744177664 - (2109*cos(k2 - k1 + k3))/20 + 230*cos(k1 + k2) - 230*cos(k1 - k2))*d_k2 + ((7420384073534669*cos(k1 + k2 + k3))/70368744177664 - (2109*cos(k2 - k1 + k3))/20)*d_k3;
% v403 = ((2109*cos(k2 + k3))/10 + 460*cos(k2))*d_k2 + ((2109*cos(k2 + k3))/10)*d_k3;
% Jacobothis(1,:) = [0 (230*sin(k1 - k2) - (7420384073534669*sin(k1 + k2 + k3))/70368744177664 - (2109*sin(k2 - k1 + k3))/20 - 230*sin(k1 + k2)) (- (7420384073534669*sin(k1 + k2 + k3))/70368744177664 - (2109*sin(k2 - k1 + k3))/20)];
% Jacobothis(2,:) = [0 ((7420384073534669*cos(k1 + k2 + k3))/70368744177664 - (2109*cos(k2 - k1 + k3))/20 + 230*cos(k1 + k2) - 230*cos(k1 - k2)) ((7420384073534669*cos(k1 + k2 + k3))/70368744177664 - (2109*cos(k2 - k1 + k3))/20)];
% Jacobothis(3,:) = [0 ((2109*cos(k2 + k3))/10 + 460*cos(k2)) ((2109*cos(k2 + k3))/10)];
% InvJacobothis = inv(Jacobothis)

% v411simpNotconj = (137*d_k1)/10 - (2109*d_k2*sin(k2 + k3))/10 - (2109*d_k3*sin(k2 + k3))/10 - 460*d_k2*sin(k2);
v411simpNotconj = - (2109*d_k2*sin(k2 + k3))/10 - (2109*d_k3*sin(k2 + k3))/10 - 460*d_k2*sin(k2);
v412simpNotconj = 0;




v401simpNotconj = (cos(k1 + k2 + k3 + k4)/2 + cos(k2 - k1 + k3 + k4)/2)*(460*d_k2*sin(k3 + k4) + (2109*d_k2*sin(k4))/10 + (2109*d_k3*sin(k4))/10 + (137*d_k1*cos(k2 + k3 + k4))/10) - (sin(k1 + k2 + k3 + k4)/2 + sin(k2 - k1 + k3 + k4)/2)*(460*d_k2*cos(k3 + k4) + (2109*d_k2*cos(k4))/10 + (2109*d_k3*cos(k4))/10 - (137*d_k1*sin(k2 + k3 + k4))/10) - (d_k1*sin(k1)*(2109*cos(k2 + k3) + 4600*cos(k2) + 120))/10
v402simpNotconj = (cos(k1 + k2 + k3 + k4)/2 - cos(k2 - k1 + k3 + k4)/2)*(460*d_k2*cos(k3 + k4) + (2109*d_k2*cos(k4))/10 + (2109*d_k3*cos(k4))/10 - (137*d_k1*sin(k2 + k3 + k4))/10) + (sin(k1 + k2 + k3 + k4)/2 - sin(k2 - k1 + k3 + k4)/2)*(460*d_k2*sin(k3 + k4) + (2109*d_k2*sin(k4))/10 + (2109*d_k3*sin(k4))/10 + (137*d_k1*cos(k2 + k3 + k4))/10) + (d_k1*cos(k1)*(2109*cos(k2 + k3) + 4600*cos(k2) + 120))/10
v403simpNotconj = (2109*d_k2*cos(k2 + k3))/10 - (5115723036350277*d_k1*cos(k2 + k3))/396140812571321687967719751680 - (3725818200016133*d_k1)/5070602400912917605986812821504 + (2109*d_k3*cos(k2 + k3))/10 - (8926439437538653*d_k1*cos(k2))/316912650057057350374175801344 + 460*d_k2*cos(k2)

% 
% Xishu1 = (cos(k1 + k2 + k3 + k4)/2 + cos(k2 - k1 + k3 + k4)/2);
% Xishu2 = (sin(k1 + k2 + k3 + k4)/2 + sin(k2 - k1 + k3 + k4)/2);
% test = [Xishu1*( (137*cos(k2 + k3 + k4))/10 )+Xishu2*((137*sin(k2 + k3 + k4))/10)-(sin(k1)*(2109*cos(k2 + k3) + 4600*cos(k2) + 120))/10 Xishu1*460*sin(k3 + k4)+Xishu1*(2109*sin(k4))/10-Xishu2*(460*cos(k3 + k4) + (2109*cos(k4))/10) Xishu1*(2109*sin(k4))/10-Xishu2*((2109*cos(k4))/10)]
Jacobo = [];
hebin = collect(v401simpNotconj,[d_k1,d_k2,d_k3]);
Jacobo = [Jacobo;[((137*cos(k2 + k3 + k4)*(cos(k1 + k2 + k3 + k4)/2 + cos(k2 - k1 + k3 + k4)/2))/10 - (sin(k1)*(2109*cos(k2 + k3) + 4600*cos(k2) + 120))/10 + (137*sin(k2 + k3 + k4)*(sin(k1 + k2 + k3 + k4)/2 + sin(k2 - k1 + k3 + k4)/2))/10) ((cos(k1 + k2 + k3 + k4)/2 + cos(k2 - k1 + k3 + k4)/2)*(460*sin(k3 + k4) + (2109*sin(k4))/10) - (sin(k1 + k2 + k3 + k4)/2 + sin(k2 - k1 + k3 + k4)/2)*(460*cos(k3 + k4) + (2109*cos(k4))/10)) ((2109*sin(k4)*(cos(k1 + k2 + k3 + k4)/2 + cos(k2 - k1 + k3 + k4)/2))/10 - (2109*cos(k4)*(sin(k1 + k2 + k3 + k4)/2 + sin(k2 - k1 + k3 + k4)/2))/10)]];
hebin1 = collect(v402simpNotconj,[d_k1,d_k2,d_k3]);
Jacobo = [Jacobo;[((cos(k1)*(2109*cos(k2 + k3) + 4600*cos(k2) + 120))/10 - (137*sin(k2 + k3 + k4)*(cos(k1 + k2 + k3 + k4)/2 - cos(k2 - k1 + k3 + k4)/2))/10 + (137*cos(k2 + k3 + k4)*(sin(k1 + k2 + k3 + k4)/2 - sin(k2 - k1 + k3 + k4)/2))/10) ((cos(k1 + k2 + k3 + k4)/2 - cos(k2 - k1 + k3 + k4)/2)*(460*cos(k3 + k4) + (2109*cos(k4))/10) + (sin(k1 + k2 + k3 + k4)/2 - sin(k2 - k1 + k3 + k4)/2)*(460*sin(k3 + k4) + (2109*sin(k4))/10)) ((2109*cos(k4)*(cos(k1 + k2 + k3 + k4)/2 - cos(k2 - k1 + k3 + k4)/2))/10 + (2109*sin(k4)*(sin(k1 + k2 + k3 + k4)/2 - sin(k2 - k1 + k3 + k4)/2))/10)]];
hebin2 = collect(v403simpNotconj,[d_k1,d_k2,d_k3]);
Jacobo = [Jacobo;[(- (5115723036350277*cos(k2 + k3))/396140812571321687967719751680 - (8926439437538653*cos(k2))/316912650057057350374175801344 - 3725818200016133/5070602400912917605986812821504) ((2109*cos(k2 + k3))/10 + 460*cos(k2)) ((2109*cos(k2 + k3))/10)]];
InvJacobo = inv(Jacobo);
InvJacobo = simplify(InvJacobo);
Jacobo = simplify(Jacobo);
InvJacobo(2,1)
DetJacobo = simplify(det(Jacobo));

Jacobo
InvJacobo
DetJacobo
InvJacobo11 = InvJacobo(1,1)
InvJacobo12 = InvJacobo(1,2)
InvJacobo13 = InvJacobo(1,3)
InvJacobo21 = InvJacobo(2,1)
InvJacobo22 = InvJacobo(2,2)
InvJacobo23 = InvJacobo(2,3)
InvJacobo31 = InvJacobo(3,1)
InvJacobo32 = InvJacobo(3,2)
InvJacobo33 = InvJacobo(3,3)

Jacobo11 = Jacobo(1,1)
Jacobo12 = Jacobo(1,2)
Jacobo13 = Jacobo(1,3)
Jacobo21 = Jacobo(2,1)
Jacobo22 = Jacobo(2,2)
Jacobo23 = Jacobo(2,3)
Jacobo31 = Jacobo(3,1)
Jacobo32 = Jacobo(3,2)
Jacobo33 = Jacobo(3,3)

syms z1 z0 x1 x0 y1 y0
Vx = Jacobo12*d_k2 + Jacobo13*d_k3
Vy = Jacobo22*d_k2 + Jacobo23*d_k3
% Vz = Jacobo32*d_k2 + Jacobo33*d_k3
% Vz = (z1-z0)/(x1-x0)*Vx
Vz = (z1-z0)/(y1-y0)*Vy
d_k2 = Vz/Jacobo32-(Jacobo33/Jacobo32)*d_k3;
d_k2 = simplify(d_k2)



k1 = 73*pi/180;
for k2=-40*pi/180:0.1:44*pi/180
    for k3=-130*pi/180:0.2:-20*pi/180
        i=0;
        for d_k3=-50*360*pi/180:1:50*360*pi/180
            i=i+1;
            %Vx
%             plot(i,(2109*d_k3*cos(k2 + k3)*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/(100*((2109*cos(k2 + k3))/10 + 460*cos(k2))) - (2109*d_k3*sin(k2 + k3)*cos(k1))/10,'r.');
            %Vy
%             plot(i,(2109*d_k3*cos(k2 + k3)*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/(100*((2109*cos(k2 + k3))/10 + 460*cos(k2))) - (2109*d_k3*sin(k2 + k3)*sin(k1))/10,'b.');  
            %d_k2
            plot(i,-(2109*d_k3*cos(k2 + k3))/(10*((2109*cos(k2 + k3))/10 + 460*cos(k2))),'y.');
            hold on 
            
        end
        pause(0.1)
    end
end

pretty(DetJacobo)

figure
lidu = 0.05;
i = -1;
for k2 = -40*pi/180:lidu:44*pi/180 %发现是没有奇异点的
%     vpasolve((102301263*sin(k2))/5 - 1164168*sin(k3) - 44626440*cos(k2)*sin(k3) - (102301263*cos(k3)^2*sin(k2))/5 - (102301263*cos(k2)*cos(k3)*sin(k3))/5==0,k3,[-130*pi/180,-20*pi/180])
    for k3 = -130*pi/180:lidu:-20*pi/180
        i = i+1;
%         if abs((102301263*sin(k2))/5 - 1164168*sin(k3) - 44626440*cos(k2)*sin(k3) - (102301263*cos(k3)^2*sin(k2))/5 - (102301263*cos(k2)*cos(k3)*sin(k3))/5)<10^-1
%             
%             plot(i,k2,'.');
%             hold on 
%             pause(0.1);
%         end
        plot(i,(102301263*sin(k2))/5 - 1164168*sin(k3) - 44626440*cos(k2)*sin(k3) - (102301263*cos(k3)^2*sin(k2))/5 - (102301263*cos(k2)*cos(k3)*sin(k3))/5,'.');
        hold on ;
        pause(0.1);
    end
end





















