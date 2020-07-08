clc
clear
close all

Vmax = 10;
Vmin = 5;
aMax = 1;
aMin = 0.5;

syms a2 a4 theta0 thetaf tf v0 vf t

a0 = theta0;
a1 = v0;

% % vf = v0 + 2*a2*tf + 3*a3*tf^2 +4*a4*tf^3;
% 
% a4 = (vf-v0-2*a2*tf-3*a3*tf^2)/(4*tf^3);
% right = theta0 + v0*tf +a2*tf^2 + a3*tf^3 +a4*tf^4
% right = collect(right,[a2,a3])
% (tf^2/2)*a2 + (tf^3/4)*a3 + theta0 + tf*v0 - (tf*(v0 - vf))/4
%  
% % a3 = collect((thetaf-( theta0 + tf*v0 - (tf*(v0 - vf))/4 ) - (tf^2/2)*a2)/(tf^3/4),a2)

a3 = (-2/tf)*a2 - (4*(theta0 - thetaf + tf*v0 - (tf*(v0 - vf))/4))/tf^3
d_theta = a1+2*a2*t+3*a3*t^2+4*a4*t^3

d_theta = 4*a4*t^3 + (- (12*theta0 - 12*thetaf + 12*tf*v0 - 3*tf*(v0 - vf))/tf^3 - (6*a2)/tf)*t^2 + 2*a2*t + v0;
d_theta<=Vmax
d_theta>=Vmin

dd_theta = 2*a2 + 6*a3*t +12*a4*t^2
dd_theta = 12*a4*t^2 + (- (24*theta0 - 24*thetaf + 24*tf*v0 - 6*tf*(v0 - vf))/tf^3 - (12*a2)/tf)*t + 2*a2;
dd_theta<=aMax
dd_theta>=aMin

% a3 = (vf-v0-2*a2*tf-4*a4*tf^3)/(3*tf^2);
% right = theta0 + v0*tf +a2*tf^2 + a3*tf^3 +a4*tf^4
% right = collect(right,[a2,a4])
% thetaf == (tf^2/3)*a2 + (-tf^4/3)*a4 + theta0 + tf*v0 - (tf*(v0 - vf))/3












