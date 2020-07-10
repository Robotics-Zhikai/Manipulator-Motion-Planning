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

%当a4=0时
function1 = collect(2*a2 + (- (24*theta0 - 24*thetaf + 24*tf*v0 - 6*tf*(v0 - vf))/tf^3 - (12*a2)/tf)*tf,a2)
% - 10*a2 - (24*theta0 - 24*thetaf + 24*tf*v0 - 6*tf*(v0 - vf))/tf^2
a4 = 0;
thetaf = pi/4;
theta0 = 0;
tf = 10;
v0 = 0;
vf = 1;
tf = 1;
a2>=aMin/2
a2<=aMax/2
a2>=(aMax+(24*theta0 - 24*thetaf + 24*tf*v0 - 6*tf*(v0 - vf))/tf^2)/(-10)
a2<=(aMin+(24*theta0 - 24*thetaf + 24*tf*v0 - 6*tf*(v0 - vf))/tf^2)/(-10)

a4 ~= 0;



clc
clear
close all

syms ta tb amax t v0 theta0 tf k
% fun = @(x) x.^2
pretty( int (int((amax/ta)*t,t,0,t),t,0,ta) )
pretty( int ( int((amax/ta)*t,t,0,ta) + int(amax,t,ta,t) ,t,ta,tb-ta ) )
pretty( int (int((amax/ta)*t,t,0,ta) + int(amax,t,ta,tb-ta) + int(-(amax/ta)*(t-tb),t,tb-ta,t),t,tb-ta,tb) )
integarequation = ( int (int((amax/ta)*t,t,0,t),t,0,ta) )+ ...
    ( int ( int((amax/ta)*t,t,0,ta) + int(amax,t,ta,t) ,t,ta,tb-ta ) )+ ...
        ( int (int((amax/ta)*t,t,0,ta) + int(amax,t,ta,tb-ta) + int(-(amax/ta)*(t-tb),t,tb-ta,t),t,tb-ta,tb) );
integarequation = collect(integarequation,[ta,tb])
pretty(integarequation)

thetab = theta0 + (amax*tb^2)/2 - (amax*ta*tb)/2
thetah = ((tb-ta)*amax)*(tf/2-tb)+thetab
thetaf = theta0+2*(thetah-theta0)
thetaf = collect(thetaf,tb) %这是需要满足的关系式 thetaf是已知量
tb = -((-amax*(k - 1)*(4*theta0 - 4*thetaf + amax*tf^2 - amax*k*tf^2))^(1/2) - amax*tf + amax*k*tf)/(2*(amax - amax*k))
% tb = (amax*ta + amax*tf - (amax*(amax*ta^2 - 2*amax*ta*tf + amax*tf^2 + 4*theta0 - 4*thetaf))^(1/2))/(2*amax)

clc
clear
close all

syms tb amax t v0 theta0 tf k

ta = k*tb;
% t>=0&&t<ta
thetat = theta0 + int (int((amax/ta)*t,t,0,t),t,0,t)
% t>=ta && t<tb-ta
thetat = subs(thetat,ta) + int ( int((amax/ta)*t,t,0,ta) + int(amax,t,ta,t) ,t,ta,t )
% t>=tb-ta && t<tb
thetat = subs(thetat,tb-ta) + int (int((amax/ta)*t,t,0,ta) + int(amax,t,ta,tb-ta) + int(-(amax/ta)*(t-tb),t,tb-ta,t),t,tb-ta,t)
%t>=tb && t<tf-tb
thetat = subs(thetat,tb) + amax*(tb-ta)*(t-tb)
%t>=tf-tb && t<tf-tb+ta
thetat = subs(thetat,tf-tb) + int(amax*(tb-ta) + int(-(amax/ta)*(t-(tf-tb)),t,tf-tb,t),t,tf-tb,t)
%t>=tf-tb+ta && t<tf-ta
thetat = subs(thetat,tf-tb+ta) + int (amax*(tb-ta) + int(-(amax/ta)*(t-(tf-tb)),t,tf-tb,tf-tb+ta) + int(-amax,t,tf-tb+ta,t) ,t,tf-tb+ta,t )
%t>=tf-ta && t<tf
thetat = subs(thetat,tf-ta) + int (amax*(tb-ta) + int(-(amax/ta)*(t-(tf-tb)),t,tf-tb,tf-tb+ta) + int(-amax,t,tf-tb+ta,tf-ta) + int((amax/ta)*(t-tf),t,tf-ta,t),t,tf-ta,t)
simplify(subs(thetat,tf)-thetaf)





















