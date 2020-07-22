clear
close all
clc

syms k1 k2 k3 d_k3 x0 x1 y0 y1 z0 z1 
d_k2 = -(- (2109*cos(k2 + k3))/(2109*cos(k2 + k3) + 4600*cos(k2)) - (2109*sin(k2 + k3)*cos(k1)*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)))*d_k3/(- (cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) - 1);
Vx = - (d_k2*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*cos(k1))/10;
Vy = - (d_k2*sin(k1)*(2109*sin(k2 + k3) + 4600*sin(k2)))/10 - (2109*d_k3*sin(k2 + k3)*sin(k1))/10;
Vz = d_k2*((2109*cos(k2 + k3))/10 + 460*cos(k2)) + (2109*d_k3*cos(k2 + k3))/10;

Vx = simplify(Vx)
Vx = collect(Vx,d_k3)
Vy = simplify(Vy)
Vy = collect(Vy,d_k3)
Vz = simplify(Vz)
Vz = collect(Vz,d_k3)
((2109*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(x0*cos(k2 + k3) - x1*cos(k2 + k3) + z0*sin(k2 + k3)*cos(k1) - z1*sin(k2 + k3)*cos(k1)))/(10*(2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)*((cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) + 1)) - (2109*sin(k2 + k3)*cos(k1))/10)*d_k3
-d_k3*((2109*sin(k2 + k3)*cos(k1))/10 - (2109*cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(x0*cos(k2 + k3) - x1*cos(k2 + k3) + z0*sin(k2 + k3)*cos(k1) - z1*sin(k2 + k3)*cos(k1)))/((21090*cos(k2 + k3) + 46000*cos(k2))*(x0 - x1)*((cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) + 1)))
((2109*cos(k2 + k3))/10 - (2109*((2109*cos(k2 + k3))/10 + 460*cos(k2))*(x0*cos(k2 + k3) - x1*cos(k2 + k3) + z0*sin(k2 + k3)*cos(k1) - z1*sin(k2 + k3)*cos(k1)))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)*((cos(k1)*(2109*sin(k2 + k3) + 4600*sin(k2))*(z0 - z1))/((2109*cos(k2 + k3) + 4600*cos(k2))*(x0 - x1)) + 1)))*d_k3
 