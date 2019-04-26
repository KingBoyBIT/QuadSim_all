function [ dstate ] = dynamicFunc( input)
%dynamicFunc 动力学方程
%   欧拉拉格朗日方程

global m g

rx = input(1);
ry = input(2);
rz = input(3);
vx = input(4);
vy = input(5);
vz = input(6);
phi = input(7);
theta = input(8);
psi = input(9);
d_phi = input(10);
d_theta = input(11);
d_psi = input(12);

u = input(13);
t_fai = input(14);
t_theta = input(15);
t_psi = input(16);

ax = (-u*sin(theta))/m;
ay = (u*cos(theta)*sin(phi))/m;
az = (u*cos(theta)*cos(phi)-m*g)/m;

dd_fai = t_fai;
dd_theta = t_theta;
dd_psi = t_psi;

dstate = zeros(12,1);
dstate(1) = vx;
dstate(2) = vy;
dstate(3) = vz;
dstate(4) = ax;
dstate(5) = ay;
dstate(6) = az;
dstate(7) = d_phi;
dstate(8) = d_theta;
dstate(9) = d_psi;
dstate(10) = dd_fai;
dstate(11) = dd_theta;
dstate(12) = dd_psi;

end

