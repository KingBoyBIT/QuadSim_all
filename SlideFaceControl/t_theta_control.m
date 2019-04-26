function [ tau_phi ] = t_theta_control( input )
%t_psi_control 俯仰控制力矩
%   此处显示详细说明
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
ry_d = input(13);


tau_phi=-sigma_phi_1(d_phi+sigma_phi_2(d_phi+phi+sigma_phi_3(d_phi+2*phi+vy/g+sigma_phi_4(d_phi+3*phi+3*vy/g+(ry-ry_d)/g))));

end

function out = sigma_phi_1(input)
alpha = 50.0;
if input>alpha
    out = alpha;
elseif input<-alpha
    out = -alpha;
else
    out = input;
end

end
function out = sigma_phi_2(input)
alpha = 50.05;
if input>alpha
    out = alpha;
elseif input<-alpha
    out = -alpha;
else
    out = input;
end

end
function out = sigma_phi_3(input)
alpha = 50.5;
if input>alpha
    out = alpha;
elseif input<-alpha
    out = -alpha;
else
    out = input;
end

end
function out = sigma_phi_4(input)
alpha = 0.5;
if input>alpha
    out = alpha;
elseif input<-alpha
    out = -alpha;
else
    out = input;
end

end