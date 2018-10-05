function omega=quad_control(s,xl,vl,psil,para)
% 整理输入
s=real(s);
x=s(1);y=s(2);z=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);

k1 = para.ctl_tanh.ctl_k1;
k2 = para.ctl_tanh.ctl_k2;

u=zeros(4,1);
tr=[para.b para.b para.b para.b;
	0 -para.b 0 para.b;
	-para.b 0 para.b 0;
	-para.d para.d -para.d para.d];


input_delta=k1*(xl-[x;y;z])+k2*(vl-[vx;vy;vz]);
attr=para.ctl_tanh.ctl_c0*saturate(input_delta)+[0;0;para.g];
rot=[cos(psi) sin(psi) 0;-sin(psi) cos(psi) 0;0 0 1];
attr=rot*attr;

u(1)=para.m*norm(attr);

phi_p=asin(-attr(2)/norm(attr));
theta_p=atan(attr(1)/attr(3));
if attr(3)<0
	theta_p=theta_p-pi;
end
psi_p=psil;

u(4)=para.Iz*saturate(angleDelta(psi_p,psi)+0-vpsi);
u(2)=para.ctl_tanh.ctl_fai*para.Ix*(u(1)+u(4))/2*saturate(para.ctl_tanh.ctl_fai1*angleDelta(phi_p,phi)+para.ctl_tanh.ctl_fai2*vphi)/para.l;
u(3)=para.ctl_tanh.ctl_theta*para.Iy*(u(1)-u(4))/2*saturate(para.ctl_tanh.ctl_theta1*angleDelta(theta_p,theta)+para.ctl_tanh.ctl_theta2*vtheta)/para.l;

omega2=inv(tr)*u;
omega2(omega2<0)=0;
omega=omega2.^0.5;
omega(omega>para.omegaMax) = para.omegaMax;
end

% function out = 
% end

function u=saturate(input)
% 饱和处理
u=tanh(input);
end

function ang=angleDelta(p2,p1)
% 计算角度差
ang=sign(det([[cos(p1);sin(p1)] [cos(p2);sin(p2)]]))*acos(dot([cos(p1);sin(p1)],[cos(p2);sin(p2)]));
end