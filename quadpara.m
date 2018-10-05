function para = quadpara()
% 四旋翼参数
para.g=9.8;% 重力
para.l=0.5;% 旋翼臂长（单臂）
para.m=1.2;% 质量
para.Ix=0.05;% 惯性矩
para.Iy=0.05;
para.Iz=0.1;
para.b=10^-4;
para.d=10^-6;
para.omegaMax=330;% 角速度边界值

% 控制参数

para.ctl_tanh.ctl_k1 = 10;
para.ctl_tanh.ctl_k2 = 20;
para.ctl_tanh.ctl_fai = 20;
para.ctl_tanh.ctl_fai1 = 1;
para.ctl_tanh.ctl_fai2 = -1;
para.ctl_tanh.ctl_theta = 20;
para.ctl_tanh.ctl_theta1 = 1;
para.ctl_tanh.ctl_theta2 = -1;
end