% UAV动力学模型
function out = dynamics(in)
%               x+
%               2+
%               |
%               |
%               |
%    -          |           -
% y+ 1—————z—————3 y-
%               |
%               |
%               |
%               |
%               4+
%               x-

constant
% 获取输入值
if true
	%input states and omegas
	x = in(1);
	x_dot = in(2);
	y = in(3);
	y_dot = in(4);
	z = in(5);
	z_dot = in(6);
	fai = in(7);
	theta = in(8);
	psi = in(9);
	Wx = in(10);
	Wy = in(11);
	Wz = in(12);
	% 控制输入
	% 1 3 为正，2 4 为负，十字模式
	%输入力和力矩
	T_motor=in(13:16);%推力
	M_motor=in(17:20);%力矩
end
% 坐标变换
if true
	C_n_b = angle2dcm(psi,theta,fai,'ZYX');% 从大地系转本体系
	C_b_n = C_n_b';% 从本体系转大地系
	UAV_I = diag([UAV_Ixx UAV_Iyy UAV_Izz]);
	T_omega2dangle = zeros(3,3);
	T_omega2dangle(1,1) = 1;
	T_omega2dangle(1,2) = sin(fai)*tan(theta);
	T_omega2dangle(1,3) = cos(fai)*tan(theta);
	T_omega2dangle(2,2) = cos(fai);
	T_omega2dangle(2,3) = -sin(fai);
	T_omega2dangle(3,2) = sin(fai)/cos(theta);
	T_omega2dangle(3,3) = cos(fai)/cos(theta);
	
	% T_dangle2omega = zeros(3,3);
	% T_dangle2omega(1,1) = 1;
	% T_dangle2omega(1,3) = -sin(pitch);
	% T_dangle2omega(2,2) = cos(roll);
	% T_dangle2omega(2,3) = sin(roll)*cos(pitch);
	% T_dangle2omega(3,2) = -sin(roll);
	% T_dangle2omega(3,3) = cos(roll)*cos(pitch);
	W_n = [Wx;Wy;Wz];
	dangle = T_omega2dangle*W_n;
	roll_dot = dangle(1);
	pitch_dot = dangle(2);
	yaw_dot = dangle(3);
end
% 质心动力学方程
if true
	F_b = [0;0;sum(T_motor)];
	G_n = [0;0;-Earth_g];
	acc_n = (C_b_n*F_b)/UAV_m+G_n;
	acc_n_x = acc_n(1);
	acc_n_y = acc_n(2);
	acc_n_z = acc_n(3);
end
% 质点系动力学方程
if true
	M_bx = UAV_L*(T_motor(1)-T_motor(3));
	M_by = UAV_L*(T_motor(2)-T_motor(4));
	M_bz = (M_motor(1)+M_motor(3)-M_motor(2)-M_motor(4));
	
	M_b = [M_bx;M_by;M_bz];
	d_W_n = inv(UAV_I)*(C_b_n*M_b-cross(W_n,UAV_I*W_n));
	
end
% outputs
if true
	out(1) = x_dot;
	out(2) = acc_n_x;
	out(3) = y_dot;
	out(4) = acc_n_y;
	out(5) = z_dot;
	out(6) = acc_n_z;
	out(7) = roll_dot;
	out(8) = pitch_dot;
	out(9) = yaw_dot;
	out(10) = d_W_n(1);
	out(11) = d_W_n(2);
	out(12) = d_W_n(3);
end
end

