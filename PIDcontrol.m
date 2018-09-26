function out = PIDcontrol(in)

constant;
global delta_old delta_sum delta_angle_old SIM_dt
target_r = in(1:3);
current_r = in(4:6);
current_angle = in(7:9);

% current_angle = max(current_angle,-pi);
% current_angle = min(current_angle,pi);

delta_r = target_r - current_r;

% delta_rx = delta_r(1);
% delta_ry = delta_r(2);
% delta_rz = delta_r(3);
% 
% target_rx = target_r(1);
% target_ry = target_r(2);
% target_rz = target_r(3);

delta_sub = delta_r-delta_old;

dd_p = delta_r.*[UAV_kpx;UAV_kpy;UAV_kpz];
dd_i = delta_sum.*[UAV_kix;UAV_kiy;UAV_kiz];
dd_d = delta_sub/SIM_dt.*[UAV_kdx;UAV_kdy;UAV_kdz];

dd_a = dd_p+dd_i+dd_d;

delta_old = delta_r;
delta_sum = delta_sum + delta_r;

U1 = sqrt((UAV_m*dd_a(1))^2+(UAV_m*dd_a(2))^2+(UAV_m*dd_a(3)+UAV_m*Earth_g)^2);
angle_des = zeros(3,1);
yaw_des = 0;
angle_des(1) = asin(UAV_m*(sin(yaw_des)*dd_a(1)-cos(yaw_des)*dd_a(2))...
	/sqrt((UAV_m*dd_a(1))^2+(UAV_m*dd_a(2))^2+(UAV_m*dd_a(3)+UAV_m*Earth_g)^2));

angle_des(2) = atan(Earth_g * (sin(dd_a(1))*dd_a(3)+cos(dd_a(1))*dd_a(2))...
	/(dd_a(3)+Earth_g));
angle_des(3) = yaw_des;

angle_des = zeros(3,1);
angle_des(1) = 0.5;
angle_des(2) = 0.5;
angle_des(3) = 0;
% angle_des = max(angle_des,-pi/2);
% angle_des = min(angle_des,pi/2);

delta_angle = angle_des - current_angle;
motor_control = delta_angle.*[UAV_kpphi;UAV_kptheta;UAV_kppsai]+...
	(delta_angle-delta_angle_old)/SIM_dt.*[UAV_kdphi;UAV_kdtheta;UAV_kdpsai];
delta_angle_old = delta_angle;
ddphi = motor_control(1);
ddtheta = motor_control(2);
ddpsi = motor_control(3);

u =zeros(4,1);
control_out = zeros(4,1);
u(1) = U1;
u(2) = ddphi*UAV_Ixx/UAV_L;
u(3) = ddtheta*UAV_Iyy/UAV_L;
u(4) = UAV_Izz*ddpsi;
% 这里得到的是角速度的平方

% 解决姿态角小角度变化的灵异事件，aaa不要用求逆，应转置
control_out(1) = UAV_aaa(1,1)*u(1)+UAV_aaa(1,2)*u(2)+UAV_aaa(1,3)*u(3)+UAV_aaa(1,4)*u(4);
control_out(2) = UAV_aaa(2,1)*u(1)+UAV_aaa(2,2)*u(2)+UAV_aaa(2,3)*u(3)+UAV_aaa(2,4)*u(4);
control_out(3) = UAV_aaa(3,1)*u(1)+UAV_aaa(3,2)*u(2)+UAV_aaa(3,3)*u(3)+UAV_aaa(3,4)*u(4);
control_out(4) = UAV_aaa(4,1)*u(1)+UAV_aaa(4,2)*u(2)+UAV_aaa(4,3)*u(3)+UAV_aaa(4,4)*u(4);
% control_out=max(control_out,0);
% control_out=min(control_out,(UAV_maxRPM*2*pi/60)^2);

out = control_out;
end