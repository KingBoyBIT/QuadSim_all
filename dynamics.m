% UAV动力学模型

function out = dynamics(in)
%               x+
%               2+
%               |
%               |
%               |
%    -          |         -
% y+ 1----------z---------3 y-
%               |
%               |
%               |
%               |
%               4+
%               x-
constant
%input states and omegas
x = in(1);
x_dot = in(2);
y = in(3);
y_dot = in(4);
z = in(5);
z_dot= in(6);
roll = in(7);
roll_dot= in(8);
pitch = in(9);
pitch_dot= in(10);
yaw = in(11);
yaw_dot= in(12);
% 控制输入（角速度）
% 1 3 为正，2 4 为负，十字模式
omega = in(13:16);

%输入力和力矩
T_motor=in(17:20);%推力
M_motor=in(21:24);%力矩

C_n_b = angle2dcm(yaw,pitch,roll,'ZYX');% 从大地系转本体系
C_b_n = C_n_b';%从本体系转大地系
UAV_I = diag([UAV_Ixx UAV_Iyy UAV_Izz]);

M_bx = UAV_L*(T_motor(1)-T_motor(3));
M_by = UAV_L*(T_motor(2)-T_motor(4));
M_bz = (M_motor(1)+M_motor(3)-M_motor(2)-M_motor(4));

M_b = [M_bx;M_by;M_bz];
d_w_n = inv(UAV_I)*C_b_n*M_b;


% Rolling moments
Rbg = -pitch_dot*yaw_dot*(Iyy-Izz);                
Rgp = jr*pitch_dot*Om;                            
Raa = L*(-T(2)+T(4));                           
Rhm = (Hy(1)+Hy(2)+Hy(3)+Hy(4))*h;              
Rrm = +RRx(1)-RRx(2)+RRx(3)-RRx(4);             
Rfm = 0.5*Cz*A*rho*roll_dot*abs(roll_dot)*L*(P/2)*L;   

% Pitch moments
Pgb = roll_dot*yaw_dot*(Izz-Ixx); 
Pgp = jr*roll_dot*Om; 
Paa = L*(-T(1)+T(3)); 
Phf = (Hx(1)+Hx(2)+Hx(3)+Hx(4))*h; 
Prm = +RRy(1)-RRy(2)+RRy(3)-RRy(4);            
Pfm = 0.5*Cz*A*rho*pitch_dot*abs(pitch_dot)*L*(P/2)*L; 

% Yaw moments
Ygb = pitch_dot*roll_dot*(Ixx-Iyy); 
Yict = jr*(Om-Om_old)/sp;           
Yct = +Q(1)-Q(2)+Q(3)-Q(4);              
Yhfx = (-Hx(2)+Hx(4))*L;                    
Yhfy = (-Hy(1)+Hy(3))*L; 

Om_old=Om;


% X forces
Xaa = (sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); 
Xdf = 0.5*Cx*Ac*rho*x_dot*abs(x_dot); 
Xhf = Hx(1)+Hx(2)+Hx(3)+Hx(4);

 % Y forces
Yaa = (-cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); 
Ydf = 0.5*Cy*Ac*rho*y_dot*abs(y_dot); 
Yhf = Hy(1)+Hy(2)+Hy(3)+Hy(4); 

% Z forces
Zaa = (cos(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4));              
Zaf = 0.5*Cz*A*rho*z_dot*abs(z_dot)*P + 0.5*Cz*Ac*rho*z_dot*abs(z_dot); 

%outputs
out(1) = x_dot;
out(2) = (Xaa - Xdf - Xhf)/m;
out(3) = y_dot;
out(4) = (Yaa - Ydf - Yhf) / m;
out(5) = z_dot;
out(6) = -g + (Zaa - Zaf)/m;
out(7) = roll_dot;
out(8) = (Rbg + Rgp + Raa + Rhm + Rrm - Rfm) /Ixx;
out(9) = pitch_dot;
out(10) = (Pgb - Pgp + Paa - Phf + Prm - Pfm) /Iyy;
out(11) = yaw_dot;
out(12) = (Ygb + Yict +Yct + Yhfx + Yhfy) /Izz;


