UAV_m = 0.75;
Earth_g = 9.8;
UAV_Ixx = 19.688e-3;
UAV_Iyy = 19.688e-3;
UAV_Izz = 3.938e-2;
UAV_L = 0.25;
UAV_km = 7.5e-7;            %力矩系数
UAV_kf = 3.13e-5;           %升力系数

UAV_wh = sqrt(UAV_m*Earth_g/(4*UAV_kf));
UAV_aaa=inv([UAV_kf UAV_kf UAV_kf UAV_kf;
	0 -UAV_kf 0 UAV_kf;
	-UAV_kf 0 UAV_kf 0;
	-UAV_km UAV_km -UAV_km UAV_km]);
UAV_xf=3;
UAV_yf=4;
UAV_zf=5;

UAV_kpx = 0;             %x 0.96  0.113
UAV_kix = 0;
UAV_kdx = 0;
UAV_kpy = 0;                %y
UAV_kiy = 0;
UAV_kdy = 0;
UAV_kpz = 0;                %z
UAV_kiz = 0;
UAV_kdz = 0;

UAV_kpphi = 0.0;            %滚转 15  193
UAV_kdphi = 0;                  %20
UAV_kptheta = UAV_kpphi;          %俯仰
UAV_kdtheta = UAV_kdphi;
UAV_kppsai = 0;           %偏航
UAV_kdpsai = 0;
UAV_kipsai = 0;
