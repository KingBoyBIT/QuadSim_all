yaw = 0;
km = 7.5e-7;            %����ϵ��
kf = 3.13e-5;           %����ϵ��

wh = sqrt(m*g/(4*kf));
aaa=inv([kf kf kf kf;
	0 -kf 0 kf;
	-kf 0 kf 0;
	-km km -km km]);

xf=3;
yf=4;
zf=5;

kpx = 0.50;             %x 0.96  0.113
kix = 0.01;
kdx = 0.8;
kpy = 5.0;                %y
kiy = 0.01;
kdy = 5.0;
kpz = 100.0;                %z
kiz = 1.0;
kdz = 10.0;

kpphi = 2000;            %��ת 15  193
kdphi = 100;                  %20
kptheta = kpphi;          %����
kdtheta = kdphi;
kppsai = 0;           %ƫ��
kdpsai = 0;
kipsai = 0;