UAV_m = 0.75;
Earth_g = 9.8;

UAV_Ixx = 19.688e-3;
UAV_Iyy = 19.688e-3;
UAV_Izz = 3.938e-2;
UAV_L = 0.25;
UAV_km = 7.5e-7;            %����ϵ��
UAV_kf = 3.13e-5;           %����ϵ��

UAV_wh = sqrt(m*g/(4*kf));
UAV_aaa=inv([kf kf kf kf;
	0 -kf 0 kf;
	-kf 0 kf 0;
	-km km -km km]);

% % % % % ����ֵ
% % % % sp= 0.01;% [s] �������ڣ�����dinamics.m����YiA���õ�
% % % % g= 9.806;% [m/s^2] �������ٶ�
% % % % b= 3.13E-5;% [N.s2] ��ͣ����ϵ��
% % % % d= 7.5E-7;% [Nm.s2] ��ͣ����ϵ��
% % % % deg2rad=1.745329E-2; % [rad/deg]�Ƕȱ任
% % % % rad2deg=57.295779; % [deg/rad]�Ƕȱ任
% % % % P=4;% �������
% % % % L= 0.232;% [m] ���۳���
% % % % m = 0.53;% [kg] ������
% % % % Ixx = 6.228E-3;	%// [kg.m2] ����ϵ��
% % % % Iyy = 6.228E-3;	%// [kg.m2] ����ϵ��
% % % % Izz = 1.121E-2;	%// [kg.m2] ����ϵ��
% % % % 
% % % % %***** Rotor inertia *****/
% % % % r=4; % reduction ratio
% % % % jm= 4e-7;	%// [kg.m2]; 
% % % % jp= 6e-5;	%// [kg.m2]; 
% % % % jr = jp+jm/r;	%// [kg.m2];
% % % % 
% % % % h= 0; %// [m] �������ڸ߶�
% % % % 
% % % % %***** constants of the linear curve Omega=f(bin) *****
% % % % slo=2.7542; % slope (of the linear curve om=f(bin))
% % % % shi=3.627; % shift
% % % % 
% % % % % *************************** �������� ***************************
% % % % 
% % % % % ********* �� ********* 
% % % % N=2; % ��Ҷ����
% % % % R=0.15; % [m] ���뾶
% % % % A=pi*(R^2); % [m^2] ��Բ���
% % % % c=0.0394; % [m] ���ҳ�
% % % % theta0=0.2618; % [rad] ������
% % % % thetatw=0.045; % [rad] ��Ťת��
% % % % sigma_=(N*c)/(pi*R); % solidity ratio (rotor fill ratio) [rad^-1]
% % % % a=5.7; % ����ϵ��б�� (given in literature)
% % % % Cd=0.052; % ����ϵ�� -- found by tests
% % % % Ac=0.005; % [m^2] ��������
% % % % 
% % % % rho= 1.293; % [kg/m^3] �����ܶ�
% % % % nu=1.8e-5; % [Pa.s] 20���¿���ճ��
% % % % 
% % % % w=(m/P)*g; % [N] ÿ������е������ط���
% % % % OmegaH=sqrt(w/b); % [rad/s] ��ͣ���ٶ�
% % % % OmegaMax = 600; %[rad/s] �����ٶ�
% % % % % ********* ��������ϵ�� *********
% % % % Cx=1.32; 
% % % % Cy=1.32; 
% % % % Cz=1.32; 
% % % % 
% % % % Vol=3.04E-4; 	% [m3] ���
% % % % PArchim = rho*g*Vol; % [N] ��������
% % % % % ***** ANNEXE ! ******
% % % % %Ftb=0.5*Cz*A*rho*v^2 % turbulent
% % % % %Fl=16*0.3*nu*v % laminary 


controlpara