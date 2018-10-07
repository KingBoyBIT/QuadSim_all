clear,clc,close all
%% �����������
dt=0.01;
stime=15;
loop=stime/dt;

%% ��ʼ״̬
state=zeros(12,1);
state(1:3,:)=[0;0;0];% r
state(4:6,:)=[0;0;0];% v
state(7,:)=0;
state(8,:)=0;
state(9,:)=0;
state(10,:)=0;
state(11,:)=0;
state(12,:)=0;
rx=state(1);ry=state(2);rz=state(3);
vx=state(4);vy=state(5);vz=state(6);
phi=state(7);theta=state(8);psi=state(9);
vphi=state(10);vtheta=state(11);vpsi=state(12);

% ��ƹ켣��ֵ
x_leader=[0;0;0];
v_leader=[0;0;0];
psi_leader = 0;
% ���������
p = quadpara();
% �켣��¼
xyHis=zeros(3,2,loop+1);% ��һ��Ϊ��ƹ켣���ڶ���Ϊʵ�ʹ켣
xyHis(:,:,1)=[x_leader state(1:3)];

%% simulation start
sp=1;
omegaHis=zeros(4,loop);
for ct=1:loop
	% ��ƹ켣����
	%a_leader = randn(3,1);
% 	v_leader=v_leader+dt*a_leader;
	v_leader = [-0.5*cos(ct*dt);-0.5*sin(ct*dt);10*dt];
	x_leader = x_leader+dt*v_leader;
	psi_leader = psi_leader + dt*0.1;% ����ƫ������΢תһת
	% ״̬�۲�ƫ��
	state_with_noise = state;
% 	state_with_noise = state + [0.01*randn(3,1);0.01*randn(3,1);0.001*randn(3,1);0.001*randn(3,1)];
	
	% ͨ����������ÿ��ƽ��ٶ�
	omega=quad_control(state_with_noise,x_leader,v_leader,psi_leader,p);
	
	% ��¼�����������������ٶȣ�
	omegaHis(:,ct)=omega;
	
	% ״̬����
	state = quad_dynamic(state,omega,p,dt);
	
	% ��¼���й켣
	xyHis(:,:,ct+1)=[x_leader state(1:3)];
	
end

% ������ʾ
figure(1)

plotHis3(xyHis,dt,-1,200)


% ��ʾ�����е�����ٶ�
figure(2)
plot(0:dt:stime-dt,omegaHis')
grid on
legend