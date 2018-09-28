clear,clc,close all
%% �����������
dt=0.01;
stime=20;
loop=stime/dt;

%% ��ʼ״̬
state=zeros(12,1);
state(1:3,:)=[0;0;0];% r
state(4:6,:)=[0;0;0];% v
state(7,:)=0;
state(8,:)=0;
state(9,:)=0;
rx=state(1);ry=state(2);rz=state(3);
vx=state(4);vy=state(5);vz=state(6);
phi=state(7);theta=state(8);psi=state(9);
vphi=state(10);vtheta=state(11);vpsi=state(12);

% ��ƹ켣��ֵ
x_leader=[0;0;0];
v_leader=[0;0;0];
% ���������
p = quadpara();
% �켣��¼
xyHis=zeros(3,2,loop+1);% ��һ��Ϊ��ƹ켣���ڶ���Ϊʵ�ʹ켣
xyHis(:,:,1)=[x_leader state(1:3)];

%% simulation start
sp=1;
omegaHis=zeros(4,loop);
for ct=1:loop
% 	a_leader = randn(3,1);
% 	v_leader=v_leader+dt*a_leader;
	v_leader = [-0.05*cos(ct*dt);-0.05*sin(ct*dt);dt];
    x_leader = x_leader+dt*v_leader;

    % ͨ����������ÿ��ƽ��ٶ�
    omega=quad_control(state,x_leader,v_leader,0,p,1,10);
    
    % ��¼���ƽ��ٶ�
    omegaHis(:,ct)=omega;
    
    % ״̬����
    state = quadcopter(state,omega,p,dt);
    
    % ��¼���й켣
    xyHis(:,:,ct+1)=[x_leader state(1:3)];
    
end

% ������ʾ
figure(1)
axis equal
grid on
plotHis3(xyHis,dt,-1,200)


%show changes in motor speeds during the flight
figure(2)
plot(omegaHis')
grid on
