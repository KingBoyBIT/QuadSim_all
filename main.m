clear,clc,close all
%% �����������
dt=0.01;
stime=20;
loop=stime/dt;

%% ��ʼ״̬
s=zeros(12,1);
s(1:3,:)=[0;0;0];% r
s(4:6,:)=[0;0;0];% v
s(7,:)=0;
s(8,:)=0;
s(9,:)=0;
rx=s(1);ry=s(2);rz=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);

% ��ƹ켣��ֵ
x_leader=[0;0;0];
v_leader=[0;0;0];
% ���������
quadpara
% �켣��¼
xyHis=zeros(3,2,loop+1);% ��һ��Ϊ��ƹ켣���ڶ���Ϊʵ�ʹ켣
xyHis(:,:,1)=[x_leader s(1:3)];

%% simulation start
sp=1;
omegaHis=zeros(4,loop);
for ct=1:loop
 
	v_leader = [-0.05*cos(ct*dt);-0.05*sin(ct*dt);dt];
    x_leader=x_leader+dt*v_leader;

    % ͨ����������ÿ��ƽ��ٶ�
    omega=quad_control(s,x_leader,v_leader,0,para,1,10);
    
    % ��¼���ƽ��ٶ�
    omegaHis(:,ct)=omega;
    
    % ״̬����
    s = quadcopter(s,omega,para,dt);
    
    % ��¼���й켣
    xyHis(:,:,ct+1)=[x_leader s(1:3)];
    
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
