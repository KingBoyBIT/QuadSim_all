clear,clc,close all
%% 仿真参数设置
dt=0.01;
stime=15;
loop=stime/dt;

%% 初始状态
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

% 标称轨迹初值
x_leader=[0;0;0];
v_leader=[0;0;0];
psi_leader = 0;
% 四旋翼参数
p = quadpara();
% 轨迹记录
xyHis=zeros(3,2,loop+1);% 第一列为标称轨迹，第二列为实际轨迹
xyHis(:,:,1)=[x_leader state(1:3)];

%% simulation start
sp=1;
omegaHis=zeros(4,loop);
for ct=1:loop
	% 标称轨迹更新
	%a_leader = randn(3,1);
% 	v_leader=v_leader+dt*a_leader;
	v_leader = [-0.5*cos(ct*dt);-0.5*sin(ct*dt);10*dt];
	x_leader = x_leader+dt*v_leader;
	psi_leader = psi_leader + dt*0.1;% 期望偏航角略微转一转
	% 状态观测偏差
	state_with_noise = state;
% 	state_with_noise = state + [0.01*randn(3,1);0.01*randn(3,1);0.001*randn(3,1);0.001*randn(3,1)];
	
	% 通过控制器获得控制角速度
	omega=quad_control(state_with_noise,x_leader,v_leader,psi_leader,p);
	
	% 记录控制量（电机输入角速度）
	omegaHis(:,ct)=omega;
	
	% 状态更新
	state = quad_dynamic(state,omega,p,dt);
	
	% 记录飞行轨迹
	xyHis(:,:,ct+1)=[x_leader state(1:3)];
	
end

% 动画演示
figure(1)

plotHis3(xyHis,dt,-1,200)


% 显示飞行中电机角速度
figure(2)
plot(0:dt:stime-dt,omegaHis')
grid on
legend