clear,clc,close all
%% 仿真参数设置
dt=0.01;
stime=500;
loop=stime/dt;

%% 初始状态
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

% public virtual leadr init
x_leader=[0;0;0];
v_leader=[0;0;0];

%parameters for quadrotor
para.g=9.8;
para.m=1.2;
para.Iy=0.05;
para.Ix=0.05;
para.Iz=0.1;
para.b=10^-4;
para.l=0.5;
para.d=10^-6;
para.Jr=0.01;
para.k1=0.02;
para.k2=0.02;
para.k3=0.02;
para.k4=0.1;
para.k5=0.1;
para.k6=0.1;
para.omegaMax=330;

% 轨迹记录
xyHis=zeros(3,2,loop+1);% 第一列为标称轨迹，第二列为实际轨迹
xyHis(:,:,1)=[x_leader s(1:3)];

%% simulation start
sp=1;
omegaHis=zeros(4,loop);
for t=1:loop
 
	a_leader = [5*sin(t);5*cos(t);dt];
    v_leader=v_leader+dt*a_leader;
    x_leader=x_leader+dt*v_leader;

    % get motor speeds form the controller
    omega=quad_control(s,x_leader,v_leader,0,para,1,10);
    
    %record the speeds
    omegaHis(:,t)=omega;
    
    %send speeds of four motors to quadrotor and get its state
    s=quadrotor_kinematics(s,omega,para,dt);
    
    %recodrd the position of quadrotor at time t/loop*stime
    xyHis(:,:,t+1)=[x_leader s(1:3)];
    
end

%show the animation of the flight process
figure(1)
plotHis3(xyHis,dt,-1,200)
axis equal
grid on

%show changes in motor speeds during the flight
figure(2)
plot(omegaHis')
grid on
