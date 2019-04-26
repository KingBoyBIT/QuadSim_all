function [ u ] = u_control( input )
%u_control ����������
%   �˴���ʾ��ϸ˵��
global a_z_1 a_z_2 m g

z = input(3);
dz = input(6);
phi = input(10);
theta = input(11);
z_d = input(13);
r_1=-a_z_1*dz-a_z_2*(z-z_d);

u=(r_1+m*g)/(cos(theta)*cos(phi));


end

