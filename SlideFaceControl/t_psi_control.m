function [ tau_psi ] = t_psi_control( input )
%t_psi_control ƫ����������
%   �˴���ʾ��ϸ˵��
global a_psi_1 a_psi_2
psi = input(9);

d_psi = input(12);
psi_d = input(13);

tau_psi=-a_psi_1*d_psi-a_psi_2*(psi-psi_d);

end

