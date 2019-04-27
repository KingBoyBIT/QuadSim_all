clear,clc,close all

global m g l I_1 I_2 I_3 K_1 K_2 K_3 K_4 K_5 K_6

m = 2;
g = 9.8;
l = 0.20;
I_1 = 1.25;
I_2 = 1.25;
I_3 = 2.5;
K_1 = 0.010;
K_2 = 0.010;
K_3 = 0.010;
K_4 = 0.012;
K_5 = 0.012;
K_6 = 0.012;

state0 = [0;0;0;0;0;0;0;0;0;0;0;0];
state_des = [4;4;5;pi/4];

k_p_x = 1.8;
k_p_y = 1.8;
k_d_x = 2.5;
k_d_y = 2.5;
k_p_z = 5.5;
k_d_z = 4.5;
M_phi = 25;
M_theta = 25;
M_psi = 25;
k_phi = 1;
k_theta = 1;
k_psi = 1;


k_1_phi = 5.5;
k_2_phi = 10;
k_1_theta = 5.5;
k_2_theta = 10;
k_1_psi = 8.5;
k_2_psi = 20;