clear,clc,close all

state0 = [0;0;0;0;0;0;0;0;0.1;0;0;0];
global m g a_z_1 a_z_2  a_psi_1 a_psi_2

m = 0.5;
g = 9.8;
a_z_1 = 5;
a_z_2 = 20;
a_psi_1 = 13;
a_psi_2 = 80;

syms a

mtx = [1 1 1 1;
	0 -1 0 1;
	-1 0 1 0;
	-1 1 -1 1];

inv(mtx)