clear,clc,close all
% constants file
% INPUT: NaF
% OUTPUT: all constants
constant
global delta_old delta_sum delta_angle_old SIM_dt

delta_old = zeros(3,1);
delta_sum = zeros(3,1);
delta_angle_old = zeros(3,1);
SIM_dt = 1;