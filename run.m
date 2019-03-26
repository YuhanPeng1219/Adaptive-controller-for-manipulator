close all;
clear all;
% Answer to Problem 1
Problem1;

% Answer to Problem 2 Note: need input to run 
jointAngle_ini = [0.2, -0.7];
jointVelocity_ini = [0, 0];
Estimate_param = [1.2, 1.5];
Ini_states = [jointAngle_ini; jointVelocity_ini];
Adaptive_control(Ini_states, Estimate_param);