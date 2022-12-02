%% Author: Jack Lambert
% ASEN 3128
% Purpose: Function for ODE45 to call to calculate the State variables
% u_dot, w_dot, q_dot, and theta_dot for the full linearized longitudinal
% set
% Last Edited: 4/9/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dydt] = ODEcall_Full(t,y,K_mat)

u_dot = y(1); % x-compoinent of Inertial velocity, Body Frame
w_dot = y(2); % z-compoinent of Inertial velocity, Body Frame
q_dot = y(3); % y-component of Angular Velocity, Body Frame
theta_dot = y(4); % Pitch Angle 

%% State Variable Matrix for Linearized Longitudinal Set
[ ~, A_BK_full] = Linearized(K_mat); % A matrix function based on plane and parameters
State = [u_dot, w_dot, q_dot, theta_dot]'; % Couple State Variables in Long. Set
var = A_BK_full*State; % Couple State Variables in Long. Set
%% Solving for State Variables in the Linearized Longitudinal Set
dydt(1) = var(1); % uE
dydt(2) = var(2); % wE
dydt(3) = var(3); % q
dydt(4) = var(4); % theta

dydt = dydt'; % Inverts for ODE45   
end