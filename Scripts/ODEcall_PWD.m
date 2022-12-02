%% Author: Jack Lambert
% ASEN 3128
% Purpose: Function for ODE45 to call to calculate the State variables 
% u_dot, w_dot, q_dot, and theta_dot for the PWD Approximation. This function
% uses the simplified assumptions for the Linearized Longitudinal Dynamics Set
% Last Edited: 4/9/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dydt] = ODEcall_PWD(t,y,K_mat)

u_dot = y(1); % x-component of Velocity, Body Frame
theta_dot = y(2); % Pitch Angle 

%% State Variable Matrix for Linearized Longitudinal Set
[A_BK, ~ ] = Linearized(K_mat); % A matrix function based on plane and parameters
State = [u_dot, theta_dot]'; % Couple State Variables in Long. Set
var = A_BK*State; % Couple State Variables in Long. Set
%% Solving for State Variables in the Linearized Longitudinal Set
dydt(1) = var(1); % uE
dydt(2) = var(2); % theta

dydt = dydt'; % Inverts for ODE45   
end