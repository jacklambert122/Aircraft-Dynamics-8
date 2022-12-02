%% Author: Jack Lambert
% ASEN 3128
% Purpose: This function calculates the dimensional derivatives for the
% state matrix and the control matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X, Z, M, X_c, Z_c, M_c ] = NonDimLong(rho,u0,S,W,theta0,Cx,Cz,...
    Cm,cbar,C_x_de,C_z_de,C_m_de)

% Computing the Nondimesnional Inital Weight Derivative
Cw0 = W/ ((1/2)*rho*S*u0^2);

%% State Variable Derivatives
% X
Xu = rho*u0*S*Cw0*sind(theta0) + .5*rho*u0*S*Cx(1);
Xw = .5*rho*u0*S*Cx(2);
Xq = .25*rho*u0*cbar*S*Cx(3);
Xwdot = .25*rho*cbar*S*Cx(4);

X = [Xu, Xw, Xq, Xwdot]';

% Z
Zu = -rho*u0*S*Cw0*cosd(theta0) + .5*rho*u0*S*Cz(1);
Zw = .5*rho*u0*S*Cz(2);
Zq = .25*rho*u0*cbar*S*Cz(3);
Zwdot = .25*rho*cbar*S*Cz(4);

Z = [Zu, Zw, Zq, Zwdot]';

% M
Mu = .5*rho*u0*cbar*S*Cm(1);
Mw = .5*rho*u0*cbar*S*Cm(2);
Mq = .25*rho*u0*(cbar^2)*S*Cm(3);
Mwdot = .25*rho*(cbar^2)*S*Cm(4);

M = [Mu, Mw, Mq, Mwdot]';

%% Control Derivatives

% Elevator controls
X_c(1) = 1/2*rho*u0^2*S*C_x_de;
Z_c(1) = 1/2*rho*u0^2*S*C_z_de;
M_c(1) = 1/2*rho*u0^2*S*cbar*C_m_de;

% Thrust Controls
X_c(2) = 0;
Z_c(2) = 0;
M_c(2) = 0;


