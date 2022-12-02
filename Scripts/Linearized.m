%% Author: Jack Lambert
% ASEN 3128
% Homework 8
% Purpose: To keep all constants in one function so they are not defined
% more than once and then Compute the constants for the state variable
% matrix A and the input matrix B. This function outputs the closed loop
% matrix given the optimal gains
% Date Modified: 4/9/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [A_BK, A_BK_full] = Linearized(K_mat)
%% Airplane Parameters
% Nondimensional Derivatives
% Table 6.1 -
Cx = [-.108, .2193, 0, 0];
Cz = [-.106, -4.92, -5.921, 5.896];
Cm = [.1043, -1.023, -23.92, -6.314];

% Nondimensional Elevator Derivatives (Page 229 in Etkin)
C_x_de = -3.818*10^-6;
C_z_de = -0.3648;
C_m_de = -1.444;

% Table E.1 B747 Case 3
Alt = 40000*(0.3048); % Altitude [ft] -> [m]
rho = 0.3045; % Density [kg/m^3]
W = 2.83176*10^6; % Weight [N]
Ix = 0.247*10^8; % Moment of Interia x-SA [kg m^2]
Iy = 0.449*10^8; % Moment of Interia y-SA [kg m^2]
Iz = 0.673*10^8; % Moment of Interia z-SA [kg m^2]
Izx = -.212*10^7; % Moment of Interia zx-SA [kg m^2]
CD = .043; % Coefficient of Drag
cbar = 8.324; % Mean Chord Length [m]
S = 511; % Surface Area [m^2]
g = 9.81; % Gravity Constant [m/s^2]
m = W/g; % Mass of Plane [kg]


%% Trim States
Vel = 235.9;% Velocity [m/s]
u0 = Vel; % Initial Velocity in x-coord - Stability Axis Frame (Trim State)
theta0 = 0; % Initial Pitch Angle [deg]
Cw0 = W/(.5*rho*S*u0^2); 
%% Function that Computes Dimensional Derivatives from Non-Dimenional derivatives
[X, Z, M, X_c, Z_c, M_c ] = NonDimLong(rho,u0,S,W,theta0,Cx,Cz,Cm,cbar,C_x_de,C_z_de,C_m_de); 

%% State Variable Matrix A
row1 = [X(1)/m, X(2)/m, 0, -g*cosd(theta0)];
row2 = [Z(1)/(m-Z(4)), Z(2)/(m-Z(4)), (Z(3)+m*u0)/(m-Z(4)), (-W*sind(theta0))/(m-Z(4))];
row3 = [(1/Iy)*(M(1) + ((M(4)*Z(1))/(m-Z(4))) ),...
        (1/Iy)*(M(2) + ((M(4)*Z(2))/(m-Z(4))) ),...
        (1/Iy)*(M(3) + ((M(4)*(Z(3)+m*u0))/(m-Z(4))) ),...
        -((M(4)*W*sind(theta0))/(Iy*(m-Z(4))))];
row4 = [0, 0, 1, 0];

A = [row1;row2;row3;row4];

%% Input Matrix B
% Dimensionalizing Elevator Derivative

% Compenents of B Matrix
row1_C = [X_c(1)/m, X_c(2)/m];
row2_C = [Z_c(1)/(m-Z(4)), Z_c(2)/(m-Z(4))];
row3_C = [M_c(1)/Iy + (M(4)*Z_c(1))/(Iy*(m-Z(4))), M_c(2)/Iy + (M(4)*Z_c(2))...
    /(Iy*(m-Z(4)))];
row4_C = [0, 0];

B = [row1_C;row2_C;row3_C;row4_C];

%% Reduced PWD Model (Phugoid Mode)
k1 = K_mat(1);
k2 = K_mat(2);

B_PWD = [X_c(1)/m - (X(2)*M_c(1))/(m*M(2));
        (M_c(1)*Z(2)-M(2)*Z_c(1))/(m*u0*M(2))];

r11 = (X(1)/m - B_PWD(1)*k1)/(1+B_PWD(1)*k2);
r12 = -g/(1+B_PWD(1)*k2);
r21 = (-Z(1) / (m*u0) - B_PWD(2)*k1) - (B_PWD(2)*k2/(1+B_PWD(1)*k2))...
    *(X(1)/m - B_PWD(1)*k1);
r22 = B_PWD(2)*k2*g/(1+B_PWD(1)*k2);
A_BK = [r11,r12;r21,r22];

I = eye(4); % 4x4 Identity Matrix
k1_mat = [-k1, 0, 0, 0; 0, 0, 0, 0];
k2_mat = [-k2, 0, 0, 0; 0, 0, 0, 0];
A_BK_full = inv(I-B*k2_mat)*(A+B*k1_mat);

end