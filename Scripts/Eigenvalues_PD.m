%% Author: Jack Lambert
% ASEN 3129
% Homework 8
% Purpose: This function find the target region for eigenvalues specified
% by a range of time constants and dampening coefficients. This code then
% finds the eigenvalues and mode and computes characterists of the the
% modes. This code then finds optimal proportional and derivative gain
% values that give eigenvalues with the largest negative real parts inside
% the specified target range
close all; 
clear all;
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
%% Defining the Target Region
z = 0.9:.001:0.95;
tau = 15:.1:20;
n = (-1./tau);
for i = 1:length(tau)
    for j = 1:length(z)
        w(i,j) = n(i)*(((1/z(j))^2-1))^(1/2);
    end

end
%% Finding The  Gain Combination Needed to reach region
k1 =  -0.0005 : 0.0001 : 0.0005;
k2 = 0 : 0.0001 : 0.03;
% Reduced PWD Model ( B Matrix )
B_PWD = [X_c(1)/m - (X(2)*M_c(1))/(m*M(2));
            (M_c(1)*Z(2)-M(2)*Z_c(1))/(m*u0*M(2))];
% Positive Proportional Gain Values
for i = 1:length(k1)
    for j = 1:length(k2)
        % Reduced PWD Model (A-B*K) 
        r11 = (X(1)/m - B_PWD(1)*k1(i))/(1+B_PWD(1)*k2(j));
        r12 = -g/(1+B_PWD(1)*k2(j));
        r21 = (-Z(1) / (m*u0) - B_PWD(2)*k1(i)) - (B_PWD(2)*k2(j)/(1+B_PWD(1)*k2(j)))...
            *(X(1)/m - B_PWD(1)*k1(i));
        r22 = B_PWD(2)*k2(j)*g/(1+B_PWD(1)*k2(j));
        A_BK = [r11,r12;r21,r22];
        % Eigenvalues
        eVal_A_BK = eig(A_BK);
        modesA_BK_1(i,j) = eVal_A_BK(1);
        modesA_BK_2(i,j) = eVal_A_BK(2);
    end

end

% Graphically finding point w/ large negative real parts
dist1 = abs(real(modesA_BK_1(8,:))+0.06647);
ind1 =find(dist1 == min(dist1));

dist2 = abs(real(modesA_BK_1(7,:))+0.06512);
ind2 =find(dist2 == min(dist2));

% Gains Corresponding to these Values

k1_1 = k1(8);
k2_1 = k2(ind1);

k1_2 = k1(7);
k2_2 = k2(ind2);

% Plotting Eigenvalues
figure
plot(real(modesA_BK_1),imag(modesA_BK_1),'.B')
hold on
plot(real(modesA_BK_2),imag(modesA_BK_2),'.B')

% Closest Point to largest real (left corner of target region)
plot(real(modesA_BK_1(8,ind1)),imag(modesA_BK_1(8,ind1)),'o')
text(real(modesA_BK_1(8,ind1)),imag(modesA_BK_1(8,ind1)),...
    ' \leftarrow Best Gain','VerticalAlignment','baseline')
plot(real(modesA_BK_1(7,ind2)),imag(modesA_BK_1(7,ind2)),'o')
text(real(modesA_BK_1(7,ind2)),imag(modesA_BK_1(7,ind2)),...
    ' \leftarrow Best Gain','VerticalAlignment','baseline')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting the "Target Zone" of EigenValue

% Plotting only the first and last values to show region
plot([n(1),n(1)],[w(1,1),w(1,end)],'Linewidth',2)
plot([n(end),n(end)],[w(end,1),w(end,end)],'Linewidth',2)
plot(n(:),w(:,1),'Linewidth',2)
plot(n(:),w(:,end),'Linewidth',2)
text(n(1),w(1,25),'  Target Range \rightarrow','HorizontalAlignment','right')

% For negative imaginary eigen values
plot([n(1),n(1)],-[w(1,1),w(1,end)],'Linewidth',2)
plot([n(end),n(end)],-[w(end,1),w(end,end)],'Linewidth',2)
plot(n(:),-w(:,1),'Linewidth',2)
plot(n(:),-w(:,end),'Linewidth',2)
text(n(1),-w(1,25),'  Target Range \rightarrow','HorizontalAlignment','right')

plot([0,0],[-.22,.22],'--k')
plot([-0.26,0.23],[0,0],'--k')
title('Eigenvalues ')
xlabel('Re(\lambda)')
ylabel('Im(\lambda)')


%% Full Matrix Comparison
% Chosen Gains
k1 = 0.0001;
k2 = 0.0263;
% State Variable Matrix A
row1 = [X(1)/m, X(2)/m, 0, -g*cosd(theta0)];
row2 = [Z(1)/(m-Z(4)), Z(2)/(m-Z(4)), (Z(3)+m*u0)/(m-Z(4)), (-W*sind(theta0))/(m-Z(4))];
row3 = [(1/Iy)*(M(1) + ((M(4)*Z(1))/(m-Z(4))) ),...
        (1/Iy)*(M(2) + ((M(4)*Z(2))/(m-Z(4))) ),...
        (1/Iy)*(M(3) + ((M(4)*(Z(3)+m*u0))/(m-Z(4))) ),...
        -((M(4)*W*sind(theta0))/(Iy*(m-Z(4))))];
row4 = [0, 0, 1, 0];

A = [row1;row2;row3;row4];

% Input Matrix B
% Dimensionalizing Elevator Derivative

% Compenents of B Matrix
row1_C = [X_c(1)/m, X_c(2)/m];
row2_C = [Z_c(1)/(m-Z(4)), Z_c(2)/(m-Z(4))];
row3_C = [M_c(1)/Iy + (M(4)*Z_c(1))/(Iy*(m-Z(4))), M_c(2)/Iy + (M(4)*Z_c(2))...
    /(Iy*(m-Z(4)))];
row4_C = [0, 0];

B = [row1_C;row2_C;row3_C;row4_C];

I = eye(4); % 4x4 Identity Matrix
k1_mat = [-k1, 0, 0, 0; 0, 0, 0, 0];
k2_mat = [-k2, 0, 0, 0; 0, 0, 0, 0];
A_BK_full = inv(A+B*k1_mat)*(I-B*k2_mat);

%% PD Closed-Loop Matrix

% Reduced PWD Model (A-B*K) 
r11 = (X(1)/m - B_PWD(1)*k1)/(1+B_PWD(1)*k2);
r12 = -g/(1+B_PWD(1)*k2);
r21 = (-Z(1) / (m*u0) - B_PWD(2)*k1) - (B_PWD(2)*k2/(1+B_PWD(1)*k2))...
    *(X(1)/m - B_PWD(1)*k1);
r22 = B_PWD(2)*k2*g/(1+B_PWD(1)*k2);
A_BK = [r11,r12;r21,r22];

% Copmaring the Eigen Values of the A matrix to the A_PWD
modesA_full = eig(A_BK_full);
modesA_PD = eig(A_BK);

max_real = max(abs(real(modesA_full)));
max_realz_PWD = max(abs(real(modesA_PD)));

% Short Mode has Larger Real Part
 j = 1;
 k = 1;
for i = 1:length(modesA_full)
    if abs(real(modesA_full(i))) == max_real
        SP_Mode_full(j) = modesA_full(i); % Short Period Mode
   
        j = j+1;
    else
        Phu_Mode_full(k) = modesA_full(i) % Phugoid Mode
        k = k+1;
        
    end
end
