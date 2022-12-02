%% Author: Jack Lambert
% ASEN 3128
% Homework 8
% Purpose: This function find the target region for eigenvalues specified
% by a range of time constants and dampening coefficients. This code then
% finds the eigenvalues and mode and computes characterists of the the
% modes. This code then finds optimal proportional gain values that give
% eigenvalues close to the target range
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

row1_PWD = [X(1)/m, -g];
row2_PWD = [-Z(1)/(u0*m), 0];

A_PWD = [row1_PWD; row2_PWD];

B_PWD = [X_c(1)/m - (X(2)*M_c(1))/(m*M(2));
        (M_c(1)*Z(2)-M(2)*Z_c(1))/(m*u0*M(2))];

%% Copmaring the Eigen Values of the A matrix to the A_PWD
[eVA,eValA] = eig(A);
[eV_PWD,eVal_PWD] = eig(A_PWD);

modesA = diag(eValA);
modesA_PWD = diag(eVal_PWD);


max_real = max(abs(real(modesA)));
max_realz_PWD = max(abs(real(modesA_PWD)));

% Short Mode has Larger Real Part
 j = 1;
 k = 1;
for i = 1:length(modesA)
    if abs(real(modesA(i))) == max_real
        SP_Mode(j) = modesA(i); % Short Period Mode
        SP_vec(:,j) = eVA(:,i); % Short Period Eigen Vec 
        j = j+1;
    else
        Phu_Mode(k) = modesA(i); % Phugoid Mode
        Phu_vec(k,:) = eVA(:,i); % Short Period Eigen Vec
        k = k+1;
        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Natural Frequency and Dampeing Ratio

% Phugoid Mode

Wn_PM = ( real(Phu_Mode(1))^2+imag(Phu_Mode(1))^(2) )^(1/2); % Natural Frequency
zeta_PM = -real(Phu_Mode(1))/Wn_PM; % Dampening Coefficient
TimeConst_PM = -1/real(Phu_Mode(1)); % Time Constant (s)

% Short Period Mode

Wn_SP = ( real(SP_Mode(1))^2+imag(SP_Mode(1))^(2) )^(1/2); % Natural Frequency
zeta_SP = -real(SP_Mode(1))/Wn_SP; % Dampening Coefficient
TimeConst_SP = -1/real(SP_Mode(1)); % Time Constant (s)

% PWD 
Wn_PWD = ( real(modesA_PWD(1))^2+imag(modesA_PWD(1))^(2) )^(1/2); % Natural Frequency
zeta_PWD = -real(modesA_PWD(1))/Wn_PWD; % Dampening Coefficient
TimeConst_PWD = -1/real(modesA_PWD(1)); % Time Constant (s)

% Percent Error ( PWD vs. Phugoid)
Wn_error = abs(Wn_PWD - Wn_PM) / (Wn_PM) * 100;
zeta_error = abs(zeta_PWD - zeta_PM) / (zeta_PM) * 100;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting the "Target Zone" of EigenValue
z = 0.9:.001:0.95;
tau = 15:.1:20;
n = (-1./tau);
for i = 1:length(tau)
    for j = 1:length(z)
        w(i,j) = n(i)*(((1/z(j))^2-1))^(1/2);
    end

end

%% Finding The Proportial Gain Needed to reach region
k1 =  0: 0.00001 : 0.01;
% Positive Proportional Gain Values
for i = 1:length(k1)
    K_mat = [k1(i), 0]; % K2 is zero for P control
    A_BK = A_PWD - B_PWD.*K_mat;
    eVal_A_BK = eig(A_BK);
    modesA_BK_pos(i,1) = eVal_A_BK(1);
    modesA_BK_pos(i,2) = eVal_A_BK(2);
end
% Negative Proportional Gain Values
for i = 1:length(k1)
    K_mat = [-k1(i), 0]; % K2 is zero for P control
    A_BK = A_PWD - B_PWD.*K_mat;
    eVal_A_BK = eig(A_BK);
    modesA_BK_neg(i,1) = eVal_A_BK(1);
    modesA_BK_neg(i,2) = eVal_A_BK(2);
        % Finding closest distance if there is Imaginary Values
    if imag(eVal_A_BK(1)) ~=0 || imag(eVal_A_BK(2)) ~=0
        dist1(i) = ( (n(end)-real(modesA_BK_neg(i,1)))^2 +...
            (w(end,1) - imag(modesA_BK_neg(i,1)))^2 )^(1/2);
        dist2(i) = ( (n(end)-real(modesA_BK_neg(i,2)))^2 +...
            (w(end,end) - imag(modesA_BK_neg(i,2)))^2 )^(1/2);
    end
end


% Gains Corresponding to these Points of Interest
ind1 = find(dist1 == min(dist1));
ind2 = find(dist2 == min(dist2));

k_1_1 = k1(ind1);
k_1_2 = k1(ind2);

% Plotting Eigenvalues
figure
plot(real(modesA_BK_pos(:,1)),imag(modesA_BK_pos(:,1)),'.B')
hold on
plot(real(modesA_BK_neg(:,1)),imag(modesA_BK_neg(:,1)),'.R')
plot(real(modesA_BK_pos(:,2)),imag(modesA_BK_pos(:,2)),'.B')
plot(real(modesA_BK_neg(:,2)),imag(modesA_BK_neg(:,2)),'.R')


% Closest Points to Target Region w/ Imag. Parts
plot(real(modesA_BK_neg(ind1,1)),imag(modesA_BK_neg(ind1,1)),'o') 
text(real(modesA_BK_neg(ind1,1)),imag(modesA_BK_neg(ind1,1)),...
    ' \leftarrow Best Gain','VerticalAlignment','baseline')
plot(real(modesA_BK_neg(ind2,1)),imag(modesA_BK_neg(ind2,1)),'o')
text(real(modesA_BK_neg(ind2,1)),imag(modesA_BK_neg(ind2,1)),...
    ' \leftarrow Closest to Range','VerticalAlignment','baseline')
plot(real(modesA_BK_neg(ind1,2)),imag(modesA_BK_neg(ind1,2)),'o') 
plot(real(modesA_BK_neg(ind2,2)),imag(modesA_BK_neg(ind2,2)),'o')

check = 1;
% Plotting Desired Region
% Plotting only the first and last values to show region
plot([n(1),n(1)],[w(1,1),w(1,end)],'Linewidth',2)
hold on
plot([n(end),n(end)],[w(end,1),w(end,end)],'Linewidth',2)
plot(n(:),w(:,1),'Linewidth',2)
plot(n(:),w(:,end),'Linewidth',2)
text(n(1),w(1,30),'  Target Range \rightarrow','HorizontalAlignment','right')

% For negative imaginary eigen values
plot([n(1),n(1)],-[w(1,1),w(1,end)],'Linewidth',2)
hold on
plot([n(end),n(end)],-[w(end,1),w(end,end)],'Linewidth',2)
plot(n(:),-w(:,1),'Linewidth',2)
plot(n(:),-w(:,end),'Linewidth',2)
text(n(1),-w(1,20),'  Target Range \rightarrow','HorizontalAlignment','right')

plot([0,0],[-.22,.22],'--k')
plot([-0.26,0.23],[0,0],'--k')
title('Eigenvalues ')
xlabel('Re(\lambda)')
ylabel('Im(\lambda)')
legend('Positive K_p','Negative K_p')

%% Finding k2 values for eigen values near range


