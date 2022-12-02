%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Jack Lambert
% Dale Lawrence
% Aircraft Dynmaics Homework 8
% Purpose: Sets Initial Conditions for each Pertubation Case and Calls ODE45
% to plot the State Variables vs time
% Date Modefied: 4/9/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ODE45 Variable Allocation
%                     u_dot = z(1); % x-component of Velocity, Body Frame
%                     z_dot = z(2); % x-component of Velocity, Body Frame
%                     q_dot = z(3); % Angular Velocity about the y-axis [rad/s]
%                     theta_dot = z(4); % Pitch Angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Conditions
c1 = 10; % Delta U: x-comp, BF Interial Velocity [m/s]
c2 = 0; % Delta Theta: Pitch Angle
condition = [c1 c2]; 
%% State Variables vs. Time

% P-control
t = [0 1000]; % Larger times to see phugoid mode, shorter for short period mode
k1 = [-9.0*10^(-4),-8.4*10^(-4)];
K_mat1 = [k1(1), 0]; % Gain Vector for K1_1
K_mat2 = [k1(2), 0]; % Gain Vector for K1_2

[t,z1] = ode45(@(t,y) ODEcall_PWD(t,y,K_mat1),t,condition);
[t,z2] = ode45(@(t,y) ODEcall_PWD(t,y,K_mat2),t,condition);

% Change in U_E vs time
figure
plot(t ,z1(:,1),'Linewidth',1)
hold on 
plot(t ,z2(:,1),'Linewidth',1)
plot(t,0*t,'--k')
tit = sprintf('%s %s %s','\Deltau_E of a B 747 (\Deltau_E = 10 [m/s],');
title(tit)
ylabel('\Deltau_E [m/s]')
xlabel('Time')
legend('K_p = -0.0009','K_p = -0.00084')
hold off

% Change in Theta vs time
figure
plot(t ,z1(:,2),'Linewidth',1)
hold on
plot(t ,z2(:,2),'Linewidth',1)
plot(t,0*t,'--k')
tit = sprintf('%s %s %s','\Delta\theta of a B 747 (\Deltau_E = 10 [m/s],');
title(tit)
ylabel('\Delta\theta [rad]')
xlabel('Time')
legend('K_p = -0.0009','K_p = -0.00084')
hold off

% Change in Elevator Response

de(:,1) = -k1(1)*z1(:,1);
de(:,2) = -k1(2)*z2(:,1);

figure
plot(t, de(:,1),'Linewidth',1)
hold on
plot(t ,de(:,2),'Linewidth',1)
plot(t,0*t,'--k')
tit = sprintf('%s %s %s','\Delta\delta_e of a B 747 (\Deltau_E = 10 [m/s],');
title(tit)
ylabel('\Delta\delta_e [rad]')
xlabel('Time')
legend('K_p = -0.0009','K_p = -0.00084')
hold off

% PD-control

t = [0 100]; % Larger times to see phugoid mode, shorter for short period mode
k1 = [2*10^(-4),1*10^(-4)];
k2 = [0.0269,0.0263];
K_mat1 = [k1(1), k2(1)]; % Gain Vector for K1_1
K_mat2 = [k1(2), k2(2)]; % Gain Vector for K1_2

[t,z1] = ode45(@(t,y) ODEcall_PWD(t,y,K_mat1),t,condition);
[t,z2] = ode45(@(t,y) ODEcall_PWD(t,y,K_mat2),t,condition);

% Change in U_E vs time
figure
plot(t ,z1(:,1),'Linewidth',1)
hold on 
plot(t ,z2(:,1),'Linewidth',1)
plot(t,0*t,'--k')
tit = sprintf('%s %s %s','\Deltau_E of a B 747 (\Deltau_E = 10 [m/s],');
title(tit)
ylabel('\Deltau_E [m/s]')
xlabel('Time')
legend('K_p = 0.0002, K_d = 0.0269 ','K_p = 0.0001, K_d = 0.0263')
hold off

% Change in Theta vs time
figure
plot(t ,z1(:,2),'Linewidth',1)
hold on
plot(t ,z2(:,2),'Linewidth',1)
plot(t,0*t,'--k')
tit = sprintf('%s %s %s','\Delta\theta of a B 747 (\Deltau_E = 10 [m/s],');
title(tit)
ylabel('\Delta\theta [rad]')
xlabel('Time')
legend('K_p = 0.0002, K_d = 0.0269 ','K_p = 0.0001, K_d = 0.0263')
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Full Matrix Response

% Initial Conditions
c1 = 10; % Delta U: x-comp, BF Interial Velocity [m/s]
c2 = 0; % Delta W: z-comp, BF Interial Velocity [m/s]
c3 = 0; % Delta q: y-comp, BF Angular Velocity [rad/s]
c4 = 0; % Delta Theta: Pitch Angle

condition = [c1 c2 c3 c4]; 

% PD Control
t = [0 100]; % Larger times to see phugoid mode, shorter for short period mode
k1 = 1*10^(-4);
k2 = 0.0263;
K_mat = [k1(1), k2(1)]; % Gain Vector for K1_1

% Phugoid Response (Longer Time)

% Calling ODE45 
[t,z] = ode45(@(t,y) ODEcall_Full(t,y,K_mat),t,condition);


% U_E vs time
figure
subplot(4,1,1)
plot(t ,z(:,1),'Linewidth',1)
hold on
plot(t,0*t,'--k')
hold off
tit = sprintf('%s %s','State Variables of a B 747 (\Deltau = 10 [m/s])');
title(tit)
ylabel('\Deltau_E [m/s]')


% W_E vs time
subplot(4,1,2)
plot(t ,z(:,2),'Linewidth',1)
hold on
plot(t,0*t,'--k')
hold off
ylabel('\Deltaw_E [m/s]')

% q vs time
subplot(4,1,3)
plot(t ,z(:,3),'Linewidth',1)
hold on
plot(t,0*t,'--k')
hold off
ylabel('\Deltaq [rad/s]')

% Theta vs time
subplot(4,1,4)
plot(t ,z(:,4),'Linewidth',1)
ylabel('\Delta\theta [rad]')
hold on
plot(t,0*t,'--k')
hold off
xlabel('Time [s]')


% Elevator Response vs time
delta_u_dot = diff(z(:,1))./diff(t); % Differentating delta u
de_full = -k1*z(1:end-1,1) - k2*delta_u_dot; % Accounting for shorter vector
figure
plot(t(1:end-1) ,de_full,'Linewidth',1)
hold on
plot(t,0*t,'--k')
hold off
tit = sprintf('%s %s %s','\Delta\delta_e of a B 747 (\Deltau_E = 10 [m/s],');
title(tit)
ylabel('\Delta\delta_e [rad]')
xlabel('Time [s]')



