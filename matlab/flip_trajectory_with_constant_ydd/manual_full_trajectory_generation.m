clear; close all; clc;

%% Drone parameters and costants

global t_step g l Ixx m u1_max u1_min u2_min u2_max ;

% Drone parameters
m =  32e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length
u1_max = 0.9* (46.3e-3 * g) /2;
u1_min = 0.1 * u1_max;
u2_max = 0.1 * (0.5 * u1_max * l);
u2_min = - u2_max;

% Constants
t_step =0.01;
g = 9.81; % m/s^2



%% Trajectory parameters (to be optimized)

% For the time for each trajectory
t1 = 0.3;
t2 = 0.2;
t3 = 0.5;


% For z
z1 = 1;

z2 = 1.5;
z2d = (z2-z1)/t1; % determined by the solver

z3 = 1.3;
z3d = (z3-z2)/t2; % determined by the solver

z4 = 1;

% For y
y1 = 0;

y2 = -0.5;
% y2d = (y2 - y1) / t1; % determined by the solver
% y2dd = - u1_min / m;

y3 = -1;
% y3d = (y3 - y2) / t2; % determined by the solver
% y3dd = - u1_min / m;

y4 = -1.5;



% For phi
phi_start = pi/2  - 0.2;
phi_end = (3/2)* pi + 0.2;

phid_start = (phi_start - 0)/t1; 
phidd_start =  0.1 * (0.5 * u1_min * l) / Ixx;

phid_end = (phi_end - phi_start) / t2; 
phidd_end =  0.1 * (0.5 * u1_min * l) / Ixx;


%% Trajectory waypoints for the z trajectory

% For y
a = u1_min/m; % 0.1*(0.5*u1_min*l)/m;
Y1 = [y1 0 0 0 0];
Y2 = [y2 ((y3-y2)/t2+a*t2/2) -a 0 0]; % yd2 and ydd2 to be found by the solver
Y3 = [y3 ((y3-y2)/t2-a*t2/2) -a 0 0]; % yd3 and ydd3 to be found by the solver
% Y2 = [y2 y2d y2dd 0 0];
% Y3 = [y3 y3d y3dd 0 0];
Y4 = [y4 0 0 0 0];

% For z
Z1 = [z1 0 0 0 0]; % initial position (start of the reaching phase)
Z2 = [z2 ((z3-z2)/t2 + g*t2/2) -g 0 0];
Z3 = [z3 ((z3-z2)/t2 - g*t2/2) -g 0 0];
% Z2 = [z2 z2d -g 0 0];
% Z3 = [z3 z3d -g 0 0];
Z4 = [z4 0 0 0 0];




% For phi
PHI1 = [0 0 0 0 0];
PHI2 = [phi_start (phi_end - phi_start)/t2 0 0 0]; % phi2d and phi2dd will be determined by the solver
PHI3 = [phi_end (phi_end - phi_start)/t2 0 0 0];   % phi3d and phi3dd will be determined by the solver
PHI4 = [2*pi 0 0 0 0];


%% Reaching phase

% For y
traj1_y = trajectory(Y1,Y2,t1);

% For z
traj1_z = trajectory(Z1,Z2,t1);

% For phi
traj1_phi = trajectory(PHI1,PHI2,t1);
T1 = 0:t_step:t1;

%% Flip phase

% For y
% traj2_y = trajectory(Y2,Y3,t2);

coeff_y2 = [-a/2 ((y3-y2)/t2+a*t2/2) y2];
coeff_yd2 = polyder(coeff_y2);
coeff_ydd2 = polyder(coeff_yd2);
traj_y2   = polyval(coeff_y2,0:t_step:t2);
traj_yd2  = polyval(coeff_yd2, 0:t_step:t2); 
traj_ydd2 = polyval(coeff_ydd2,0:t_step:t2);

traj2_y =[traj_y2;
          traj_yd2;
          traj_ydd2];

% For z
coeff_z2 = [-g/2 ((z3-z2)/t2+g*t2/2) z2];
coeff_zd2 = polyder(coeff_z2);
coeff_zdd2 = polyder(coeff_zd2);

traj_z2   = polyval(coeff_z2,0:t_step:t2);
traj_zd2  = polyval(coeff_zd2, 0:t_step:t2); 
traj_zdd2 = polyval(coeff_zdd2,0:t_step:t2);

traj2_z =[traj_z2;
          traj_zd2;
          traj_zdd2];
%traj2_z = trajectory(Z2,Z3,t2); 


% For phi
coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
coeff_phid2 = polyder(coeff_phi2);
coeff_phidd2 = polyder(coeff_phid2);
phi2 = polyval(coeff_phi2,0:t_step:t2);
phid2 = polyval(coeff_phid2,0:t_step:t2);
phidd2 = polyval(coeff_phidd2,0:t_step:t2);
traj2_phi = [phi2;
             phid2;
             phidd2];
      
      
T2 = t1:t_step:t1+t2;

%%
% Recovery phase

% For y
traj3_y = trajectory(Y3,Y4,t3);

% For z
traj3_z = trajectory(Z3,Z4,t3);

% For phi
traj3_phi = trajectory(PHI3,PHI4,t3);

T3 = t1+t2:t_step:t1+t2+t3;


%% Plotting y, z and phi by parts

% Trajectory along y
figure, plot(T1,traj1_y(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_y(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_y(1,:),'LineWidth',1.5), hold off

title('Trajectory of y(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('y[m]')

%% Trajectory along ydd
figure, plot(T1,traj1_y(3,:),'LineWidth',1.5), hold on
plot(T2,traj2_y(3,:),'LineWidth',1.5), hold on
plot(T3,traj3_y(3,:),'LineWidth',1.5), hold off

title('Trajectory of ydd(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('ydd[m/s^2]')


%%
% Trajectory along z
figure, plot(T1,traj1_z(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_z(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_z(1,:),'LineWidth',1.5), hold off

title('Trajectory of z(t) with Polynomials of degree 5')
xlabel('time [s]')
ylabel('z[m]')
%%
% Trajectory along zdd
figure, plot(T1,traj1_z(3,:),'LineWidth',1.5), hold on
plot(T2,traj2_z(3,:),'LineWidth',1.5), hold on
plot(T3,traj3_z(3,:),'LineWidth',1.5), hold off

title('Trajectory of zdd(t) with Polynomials of degree 9')
xlabel('time [s]')
ylabel('zdd[m/s^2]')
%%
% Trajectory along phi 
figure, plot(T1,traj1_phi(1,:),'LineWidth',1.5), hold on
plot(T2,traj2_phi(1,:),'LineWidth',1.5), hold on
plot(T3,traj3_phi(1,:),'LineWidth',1.5), hold off

title('Trajectory of phi(t) with Polynomials of degree 9')
xlabel('time [s]')
ylabel('phi[rad]')

%%
% Trajectory along phidd
figure, plot(T1,traj1_phi(3,:),'LineWidth',1.5), hold on
plot(T2,traj2_phi(3,:),'LineWidth',1.5), hold on
plot(T3,traj3_phi(3,:),'LineWidth',1.5), hold off

title('Trajectory of phidd(t) with Polynomials of degree 9')
xlabel('time [s]')
ylabel('phidd[rad/s^2]')

%% removing the identical points

traj2_y = traj2_y(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_y = traj3_y(:,2:end); % removes the common point between the 2 consecutive trajectories

traj2_z = traj2_z(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_z = traj3_z(:,2:end); % removes the common point between the 2 consecutive trajectories

traj2_phi = traj2_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
traj3_phi = traj3_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
%%
z = [traj1_z(1,:) traj2_z(1,:) traj3_z(1,:)];
phi = [traj1_phi(1,:) traj2_phi(1,:) traj3_phi(1,:)];

T = 0:t_step:t1+t2+t3;

% figure, plot(T,z)

% figure, plot(T,phi)

%% Calculating the thrust u1 along the trajectory

% zdd, ydd and phidd for the full trajectory
ydd = [traj1_y(3,:) traj2_y(3,:) traj3_y(3,:)];
zdd = [traj1_z(3,:) traj2_z(3,:) traj3_z(3,:)];
phidd = [traj1_phi(3,:) traj2_phi(3,:) traj3_phi(3,:)];

% zdd and ydd for the reaching phase
ydd1 = traj1_y(3,:);
zdd1 = traj1_z(3,:);
phi1 = traj1_phi(1,:);

% zdd and ydd for the flip phase
ydd2 = traj2_y(3,:);
zdd2 = traj2_z(3,:);
phi2 = traj2_phi(1,:);

% zdd and ydd for the recovery phase
ydd3 = traj3_y(3,:);
zdd3 = traj3_z(3,:);
phi3 = traj3_phi(1,:);


gravity = g*ones(size(z));

% Full u1 calculated from zdd
%u1_zdd = m*(zdd+gravity)./cos(phi);

% u1 in the reaching phase
%u1_1_zdd = m*(zdd1 + g*ones(size(zdd1)))./ cos(phi1);
u1_1_combined = m * (-ydd1 + zdd1 + g*ones(size(ydd1))) ./ (sin(phi1) + cos(phi1));

% u1_1_ydd = -m* ydd1 ./sin(phi1); NOT GOOD to determine u1 with ydd in the
% reaching phase

%u1_2_zdd = m*(zdd2 + g*ones(size(zdd2)))./ cos(phi2);
%u1_2_ydd = -m* ydd2 ./sin(phi2);
u1_2_combined = m * (-ydd2 + zdd2 + g*ones(size(ydd2))) ./ (sin(phi2) + cos(phi2));

u1_3_combined =  m * (-ydd3 + zdd3 + g*ones(size(ydd3))) ./ (sin(phi3) + cos(phi3));

% u1 from zdd
%figure, plot(T1,u1_1_zdd), hold on
%plot(T2, u1_2_zdd), hold on

u1= m * (-ydd + zdd + g*ones(size(ydd))) ./ (sin(phi) + cos(phi));

%% u1 from zdd and ydd
%figure,plot(T1,u1_1_combined), hold on
%plot(T2, u1_2_combined), hold on
%plot(T3, u1_3_combined), hold off

%u1 = [u1_1_combined u1_2_combined u1_3_combined];
figure, plot(T,u1,'LineWidth',1.5)
title('Thrust input u1')
xlabel('time [s]')
ylabel('u1[N]')
%% u2 
u2 = phidd * Ixx;
figure, plot(T,u2,'LineWidth',1.5)
title('Torque input u2')
xlabel('time [s]')
ylabel('u2[N.m]')

%% Calculating the trajectory along y

y = [traj1_y(1,:) traj2_y(1,:) traj3_y(1,:)];

figure, plot(T,y,'LineWidth',1.5)
title('Trajectory of y(t)')
xlabel('time[s]')
ylabel('y[m]')

figure, plot(y,z,'LineWidth',1.5)
title('Planar Trajectory')
xlabel('y[m]')
ylabel('z[m]')