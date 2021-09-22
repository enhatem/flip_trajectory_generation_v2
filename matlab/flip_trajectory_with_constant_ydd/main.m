clear; close all; clc;

%% Drone parameters and costants

global g t_step y_min y_max z1_min z2_min z3_min z1_max z2_max z3_max m Ixx l u1_max u2_min u2_max

% Drone parameters
m =  29-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
t_step = 0.01; % 100 Hz
g = 9.81; % m/s^2

%% Constraints on the Thrust and Torques that are applied on the opitmization problem
% Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 46.3e-3 * g ) / 2 ); % Maximum thrust 
u2_max = 0.1 * ( 1 / 2 * u1_max * l); % Maximum torque
u2_min = - u2_max;


%% Constaints on y that are applied on the optimization problem
% Bounds on the y trajectory
y_min = -3;
y_max = 0;


%% Constaints on z that are applied on the optimization problem
% Bounds on the z trajectory of the reaching phase
z1_min = 0.8;
z1_max = 3.5;

% Bounds on the z trajectory of the flipping phase
z2_min = 0.8;
z2_max = 3.5;

% Bounds on the z trajectory of the recovery phase
z3_min = 0.8;
z3_max = 3.5;

%% Constraints on phi that are applied on the optimization problem

% lower bound on the initial phi angle at the start of the reaching phase
phi_start_min = 0;

% upper bound on the final phi angle at the end of the reaching phase
phi_start_max = pi/2 - 0.1;

% lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_end_min = 3/2*pi + 0.1;

% upper bound on the final phi angle at the end of the recovery phase
phi_end_max   = 2*pi;

%% Constraints on the time that are applied on the optimization problem
% bounds on t1 (time of the reaching phase trajectory)
t1_min = 0.1;
t1_max = inf;

% bounds on t2 (time of the flip phase trajectory)
t2_min = 0.1;
t2_max = inf;

% bounds on t3 (time of the recovery phase trajectory)
t3_min = 0.1;
t3_max = inf;


%% Constraints on yd (bounds will be determined by the solver based on u1)
yd_min = -inf; 
yd_max =  inf;

%% Constraints on ydd (bounds will be determined by the solver based on u1)
ydd_min = -inf;
ydd_max =  inf;
%% Constraints on zd that are applied to the optimization problem (bounds will be determined by the solver based on u1)
zd_min = -inf;
zd_max = inf;

%% Constraints on zdd that are applied to the optimization problem (bounds will be determined by the solver based on u1)
% zdd_min = -inf;
% zdd_max =  inf;

%% Constraint of phid that are applied to the optimization problem (bounds will be determined by the solver based on u2)
% phid_min = -inf;
% phid_max =  inf;

%% Constraints of phidd that are applied to the optimization problem (bounds will be determined by the solver based on u2)
% phidd_min = -inf;
% phidd_max =  inf;

lb = [z1_min z1_min z2_min z3_min y_min y_min y_min phi_start_min phi_end_min t1_min t2_min t3_min yd_min yd_min zd_min zd_min ydd_min ydd_min];
ub = [z1_max z1_max z2_max z3_max y_max y_max y_max phi_start_max phi_end_max t1_max t2_max t3_max yd_max yd_max zd_max zd_max ydd_max ydd_max];

% nonlinear bounds
nl_con = @nonlinear_bounds;

% objective function
obj = @objective_function;

%% Initial Guess
x0 = [0.81 2.5 2.0 0.81 -0.5 -1.5 -1.9 pi/2-0.1 3*pi/2+0.1 1 0.5 1 1 1 1 1 1 1 1 1];
               
%% Optimization problem
% options  = optimset('Display', 'iter', 'Tolx', 1e-14, 'Tolfun', ...
%                    1e-14, 'MaxIter', 1e20, 'MaxFunEvals', 1e20);
% fmincon optimization               
% x = fmincon(obj,x0,[],[],[],[],lb,ub,nl_con,options);

options = optimoptions(@fmincon,'Algorithm','interior-point');


problem = createOptimProblem('fmincon','x0',x0,'objective',obj,'lb',lb,'ub',ub,'nonlcon',nl_con);
problem.options.MaxIterations=1e50;
problem.options.MaxFunctionEvaluations=1e50;

gs = GlobalSearch('Display','iter');

[x, fg, flg, of] = run(gs, problem)

%% 
% Solution of the optimization problem
z1 = x(1);
z2 = x(2);
z3 = x(3);
z4 = x(4);
y2 = x(5);
y3 = x(6);
y4 = x(7);
phi_start = x(8);
phi_end = x(9);
t1 = round(x(10),2);
t2 = round(x(11),2);
t3 = round(x(12),2);
y2d = x(13);
y3d = x(14);
z2d = x(15);
z3d = x(16);
y2dd = x(17);
y3dd = x(18);

% Building the trajectory
build_trajectory;