function [c,ceq] = nonlinear_bounds(x)
%nonlinear_bounds extracts the nonlinear bounds that will be used in the
%optimization problem.

    global g t_step m u1_min u1_max u2_min u2_max Ixx z1_min z1_max

    ceq = []; % no equality constraints

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
    
    %% Trajectory waypoints for the y, z and phi trajectories
    % For y
    a = u1_min/m; % 0.1*(0.5*u1_min*l)/m;
    Y1 = [0 0 0 0 0];
    % Y2 = [y2 ((y3-y2)/t2+a*t2/2) -a 0 0]; 
    % Y3 = [y3 ((y3-y2)/t2-a*t2/2) -a 0 0]; 
    % Y2 = [y2 y2d -a 0 0];
    % Y3 = [y3 y3d -a 0 0];
    Y2 = [y2 y2d y2dd 0 0];
    Y3 = [y3 y3d y3dd 0 0];
    Y4 = [y4 0 0 0 0];

    % For z
    Z1 = [z1 0 0 0 0]; % initial position (start of the reaching phase)
    % Z2 = [z2 ((z3-z2)/t2 + g*t2/2) -g 0 0];
    % Z3 = [z3 ((z3-z2)/t2 - g*t2/2) -g 0 0];
    Z2 = [z2 z2d -g 0 0];
    Z3 = [z3 z3d -g 0 0];
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
    % T1 = 0:t_step:t1;
    
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


    % T2 = t1:t_step:t1+t2;
    
    
    %% Recovery phase

    % For y
    traj3_y = trajectory(Y3,Y4,t3);

    % For z
    traj3_z = trajectory(Z3,Z4,t3);

    % For phi
    traj3_phi = trajectory(PHI3,PHI4,t3);

    % T3 = t1+t2:t_step:t1+t2+t3;
    
    %% removing the identical points

    traj2_y = traj2_y(:,2:end); % removes the common point between the 2 consecutive trajectories
    traj3_y = traj3_y(:,2:end); % removes the common point between the 2 consecutive trajectories

    traj2_z = traj2_z(:,2:end); % removes the common point between the 2 consecutive trajectories
    traj3_z = traj3_z(:,2:end); % removes the common point between the 2 consecutive trajectories

    traj2_phi = traj2_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
    traj3_phi = traj3_phi(:,2:end); % removes the common point between the 2 consecutive trajectories
    
    %% Extracting z and phi
    % z = [traj1_z(1,:) traj2_z(1,:) traj3_z(1,:)];
    phi = [traj1_phi(1,:) traj2_phi(1,:) traj3_phi(1,:)];

    % T = 0:t_step:t1+t2+t3;
    
    %% Calculating the thrust u1 and the torque u2 along the trajectory

    % Extracting zdd, ydd and phidd of the trajectory
    ydd = [traj1_y(3,:) traj2_y(3,:) traj3_y(3,:)];
    zdd = [traj1_z(3,:) traj2_z(3,:) traj3_z(3,:)];
    phidd = [traj1_phi(3,:) traj2_phi(3,:) traj3_phi(3,:)];
    gravity = g*ones(size(zdd));
    
    u1= m * (-ydd + zdd + gravity) ./ (sin(phi) + cos(phi));    
    u2 = phidd * Ixx;
    
    %% Inequality Constraints
    c = [   z1 - z2;
            z3 - z2;
            z4 - z3;
            z1_min - min(traj1_z(1,:));
            z1_min - min(traj2_z(1,:));
            z1_min - min(traj3_z(1,:));
            max(traj1_z(1,:)) - z1_max;
            max(traj2_z(1,:)) - z1_max;
            max(traj3_z(1,:)) - z1_max;
            -0.2 - min(traj1_phi(1,:));
            -0.2 - min(traj2_phi(1,:));
            -0.2 - min(traj3_phi(1,:));
            max(traj1_phi(1,:)) - 0.2;
            max(traj2_phi(1,:)) - 0.2;
            max(traj3_phi(1,:)) - 0.2;
            y2 - 0;
            y3 - y2;
            y4 - y3;
            u1_min-min(u1);          % lower bound on u1 (u1>=0)
            max(u1)-u1_max;    % upper bound on u1 (u1<=u1_max)
            u2_min-min(u2);
            max(u2)-u2_max ];




end