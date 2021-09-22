function traj = trajectory(start,goal,time)
 
    global t_step;

    [A,b] = build_matrix(start,goal,time);
    
    [U,S,V] = svd(A);
    
    A_inv = V * ( S \ U' );
    
    coeff_p = fliplr((A_inv*b)');    
 
    t = 0:t_step:time;
    % t = 0:step:time;
    
    coeff_v = polyder(coeff_p); % coefficients of the velocity polynomial
    coeff_a = polyder(coeff_v); % coefficients of the acceleration polynomial
    % coeff_j = polyder(coeff_a); % coefficients of the jerk polynomial (derivative of the acceleration)
    % coeff_s = polyder(coeff_j); % coefficients of the snap polynomial (derivative of the jerk)
    
    % Resulting trajectory
    traj = [polyval(coeff_p,t);
            polyval(coeff_v,t);
            polyval(coeff_a,t)];
    
end
