function [p_ball_est, v_ball_est, v_theta_est] = kalman_filter(t, u)
    % Model parameters
    r_g = 0.0254; % (m)
    L = 0.4255;   % (m)
    g = 9.81;     % (m/s^2)
    K = 1.5;      % (rad/sV)
    tau = 0.025;  % (s)
    
    % Filter Design
    A = [0 1 0 0
         0 0 1 0
         0 0 0 1
         0 0 0 0];
    B = [0; 0; 0; 1];
    C = [1; 0; 0; 0];
    D = 0;
    
    % Plant Model
    Ts = -1;
    sys = ss(A, [B B], C, D, Ts, 'InputName', {'u', 'w'}, 'OutputName', 'y');
    Q = 2.3;
    R = 1;
    
    % Kalman Filter
    [kalmf, L, ~, Mx, Z] = kalman(sys, Q, R);
    
    % Output analysis
    y_est = kalmf(1,:);
    x_est = kalmf(2,:);
    xi1_est = x_est(1);
    xi2_est = x_est(2);
    xi3_est = x_est(3);
    xi4_est = x_est(4);
    
    p_ball_est = mean([y_est xi1_est]);
    v_ball_est = xi2_est;
    v_theta_est_1 = asin((7*L)/(5*g*r_g) * xi3_est);
    v_theta_est_2 = acos((7*L)/(5*g*r_g) * xi4_est);
    v_theta_est = mean([v_theta_est_1 v_theta_est_2]);

end
