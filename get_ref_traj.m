function [p_ref, v_ref, a_ref] = get_ref_traj(t)
% [p_ref, v_ref, a_ref] = get_ref_traj(t)
%% Reference trajectory at time t:
%   Inputs
%       t: query time
%   Outputs
%       p_ref: reference position of the ball
%       v_ref: reference velocity of the ball
%       a_ref: reference acceleration of the ball
    amplitude = .15; % m
    period = 6; % sec
    
    omega = 2 * pi / period;    
    do_sine = 1;
    if do_sine
        %% Sine wave.
        p_ref = amplitude * sin(omega * t);
        v_ref = amplitude * omega * cos(omega * t);
        a_ref = - amplitude * omega^2 * sin(omega * t);
    else
        %% Square wave.
        p_ref = amplitude * sign(sin(omega * t));
        v_ref = 0;
        a_ref = 0;
    end


end