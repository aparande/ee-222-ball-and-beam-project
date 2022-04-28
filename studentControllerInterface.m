classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        p_ball_prev = 0;
        theta_prev = 0;
        extra_dummy1 = 0;
        extra_dummy2 = 0;
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function V_servo = stepImpl(obj, t, p_ball, theta)
        % This is the main function called every iteration. You have to implement
        % the controller in this function, bu you are not allowed to
        % change the signature of this function. 
        % Input arguments:
        %   t: current time
        %   p_ball: position of the ball provided by the ball position sensor (m)
        %
        %   theta: servo motor angle provided by the encoder of the motor (rad)
        % Output:
        %   V_servo: voltage to the servo input.
            
            t_prev = obj.t_prev;
            dt = t - t_prev;
            p_ball_prev = obj.p_ball_prev;
            theta_prev = obj.theta_prev;

            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Approximate state derivatives
            p_ball_dot = (p_ball - p_ball_prev)/dt;
            theta_dot = (theta - theta_prev)/dt;
            z = [p_ball; p_ball_dot; theta; theta_dot];

            % Get linear dynamics
            out = lin_dyn(0, 0, 0, 0);
            A_lin = out(1:4, 1:4);
            B_lin = out(1:4, 5);
            % f_affine = nonlin_dyn(p_ball, p_ball_dot, theta, theta_dot) - (A_lin * z);
            
            % Get LQR terms
            %A_lqr = [A_lin, f_affine; 0, 0, 0, 0, 0];
            % B_lqr = [B_lin; 0];
            %Q = diag([100, 1, 15, 1, 0]);
            % R = 1;
            Q = diag([10, 1, .15, .01]) * 100;
            R = 1;
            coder.extrinsic("lqr")
            [K_sol, ~, ~] = lqr(A_lin, B_lin, Q, R);
            K = zeros(1, 4);
            K = K_sol;
            error = [p_ball - p_ball_ref; p_ball_dot - v_ball_ref; theta; theta_dot];
            V_servo = - K * error;

            % Make sure that the desired servo angle does not exceed the physical
            % limit.
            % theta_saturation = 56 * pi / 180;    
            % theta = min(theta_d, theta_saturation);
            % theta = max(theta_d, -theta_saturation);

            % Make sure that the control input does not exceed the physical limit
            V_saturation = 10;    
            V_servo = min(V_servo, V_saturation);
            V_servo = max(V_servo, -V_saturation);
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.p_ball_prev = p_ball;
            obj.theta_prev = theta;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = 40
        end
    end
    
end
