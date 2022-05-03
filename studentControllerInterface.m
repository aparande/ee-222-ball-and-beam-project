classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and update while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        p_ball_prev = 0;
        theta_prev = 0;
        extra_dummy1 = 0;
        extra_dummy2 = 0;
        derivatives = zeros(1, 5)
        control_inputs = zeros(1, 3)
        K = cell2mat(struct2cell(load("lqr_gain.mat")));
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
            
            dt = t - obj.t_prev;

            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            
            if(abs(p_ball) > 0.05)
                p_ball_ref = 0;
            end

            % Approximate state derivatives
            p_ball_dot_cur = (p_ball - obj.p_ball_prev)/dt;
            obj.derivatives = [obj.derivatives(2:end), p_ball_dot_cur];
            p_ball_dot = mean(obj.derivatives);
            theta_dot = (theta - obj.theta_prev)/dt;
            z = [p_ball; p_ball_dot; theta; theta_dot];

            % Get linear dynamics
%             out = lin_dyn(0, 0, 0, 0);
%             A_lin = out(1:4, 1:4);
%             B_lin = out(1:4, 5);
            % f_affine = nonlin_dyn(p_ball, p_ball_dot, theta, theta_dot) - (A_lin * z);
            
%             Q = diag([10, 1, .15, .01]) * 100;
%             R = 1;
%             [K, ~, ~] = lqr(A_lin, B_lin, Q, R);
            
            error = [p_ball - p_ball_ref; p_ball_dot - v_ball_ref; theta; theta_dot];
            V_servo_cur = - obj.K * error;
            obj.control_inputs = [obj.control_inputs(2:end), V_servo_cur];
            V_servo = mean(obj.control_inputs);
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
        function [V_servo] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
        end
    end
    
end
