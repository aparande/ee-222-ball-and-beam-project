classdef studentControllerInterface < matlab.System
  properties (Constant)
        g = 9.81;
        r_arm = 0.0254;
        L = 0.4255;
        a = 5 * studentControllerInterface.g * studentControllerInterface.r_arm / (7 * studentControllerInterface.L);
        b = (5 * studentControllerInterface.L / 14) * (studentControllerInterface.r_arm / studentControllerInterface.L)^2;
        c = (5 / 7) * (studentControllerInterface.r_arm / studentControllerInterface.L)^2;
   end
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        theta_d = 0;
        t_prev = -1;
        p_ball_prev = 0;
        theta_prev = 0;
        extra_dummy1 = 0;
        extra_dummy2 = 0;
        positions = zeros(1, 5);
        control_inputs = zeros(1, 5);
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function V_servo = stepImpl(obj, t, p_ball_cur, theta)
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
            if abs(p_ball_cur) > .13
                p_ball_ref = sign(p_ball_ref) * .1;
                v_ball_ref = 0;
            end

            obj.positions = [obj.positions(2:end), p_ball_cur];
            p_ball = mean(obj.positions);

            % Approximate state derivatives
            p_ball_dot_cur = (p_ball - obj.p_ball_prev)/dt;
            if t < .1
                p_ball_dot_cur = 0;
            end
            p_ball_dot = p_ball_dot_cur;
            theta_dot = (theta - obj.theta_prev)/dt;
            z = [p_ball; p_ball_dot; theta; theta_dot];
            
            % error = [p_ball - p_ball_ref; p_ball_dot - v_ball_ref; theta; theta_dot];
            p_ball_error = p_ball - p_ball_ref;
            p_ball_dot_error = p_ball_dot - v_ball_ref;
            Kp = 5.0;
            Kd = 3;
            V_servo_cur = obj.solveu(p_ball, theta, theta_dot, -Kp * p_ball_error -Kd * p_ball_dot_error);
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
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end

        function V_servo = solveu(obj, p_ball, theta, theta_dot, v)
            fx = obj.a * sin(theta);
            gx = - obj.b * cos(theta)^2 + obj.c * p_ball * cos(theta)^2;
            x4_2 = (v - fx)/gx;
%            if x4_2 >= 0
            if 1 == 0
%                 disp("nay")
                x4 = - sign(theta + 1e-3) * sqrt(x4_2);
                k_servo = .01;
                V_servo = k_servo * (x4 - theta_dot);
                obj.theta_d = 0;
           else
%                 disp("hey")
                sin_val = v/obj.a;
                max_sin_val = .75;
                sin_val = min(sin_val, max_sin_val);
                sin_val = max(sin_val, -max_sin_val);
                obj.theta_d = asin(sin_val);
%                 disp(theta_d)
% %                 disp(obj.t_prev)
                k_servo = 4;
                V_servo = k_servo * (obj.theta_d - theta) - .2 * theta_dot;
            end
        end
    end
    
end
