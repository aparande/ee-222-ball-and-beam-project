classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
		theta_prev = 0;
        pos_prev = 0;
        vel_prev = 0;
        
        theta_d = 0;
        extra_dummy1 = 0;
        extra_dummy2 = 0;
        virtual_u = 0;

		% System Parameters
		r_arm = 0.0254;
		L = 0.4255;
		g = 9.81;
		K = 1.5;
		tau = 0.025;
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
            theta_saturation = 56 * pi / 180;
            k_p = 2;
            k_v = 1;
            k_u = 2;

            a = 5 * obj.g * obj.r_arm / (7 * obj.L);
            b = (5 * obj.L / 14) * (obj.r_arm / obj.L)^2;
            c = (5 / 7) * (obj.r_arm / obj.L)^2;

            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            vel = (p_ball - obj.pos_prev) / 0.01;
            dtheta = (theta - obj.theta_prev) / 0.01;

		    u_d = 1 ./ (b - c .* p_ball) .* (a .* sin(theta) + k_v * (vel - v_ball_ref) + k_p .* (p_ball - p_ball_ref));
            z = obj.tau / ( 2 * obj.K .* cos(theta) .^ 2);

            if dtheta < 1
                dtheta = 1;
            end

            z = z ./ dtheta;
            
            u_tilde = (dtheta .* cos(theta)) .^ 2;

		    u =  z * (2 / obj.tau * u_tilde + dtheta .^ 3 .* sin(2 .* theta) - k_u .* (u_d - u_tilde));
            if abs(u) < 10
                V_servo = u;
            else
                V_servo = sign(u) * 10;
            end
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.theta_d = 0;
			obj.theta_prev = theta;
            obj.pos_prev = p_ball;
            obj.vel_prev = vel;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
    
end
