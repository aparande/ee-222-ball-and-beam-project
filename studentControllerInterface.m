classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
		theta_prev = -1;
        pos_prev = -1;
        vel_prev = -1;
        
        theta_d = 0;
        extra_dummy1 = 0;
        extra_dummy2 = 0;
        virtual_u = 0;
        dtheta = 0;

		% System Parameters
		r_arm = 0.0254;
		L = 0.4255;
		g = 9.81;
		K = 10;
		tau = 0.025;

        % Controller Parameters
        k_p = 2;
        k_v = 1;
        k_u = 2;
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

            a = 5 * obj.g * obj.r_arm / (7 * obj.L);
            b = (5 * obj.L / 14) * (obj.r_arm / obj.L)^2;
            c = (5 / 7) * (obj.r_arm / obj.L)^2;

            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Account for IC in finite differencing
            if obj.t_prev < 0
                vel = 0;
                dtheta = 0;
            else
                vel = (p_ball - obj.pos_prev) / (t - obj.t_prev);
                dtheta = (theta - obj.theta_prev) / (t - obj.t_prev);
                obj.dtheta = dtheta;
            end

            u_tilde = (dtheta .* cos(theta)) .^ 2;

		    u_d = obj.optimal_virtual_u(p_ball, p_ball_ref, vel, v_ball_ref, theta);
            z = obj.tau / ( 2 * obj.K .* cos(theta) .^ 2);

            if dtheta < 1
                dtheta = 1;
            end

            z = z ./ dtheta;

		    u =  z * (2 / obj.tau * u_tilde + (dtheta .^ 3) .* sin(2 .* theta) - obj.k_u .* (u_d - u_tilde));
            %u = 1 ./ (2 .* obj.K) * dtheta + obj.tau * (dtheta .^ 2) * sin(2 .* theta) / (2 .* obj.K .* (cos(theta) .^ 2)) + z * (- obj.k_u .* (u_d - u_tilde));

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
            obj.virtual_u = u_d;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d, virtual_u, vel, dtheta] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            virtual_u = obj.virtual_u;
            vel = obj.vel_prev;
            dtheta = obj.dtheta;
        end

        function virtual_u = optimal_virtual_u(obj, p_ball, p_ball_ref, v_ball, v_ball_ref, theta)
            a = 5 * obj.g * obj.r_arm / (7 * obj.L);
            b = (5 * obj.L / 14) * (obj.r_arm / obj.L)^2;
            c = (5 / 7) * (obj.r_arm / obj.L)^2;

            virtual_u = 1 ./ (b - c .* p_ball) .* (a .* sin(theta) + obj.k_v .* (v_ball - v_ball_ref) + obj.k_p .* (p_ball - p_ball_ref));
        end
    end
    
end
