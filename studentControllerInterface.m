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
        virtual_u = 0;
        dtheta = 0;
        u_unapprox = 0;

		% System Parameters
		r_arm = 0.0254;
		L = 0.4255;
		g = 9.81;
		K = 10;
		tau = 0.025;

        % Controller Parameters
        k_p_agg = 4;
        k_v_agg = 2;

        k_p_rel = 1;
        k_v_rel = 3.6;
        
        k_u_max = 1;
        k_u_min = 0.0001;
        virtual_control_clamp = 1000000;
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
            if abs(u_d) > obj.virtual_control_clamp
                u_d = sign(u_d) * obj.virtual_control_clamp;
            end

            z = obj.tau / ( 2 * obj.K .* cos(theta) .^ 2);

            if dtheta < 1
                dtheta = 1;
            end

            z = z ./ dtheta;

            % u = 1 / obj.K * (1 + obj.k_u_max * obj.tau / 2) .* dtheta + obj.tau / (2 .* obj.K) * (dtheta .^ 2) * sin(2 .* theta) / (cos(theta) .^ 2) - obj.k_u_max * z .* u_d;
            % obj.u_unapprox = 1 / obj.K * (1 + obj.k_u_max * obj.tau / 2) .* dtheta + obj.tau / (2 .* obj.K) * (dtheta .^ 2) * sin(2 .* theta) / (cos(theta) .^ 2) - obj.k_u_max * z .* u_d / dtheta;

            if abs(p_ball - p_ball_ref) < 0.1
                %"Unrestraining Controller"
	            u =  z * (2 / obj.tau * u_tilde + (dtheta .^ 3) .* sin(2 .* theta) - obj.k_u_max .* (u_d - u_tilde));
            else
                %"Restraining Controller"
                u =  z * (2 / obj.tau * u_tilde + (dtheta .^ 3) .* sin(2 .* theta) - obj.k_u_min .* (u_d - u_tilde));
            end

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
        function [V_servo, theta_d, virtual_u, vel, dtheta, u_unapprox] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            virtual_u = obj.virtual_u;
            vel = obj.vel_prev;
            dtheta = obj.dtheta;
            u_unapprox = obj.u_unapprox;
        end

        function virtual_u = optimal_virtual_u(obj, p_ball, p_ball_ref, v_ball, v_ball_ref, theta)
            a = 5 * obj.g * obj.r_arm / (7 * obj.L);
            b = (5 * obj.L / 14) * (obj.r_arm / obj.L)^2;
            c = (5 / 7) * (obj.r_arm / obj.L)^2;

            if abs(v_ball - v_ball_ref) >= 0.1
                k_p = obj.k_p_rel;
                k_v = obj.k_v_rel;
            else
                k_p = obj.k_p_agg;
                k_v = obj.k_v_agg;
            end

            virtual_u = 1 ./ (b - c .* p_ball) .* (a .* sin(theta) + k_v .* (v_ball - v_ball_ref) + k_p .* (p_ball - p_ball_ref));
        end
    end
    
end
