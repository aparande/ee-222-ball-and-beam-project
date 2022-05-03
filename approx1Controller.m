classdef approx1Controller < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        p_ball_prev = 0;
        v_ball_prev = 0;
        theta_prev = 0;
        v_theta_prev = 0;
        
        a_ball_ref_prev = 0;
        
        theta_d = 0;
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function [V_servo, psi] = stepImpl(obj, t, p_ball, theta)
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
            %% Approximate 1 - Feedback Linearization
            % Model parameters
            r_g = 0.0254; % (m)
            L = 0.4255;   % (m)
            g = 9.81;     % (m/s^2)
            K = 1.5;      % (rad/sV)
            tau = 0.025;  % (s)
            % Update previous state values
            t_prev = obj.t_prev;
            p_ball_prev = obj.p_ball_prev;
            v_ball_prev = obj.v_ball_prev;
            theta_prev = obj.theta_prev;
            v_theta_prev = obj.v_theta_prev;
            
            dt = t - t_prev;
            % Calculate ball velocity (x2) and angular velocity (x4)
            v_ball = (p_ball - p_ball_prev) / dt;
            v_theta = (theta - theta_prev) / dt;
            % Extract reference trajectory at the current timestep.
                % y_d, dot(y_d), ddot(y_d)
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            a_ball_ref_prev = obj.a_ball_ref_prev;
                % y_d^(3)
            dot_a_ball_ref = (a_ball_ref - a_ball_ref_prev) / dt;
            
            
            % Approximate normal form
            xi1 = p_ball;
            xi2 = theta;
            xi3 = -5/(14*L)*r_g^2*v_theta^2*cos(theta)^2 + 5/(7*L^2)*r_g^2*p_ball*v_theta^2*cos(theta)^2;
            psi2 = (5*g)/(7*L)*r_g*sin(theta);
            
            a = (10*K)/(7*L^2*tau)*r_g^2*p_ball*v_theta*cos(theta)^2 - (5*K)/(7*L*tau)*r_g^2*v_theta*cos(theta)^2;
            b = (5*r_g^2*cos(theta))/(7*L) * (v_theta^2*cos(theta)/tau + v_ball*v_theta^2*cos(theta)/L + v_theta^3*sin(theta) - 2*p_ball*v_theta^2*cos(theta)^2/(L*tau) - 2*p_ball*v_theta^3*sin(theta)/L);
            
            % Controller design
            alpha_0 = 17;
            alpha_1 = 19;
            alpha_2 = 3;
            % Claire Exact Tracking
%             u = 1/a * (-b + dot_a_ball_ref + alpha_2*(a_ball_ref - xi3) + alpha_1*(v_ball_ref - xi2) + alpha_0*(p_ball_ref - xi1)); 
            % Sastry Tracking
            u = 1/a * (-b - alpha_2*xi3 - alpha_1*xi2 - alpha_0*xi1);
            
            % V saturation
            K_v = 0.001;
            V_sat = 10;
            V_servo = K_v * u;
            V_servo = max(V_servo, -V_sat);
            V_servo = min(V_servo, V_sat);
            
            % Neglected nonlinearity psi
            psi = psi2;
            
            % Update class properties.
            obj.t_prev = t;
            obj.p_ball_prev = p_ball;
            obj.v_ball_prev = v_ball;
            obj.theta_prev = theta;
            obj.v_theta_prev = v_theta;
            
            obj.a_ball_ref_prev = a_ball_ref;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want. 
        % (KAI) Neglected nonlinearity - psi added
        function [V_servo, theta_d, psi] = stepController(obj, t, p_ball, theta)        
            [V_servo, psi] = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
    
end
