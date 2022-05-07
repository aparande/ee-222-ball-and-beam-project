classdef studentControllerInterface < matlab.System
  properties (Constant)
        g = 9.81;
        r_arm = 0.0254;
        L = 0.4255;
        a = 0.75 * 5 * studentControllerInterface.g * studentControllerInterface.r_arm / (7 * studentControllerInterface.L);
        b = (5 * studentControllerInterface.L / 14) * (studentControllerInterface.r_arm / studentControllerInterface.L)^2;
        c = (5 / 7) * (studentControllerInterface.r_arm / studentControllerInterface.L)^2;

        k_p = [2.0 3.5 3.5 0.7];
        k_v = [4.5 2.5 7.0 7.5];
        
        k_theta = 6.0;
        k_omega = 1.0806;
        safety_angle_deg = 50.1070; 
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

        ukf;
    end
    methods(Access = protected)
        function setupImpl(obj)
%            disp("You can use this function for initializaition.");
                obj.ukf = unscentedKalmanFilter(...
                    @discrete_update,... % State transition function
                    @measurement,... % Measurement function
                    [0, 0, -56 * pi/180, 0],... % Initial state
                    'MeasurementNoise', diag([.05^2, .05^2]), ...
                    'ProcessNoise', diag([.001^2, .05^2, .001^2, 2^2]));
        end

        function [V_servo, ukf_state] = stepImpl(obj, t, p_ball_obs, theta_obs)
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
            correct(obj.ukf, [p_ball_obs, theta_obs]);

            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

%             FILTERING
%             ---------
            p_ball = obj.ukf.State(1);
            p_ball_dot = obj.ukf.State(2);
            theta = obj.ukf.State(3);
            theta_dot = obj.ukf.State(4);
            
            p_ball_error = p_ball - p_ball_ref;
            p_ball_dot_error = p_ball_dot - v_ball_ref;

            % SAFETY
            % ------
%             if (p_ball_ref < - 0.08) || (p_ball_ref > 0.1)
%                 p_ball_ref = p_ball_ref / 2;
%             end 
            
            % Position Control
            if (p_ball_ref > 0.07) && (v_ball_ref ~= 0)
                k_p = obj.k_p(1);
                k_v = obj.k_v(1);
            elseif (p_ball_ref > 0)
                k_p = obj.k_p(2);
                k_v = obj.k_v(2);
            elseif (p_ball_ref >= -0.07) || (v_ball_ref == 0)
                k_p = obj.k_p(3);
                k_v = obj.k_v(3);
            else
                k_p = obj.k_p(4);
                k_v = obj.k_v(4);
            end
            
            des_acc = -k_p * p_ball_error - k_v * p_ball_dot_error;
            V_servo_cur = obj.solveu(theta, theta_dot, des_acc);
            V_servo = V_servo_cur;

            % Make sure that the control input does not exceed the physical limit
            V_saturation = 10;    
            V_servo = min(V_servo, V_saturation);
            V_servo = max(V_servo, -V_saturation);

            ukf_state = obj.ukf.State;
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.p_ball_prev = p_ball;
            obj.theta_prev = theta;
            predict(obj.ukf, V_servo, dt);
            
            disp(obj.ukf.MeasurementNoise)
            disp(obj.ukf.ProcessNoise)
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d, ukf_state] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            ukf_state = obj.ukf.State;
        end

        function V_servo = solveu(obj, theta, theta_dot, v)
            max_sin_val = sin(obj.safety_angle_deg * pi / 180);

            sin_val = v/obj.a;
            sin_val = min(sin_val, max_sin_val);
            sin_val = max(sin_val, -max_sin_val);
            obj.theta_d = asin(sin_val);
            V_servo = obj.k_theta * (obj.theta_d - theta) - obj.k_omega * theta_dot;

        end
    end
    
end
