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
%         positions = zeros(1, 5);
%         control_inputs = zeros(1, 7);
%         theta_traj = zeros(1, 5);
        ukf;
    end
    methods(Access = protected)
        function setupImpl(obj)
%            disp("You can use this function for initializaition.");
                obj.ukf = unscentedKalmanFilter(...
                    @discrete_update,... % State transition function
                    @measurement,... % Measurement function
                    [0, 0, 0, 0],... % Initial state
                    'MeasurementNoise', diag([.02^2, .05^2]), ...
                    'ProcessNoise', diag([.001^2, .01^2, .001^2, 2^2]));
        end

        function V_servo = stepImpl(obj, t, p_ball_obs, theta_obs)
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

%             obj.positions = [obj.positions(2:end), p_ball_cur];
%             p_ball = mean(obj.positions);
%             if abs(p_ball_cur) > .1
%                 p_ball_ref = 0;
%             end

%             FILTERING
%             ---------
            p_ball = obj.ukf.State(1);
            p_ball_dot = obj.ukf.State(2);
            theta = obj.ukf.State(3);
            theta_dot = obj.ukf.State(4);

%               NO FILTERING
%               ------------
%               p_ball = p_ball_obs;
%               p_ball_dot = (p_ball - obj.p_ball_prev)/dt;
%               theta = theta_obs;
%               theta_dot = (theta - obj.theta_prev)/dt;

            % Approximate state derivatives
%             p_ball_dot = (p_ball - obj.p_ball_prev)/dt;
%             if t < .1
%                 p_ball_dot_cur = 0;
%             end
%             if abs(p_ball_dot) > .1
%                 v_ball_ref = 0;
%             end
%             
%             theta_dot = (theta - obj.theta_prev)/dt;
%             z = [p_ball; p_ball_dot; theta; theta_dot];
            
            % error = [p_ball - p_ball_ref; p_ball_dot - v_ball_ref; theta; theta_dot];
            p_ball_error = p_ball - p_ball_ref;
            p_ball_dot_error = p_ball_dot - v_ball_ref;
            pid = [5.0, 3]; % working gains
%             pid = [31.6228   16.2248];
%             pid = [7.0711    4.9135];
%             pid = [5.7735    4.2677];
%             pid = [4.4721    3.5978];
%             pid = [3.1623    2.8852];
            des_acc = -pid(1) * p_ball_error -pid(2) * p_ball_dot_error;
            V_servo_cur = obj.solveu(theta, theta_dot, des_acc);
%             obj.control_inputs = [obj.control_inputs(2:end), V_servo_cur];
%             V_servo = mean(obj.control_inputs);
            V_servo = V_servo_cur;

            % Make sure that the control input does not exceed the physical limit
            V_saturation = 10;    
            V_servo = min(V_servo, V_saturation);
            V_servo = max(V_servo, -V_saturation);
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.p_ball_prev = p_ball;
            obj.theta_prev = theta;
            predict(obj.ukf, V_servo);
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
                sin_val = v/obj.a;
                max_sin_val = .70;
                sin_val = min(sin_val, max_sin_val);
                sin_val = max(sin_val, -max_sin_val);
                obj.theta_d = asin(sin_val);
                pid = [4.0, .2]; % working gains
                V_servo = pid(1) * (obj.theta_d - theta) - pid(2) * theta_dot;

        end
    end
    
end
