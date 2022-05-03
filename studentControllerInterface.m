classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        z_prev = 0;
        theta_prev = 0;
        theta_d = 0;
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
            %% Sample Controller: Simple Proportional Controller
            t_prev = obj.t_prev;
            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            % Decide desired servo angle based on simple proportional feedback.
            % k_p = 3;
            % theta_d = - k_p * (p_ball - p_ball_ref);
            p = 10;
            lambda = 10;
            gamma = 100;
            H = [[1, 0];
                 [0, p]];
            f = [0; 0];
            z = p_ball;
            zref = p_ball_ref;
            dz = (p_ball - obj.z_prev) / (t - obj.t_prev) - 0.001 * sign(z);
            dzref = v_ball_ref;
            dtheta = (theta - obj.theta_prev) / (t - obj.t_prev) - 0.001 * sign(theta);

            V = (z - zref)^2 + dtheta^2/100 + theta^2/100 + (dz - dzref)^2/10;
            LfffV = dz*(((2934539830446257*dtheta^2*cos(theta)^2)/5764607523034234880 + 2)*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976) - (14672699152231285*dtheta^2*cos(theta)^2*(dz/5 - dzref/5))/288230376151711744 + (2934539830446257*dtheta^2*cos(theta)^2*(2*z - 2*zref + (7535201836833413*sin(theta))/90071992547409920 - (4994586791419529*dtheta^2*cos(theta)^2)/46116860184273879040 + (2934539830446257*dtheta^2*z*cos(theta)^2)/5764607523034234880))/1152921504606846976 - (2934539830446257*dtheta^3*cos(theta)*sin(theta)*(dz/5 - dzref/5))/576460752303423488) - 10*dtheta*((201*dtheta)/50 - theta/5 - 10*dtheta*(((2934539830446257*z*cos(theta)^2)/576460752303423488 - (4994586791419529*cos(theta)^2)/4611686018427387904)*(dz/5 - dzref/5) - 2/5) - ((4994586791419529*dtheta*cos(theta)^2)/23058430092136939520 - (2934539830446257*dtheta*z*cos(theta)^2)/2882303761517117440)*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976) + (dz/5 - dzref/5)*((7535201836833413*cos(theta))/18014398509481984 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/4611686018427387904 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/576460752303423488) + 10*((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488)*(dz/5 - dzref/5) - ((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488)*(2*z - 2*zref + (7535201836833413*sin(theta))/90071992547409920 - (4994586791419529*dtheta^2*cos(theta)^2)/46116860184273879040 + (2934539830446257*dtheta^2*z*cos(theta)^2)/5764607523034234880) + dtheta*(((4994586791419529*dtheta*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)*sin(theta))/288230376151711744)*(dz/5 - dzref/5) + 1/50) + (2934539830446257*dtheta*dz*cos(theta)^2*(dz/5 - dzref/5))/576460752303423488) - dtheta*(10*dtheta*(((4994586791419529*dtheta*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)*sin(theta))/288230376151711744)*(dz/5 - dzref/5) + 1/50) - ((7535201836833413*cos(theta))/18014398509481984 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/4611686018427387904 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/576460752303423488)*(2*z - 2*zref + (7535201836833413*sin(theta))/90071992547409920 - (4994586791419529*dtheta^2*cos(theta)^2)/46116860184273879040 + (2934539830446257*dtheta^2*z*cos(theta)^2)/5764607523034234880) - ((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976)*((7535201836833413*cos(theta))/90071992547409920 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/23058430092136939520 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/2882303761517117440) + dtheta*(dz/5 - dzref/5)*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/4611686018427387904 + (4994586791419529*dtheta^2*sin(theta)^2)/4611686018427387904 + (2934539830446257*dtheta^2*z*cos(theta)^2)/576460752303423488 - (2934539830446257*dtheta^2*z*sin(theta)^2)/576460752303423488) + (2934539830446257*dtheta^2*dz*cos(theta)*sin(theta)*(dz/5 - dzref/5))/576460752303423488) + ((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976)*(2*dz + dtheta*((7535201836833413*cos(theta))/90071992547409920 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/23058430092136939520 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/2882303761517117440) + 10*dtheta*((4994586791419529*dtheta*cos(theta)^2)/23058430092136939520 - (2934539830446257*dtheta*z*cos(theta)^2)/2882303761517117440) + dz*((2934539830446257*dtheta^2*cos(theta)^2)/5764607523034234880 + 2) + (2934539830446257*dtheta^2*cos(theta)^2*(dz/5 - dzref/5))/1152921504606846976);
            LgffV = 402*dtheta - 20*theta - 1000*dtheta*(((2934539830446257*z*cos(theta)^2)/576460752303423488 - (4994586791419529*cos(theta)^2)/4611686018427387904)*(dz/5 - dzref/5) - 2/5) - 100*((4994586791419529*dtheta*cos(theta)^2)/23058430092136939520 - (2934539830446257*dtheta*z*cos(theta)^2)/2882303761517117440)*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976) + 100*(dz/5 - dzref/5)*((7535201836833413*cos(theta))/18014398509481984 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/4611686018427387904 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/576460752303423488) + 1000*((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488)*(dz/5 - dzref/5) - 100*((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488)*(2*z - 2*zref + (7535201836833413*sin(theta))/90071992547409920 - (4994586791419529*dtheta^2*cos(theta)^2)/46116860184273879040 + (2934539830446257*dtheta^2*z*cos(theta)^2)/5764607523034234880) + 100*dtheta*(((4994586791419529*dtheta*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)*sin(theta))/288230376151711744)*(dz/5 - dzref/5) + 1/50) + (73363495761156425*dtheta*dz*cos(theta)^2*(dz/5 - dzref/5))/144115188075855872;
            B = (theta^2 - 2025)*(z^2 - 1/25) - dz^2 - dtheta^2 + 8;
            LfffB = dtheta*(dtheta*(2*dz*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/4611686018427387904 + (4994586791419529*dtheta^2*sin(theta)^2)/4611686018427387904 + (2934539830446257*dtheta^2*z*cos(theta)^2)/576460752303423488 - (2934539830446257*dtheta^2*z*sin(theta)^2)/576460752303423488) + 4*dz*z) - ((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976)*((7535201836833413*cos(theta))/9007199254740992 - 4*theta*z + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/288230376151711744) - ((7535201836833413*cos(theta))/18014398509481984 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/4611686018427387904 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/576460752303423488)*((7535201836833413*sin(theta))/9007199254740992 - 2*z*(theta^2 - 2025) - (4994586791419529*dtheta^2*cos(theta)^2)/4611686018427387904 + (2934539830446257*dtheta^2*z*cos(theta)^2)/576460752303423488) + 10*dtheta*(2*dz*((4994586791419529*dtheta*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)*sin(theta))/288230376151711744) - 2*z^2 + 2/25) + dz*((2934539830446257*dz*cos(theta)*sin(theta)*dtheta^2)/288230376151711744 + 4*z*dtheta + 4*dz*theta)) - 10*dtheta*(((4994586791419529*dtheta*cos(theta)^2)/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)^2)/288230376151711744)*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976) - 20*theta*(z^2 - 1/25) - 2*dz*((7535201836833413*cos(theta))/18014398509481984 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/4611686018427387904 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/576460752303423488) - 400*dtheta - 20*dz*((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488) + ((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488)*((7535201836833413*sin(theta))/9007199254740992 - 2*z*(theta^2 - 2025) - (4994586791419529*dtheta^2*cos(theta)^2)/4611686018427387904 + (2934539830446257*dtheta^2*z*cos(theta)^2)/576460752303423488) + 10*dtheta*(2*dz*((2934539830446257*z*cos(theta)^2)/576460752303423488 - (4994586791419529*cos(theta)^2)/4611686018427387904) - 40) + dz*(4*theta*z - (2934539830446257*dtheta*dz*cos(theta)^2)/288230376151711744) - dtheta*(2*dz*((4994586791419529*dtheta*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)*sin(theta))/288230376151711744) - 2*z^2 + 2/25) + 2*dtheta*(z^2 - 1/25) + 4*dz*theta*z) - ((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976)*(10*dtheta*((4994586791419529*dtheta*cos(theta)^2)/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)^2)/288230376151711744) + dtheta*((7535201836833413*cos(theta))/9007199254740992 - 4*theta*z + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/288230376151711744) + dz*((2934539830446257*dtheta^2*cos(theta)^2)/576460752303423488 - 2*theta^2 + 4050) - 2*dz*(theta^2 - 2025) - 4*dtheta*theta*z + (2934539830446257*dtheta^2*dz*cos(theta)^2)/576460752303423488) - dz*(((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976)*((2934539830446257*dtheta^2*cos(theta)^2)/576460752303423488 - 2*theta^2 + 4050) + 10*dtheta*(4*theta*z - (2934539830446257*dtheta*dz*cos(theta)^2)/288230376151711744) - dtheta*((2934539830446257*dz*cos(theta)*sin(theta)*dtheta^2)/288230376151711744 + 4*z*dtheta + 4*dz*theta) - 4*dtheta*dz*theta + (2934539830446257*dtheta^2*cos(theta)^2*((7535201836833413*sin(theta))/9007199254740992 - 2*z*(theta^2 - 2025) - (4994586791419529*dtheta^2*cos(theta)^2)/4611686018427387904 + (2934539830446257*dtheta^2*z*cos(theta)^2)/576460752303423488))/1152921504606846976);
            LgffB = 100*((4994586791419529*dtheta*cos(theta)^2)/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)^2)/288230376151711744)*((7535201836833413*sin(theta))/18014398509481984 - (4994586791419529*dtheta^2*cos(theta)^2)/9223372036854775808 + (2934539830446257*dtheta^2*z*cos(theta)^2)/1152921504606846976) - 2000*theta*(z^2 - 1/25) - 200*dz*((7535201836833413*cos(theta))/18014398509481984 + (4994586791419529*dtheta^2*cos(theta)*sin(theta))/4611686018427387904 - (2934539830446257*dtheta^2*z*cos(theta)*sin(theta))/576460752303423488) - 40000*dtheta - 2000*dz*((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488) + 100*((4994586791419529*dtheta*cos(theta)^2)/4611686018427387904 - (2934539830446257*dtheta*z*cos(theta)^2)/576460752303423488)*((7535201836833413*sin(theta))/9007199254740992 - 2*z*(theta^2 - 2025) - (4994586791419529*dtheta^2*cos(theta)^2)/4611686018427387904 + (2934539830446257*dtheta^2*z*cos(theta)^2)/576460752303423488) + 1000*dtheta*(2*dz*((2934539830446257*z*cos(theta)^2)/576460752303423488 - (4994586791419529*cos(theta)^2)/4611686018427387904) - 40) + 100*dz*(4*theta*z - (2934539830446257*dtheta*dz*cos(theta)^2)/288230376151711744) - 100*dtheta*(2*dz*((4994586791419529*dtheta*cos(theta)*sin(theta))/2305843009213693952 - (2934539830446257*dtheta*z*cos(theta)*sin(theta))/288230376151711744) - 2*z^2 + 2/25) + 200*dtheta*(z^2 - 1/25) + 400*dz*theta*z;
 
            A = [[LgffV, -1];
                 [-LgffB, 0]];
            b = [[-LfffV - lambda * V]; 
                 [LfffB + gamma * B]];
            u_delta = quadprog(H, f, A, b);
            % Make sure that the desired servo angle does not exceed the physical
            % limit. This part of code is not necessary but highly recommended
            % because it addresses the actual physical limit of the servo motor.
            % theta_saturation = 56 * pi / 180;    
            % theta_d = min(theta_d, theta_saturation);
            % theta_d = max(theta_d, -theta_saturation);

            % Simple position control to control servo angle to the desired
            % position.
            % k_servo = 10;
            % V_saturation = 56 * pi / 18;
            V_servo = u_delta(1)
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.z_prev = p_ball;
            obj.theta_prev = theta;
            % obj.theta_d = theta_d;
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
