g = 9.81;
r_arm = 0.0254;
L = 0.4255;
K = 10;
tau = 0.1;

global a b c;

a = 5 * g * r_arm / (7 * L);
b = (5 * L / 14) * (r_arm / L)^2;
c = (5 / 7) * (r_arm / L)^2;

plant = @(t, x) dynamics(t, x);
    
ic = [-.01; 0; 0; 0];
[t, x] = ode89(plant, [0, 10], ic);

[p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

subplot(4, 1, 1)
hold on

plot(t, x(:, 1))
plot(t, p_ball_ref);
title("x")

hold off

subplot(4, 1, 2)
hold on
plot(t, x(:, 2))
plot(t, v_ball_ref)
hold off
title("vel")

subplot(4, 1, 3)
plot(t, 180 * x(:, 3) / pi)
title("theta")

subplot(4, 1, 4)
plot(t, x(:, 4))
title("theta dot")

figure()

subplot(3, 1, 1)
opt_virtual_ctrl = virtual_ctrl(t, x(:, 1), x(:, 2), x(:, 3));
hold on;
plot(t, opt_virtual_ctrl)
plot(t, x(:, 4) .* cos(x(:, 3)) .^ 2)
title("Optimal virtual Control")
legend("Optimal", "Actual")

subplot(3, 1, 2)
plot(t, b - c * x(:, 1))
title("Denom")

subplot(3, 1, 3)
V = ctrl(x(:, 3), x(:, 4), opt_virtual_ctrl);
plot(t, V);
title("V")



function u_tilde = virtual_ctrl(t, p_ball, v_ball, theta)
global a b c;

[p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

u_tilde = 1 ./ (b - c .* p_ball) .* (a .* sin(theta) + (v_ball - v_ball_ref) + 2 .* (p_ball - p_ball_ref));
%u_tilde = sin(2 * pi * t);
end

function u = ctrl(theta, dtheta, u_d)
K = 10;
tau = 0.1;

u_tilde = (dtheta .* cos(theta)) .^ 2;

z = tau / ( 2 * K .* cos(theta) .^ 2);
dtheta(dtheta < 1) = 1;
z = z ./ dtheta;

u = z * (2 / tau * u_tilde + dtheta .^ 3 .* sin(2 .* theta) - 2 .* (u_d - u_tilde));
end

function dx = dynamics(t, x)
global a b c;

K = 10;
tau = 0.1;

p_ball = x(1);
v_ball = x(2);
theta = x(3);
dtheta = x(4);

dx = zeros(4, 1);

u_tilde = virtual_ctrl(t, p_ball, v_ball, theta);

% dynamics
dx(1) = v_ball;
dx(2) = a * sin(theta) - b * dtheta^2 * cos(theta)^2 + c * p_ball * dtheta^2 * cos(theta)^2;
dx(3) = dtheta;
dx(4) = (- dtheta + K * ctrl(theta, dtheta, u_tilde)) / tau;
end
