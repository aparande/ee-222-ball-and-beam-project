% Define constants
r_g = 0.0254;
L = 0.4255;
g = 9.81;
K = 1.5;
tau = 0.025;

% Define symbolic variables
syms x x_dot theta theta_dot u real
z = [x; x_dot; theta; theta_dot];

f = [
    x_dot;
    (5*g/7)*(r_g/L)*sin(theta) - (5/7)*(L/2 - x)*(r_g/L)^2*theta_dot^2*cos(theta)^2;
    theta_dot;
    -(theta_dot/tau)
    ];
g = [
    0;
    0;
    0;
    (K/tau)
    ];
z_dot = f + g*u;

matlabFunction(f,'File','nonlin_dyn', 'Vars', [x, x_dot, theta, theta_dot, u]);

A_lin = jacobian(f, [x, x_dot, theta, theta_dot]);
B_lin = g;

matlabFunction([A_lin, B_lin], 'File', 'lin_dyn', 'Vars', [x, x_dot, theta, theta_dot]);