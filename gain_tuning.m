close all;

global K tau;
K = 10;
tau = 0.1;

[omega_n, zeta] = get_angle_response(8.5703, 1.0806, 1)

[k_theta, k_omega] = get_angle_gains(10, 1)
get_angle_response(k_theta, k_omega, 1);

[omega_n, zeta] = get_position_response(2, 5)

[k_p, k_v] = get_position_gains(3, 2)
get_position_response(k_p, k_v);

function [k_theta, k_omega] = get_angle_gains(omega_n, zeta)
global K tau;

k_theta = tau / K * (omega_n ^ 2);
k_omega = (2 * tau * omega_n - 1) / K;
end

function [omega_n, zeta] = get_angle_response(k_theta, k_omega, plot)
global K tau;

omega_n = sqrt(k_theta * K / tau);
zeta = (1 + K * k_omega) / (2 * omega_n * tau);

sys = tf(omega_n ^ 2, [1, 2 * zeta * omega_n, omega_n ^ 2]);
stepinfo(sys)
if plot == 1
    figure()
    subplot(2, 1, 1)
    step(sys)

    subplot(2, 1, 2)
    bode(sys)
    xline(0.1667)
    xline(0.1)
end
end

function [k_p, k_v] = get_position_gains(omega_n, zeta)
g = 9.81;
r_arm = 0.0254;
L = 0.4255;
a = 5 * g * r_arm / (7 * L);

k_p = (omega_n ^ 2) / a;
k_v = 2 * zeta * omega_n;  
end

function [omega_n, zeta] = get_position_response(k_p, k_v)
g = 9.81;
r_arm = 0.0254;
L = 0.4255;
a = 5 * g * r_arm / (7 * L);

omega_n = sqrt(a * k_p);
zeta = k_v / (2 * omega_n);

sys = tf([k_v, a * k_p], [1, k_v, a * k_p]);
stepinfo(sys)

figure()
subplot(2, 1, 1)
step(sys)


subplot(2, 1, 2)
bode(sys)
xline(0.1667)
xline(0.1)
end

