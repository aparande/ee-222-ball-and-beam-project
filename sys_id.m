load("simulink_thetas.mat", "thetas")
load("simulink_ps.mat", "ps")

ts = thetas(1, :);
thetas = thetas(2, :);
ang_vels = diff(thetas);

obj_func = @(x) obj_first_order(x, ts, thetas);
[params, val] = fminunc(obj_func, [-1 1])

obj_func = @(x) obj_second_order(x, ts, thetas);
[params, val] = fminunc(obj_func, [-1, -1, 1])

ps = ps(2, :);
vels = diff(ps);
accels = diff(vels);

obj_func = @(x) obj_accel(x, ps, thetas, ang_vels, accels);
[params, val] = fminunc(obj_func, [0 0])

% x4_dot = a x4 + b u
function f = obj_first_order(x, ts, thetas)
A = - x(2) / x(1);
B = x(2) / (x(1) ^ 2);
C = -B;

step_resp = 0.001 * (A * ts + B * exp(x(1) * ts) + C);
f = sum((step_resp - thetas) .^ 2, 'all');
end

% x4_dot = a x3 + b x4 + c u
function f = obj_second_order(x, ts, thetas)
p1 = (x(2) + sqrt(x(2)^2 - 4 * x(1))) / (2 * x(1));
p2 = (x(2) - sqrt(x(2)^2 - 4 * x(1))) / (2 * x(1));

step_resp = 0.001 * x(3) * (exp(p1 * ts) / (p1 - p2) + exp(p2 * ts) / (p2 - p1));
step_resp = real(step_resp);

f = sum((step_resp - thetas) .^ 2, 'all');
end

function f = obj_accel(x, ps, thetas, ang_vels, accels)
A = sin(thetas(1:end-2));
B = (ang_vels(1:end - 1) .* cos(thetas(1: end - 2))) .^2 .* (0.21275 - ps(1:end-2));

target = x(1) * A + x(2) * B;
f = sum((accels - target) .^ 2, 'all');
end



