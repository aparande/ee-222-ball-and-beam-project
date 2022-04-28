clear; clc;
syms x1(t) x2(t) x3(t) x4(t) u(t) y(t) x1_dot(t) x2_dot(t) x3_dot(t) x4_dot(t) m g L r_g K tau

x1_dot(t) = x2(t);
x2_dot(t) = (5*g*r_g)/(7*L) * sin(x3(t)) - 5/7 * (L/2 - x1(t)) * (r_g/L)^2 * x4(t)^2 * cos(x3(t))^2;
x3_dot(t) = x4(t);
x4_dot = -x4(t)/tau + K/tau * u;
y = x1(t);

xi_3(t) = 5*g/(7*L)*r_g*sin(x3(t));
xi_3_dot(t) = diff(xi_3(t), x1(t))*x1_dot(t) + diff(xi_3(t), x2(t))*x2_dot(t) + diff(xi_3(t), x3(t))*x3_dot(t) + diff(xi_3(t), x4(t))*x4_dot(t);

xi_4(t) = xi_3_dot(t);
xi_4_dot(t) = diff(xi_4(t), x1(t))*x1_dot(t) + diff(xi_4(t), x2(t))*x2_dot(t) + diff(xi_4(t), x3(t))*x3_dot(t) + diff(xi_4(t), x4(t))*x4_dot(t);

Latex_3 = latex(xi_3_dot(t));
display(xi_3_dot(t));
disp(Latex_3);

Latex_4 = latex(xi_4_dot(t));
display(xi_4_dot(t));
disp(Latex_4);