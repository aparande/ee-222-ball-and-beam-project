syms x1(t) x2(t) x3(t) x4(t) u(t) y(t) x1_dot(t) x2_dot(t) x3_dot(t) x4_dot(t) m g L r_g K tau

x1_dot(t) = x2(t);
x2_dot(t) = (5*g*r_g)/(7*L) * sin(x3(t)) - 5/7 * (L/2 - x1(t)) * (r_g/L)^2 * x4(t)^2 * cos(x3(t))^2;
x3_dot(t) = x4(t);
x4_dot = -x4(t)/tau + K/tau * u;
y = x1(t);

xi_4_dummy(t) = diff(x2_dot(t), x1(t)) + diff(x2_dot(t), x2) + diff(x2_dot(t), x3) + diff(x2_dot(t), x4);
xi_4(t) = diff(x2_dot(t), x1(t))*x1_dot(t) + diff(x2_dot(t), x2)*x2_dot(t) + diff(x2_dot(t), x3)*x3_dot(t) + diff(x2_dot(t), x4)*x4_dot(t);

xi_4_hand(t) = (5*g)/(7*L)*r_g*x4(t)*cos(x3(t)) + 5/(7*L)*r_g^2*x2(t)*x4(t)^2*cos(x3(t))^2 + 5/(7*L^2)*r_g^2*x4(t)^3*(L-2*x1(t))*cos(x3(t))*sin(x3(t)) ...
               +5/(7*L)*r_g^2*x4(t)^2/tau*cos(x3(t))^2 - 10/(7*L^2)*r_g^2*x1(t)*x4(t)^2/tau*cos(x3(t))^2 + 5/(7*L)*r_g^2*x4(t)*K/tau*cos(x3(t))^2*(2/L*x1(t)-1)*u;

% xi_4(t) = simplify(xi_4(t));
disp(xi_4_dummy(t));
disp(xi_4(t));
disp(isequal(xi_4(t), xi_4_hand(t)));

Latex = latex(xi_4(t));
disp(Latex);