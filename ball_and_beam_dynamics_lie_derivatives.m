syms z dz theta dtheta u

g = 9.81;
r_arm = 0.0254;
L = 0.4255;
K = 10;
tau = 0.1;

a = 5 * g * r_arm / (7 * L);
b = (5 * L / 14) * (r_arm / L)^2;
c = (5 / 7) * (r_arm / L)^2;

x = [z; dz; theta; dtheta];
h = [z; theta];

fx = [dz;
      a * sin(theta) - b * dtheta^2 * cos(theta)^2 + c * z * dtheta^2 * cos(theta)^2;
      dtheta;
      -dtheta / tau];

gx = [0; 0; 0; (K * u) / tau];

Lfh = jacobian(h, x) * fx
Lgh = jacobian(h, x) * gx

Lffh = jacobian(Lfh, x) * fx
Lgfh = jacobian(Lfh, x) * gx

Lfffh = jacobian(Lffh, x) * fx
Lgffh = jacobian(Lffh, x) * gx