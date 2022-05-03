syms z dz theta dtheta u zref dzref

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

gx = [0; 0; 0; K / tau];

Lfh = jacobian(h, x) * fx;
Lgh = jacobian(h, x) * gx;

Lffh = jacobian(Lfh, x) * fx;
Lgfh = jacobian(Lfh, x) * gx;

Lfffh = jacobian(Lffh, x) * fx;
Lgffh = jacobian(Lffh, x) * gx;

V = (z - zref)^2 + 0.1 * (dz - dzref)^2 + 0.01 * theta^2 + 0.01 * dtheta^2
LfV = jacobian(V, x) * fx;
LgV = jacobian(V, x) * gx;
LffV = jacobian(LfV, x) * fx;
LgfV = jacobian(LfV, x) * gx;
LfffV = jacobian(LffV, x) * fx
LgffV = jacobian(LffV, x) * gx
dddV = LfffV + LgffV * u;

B = (0.2^2 - z^2) * (45^2 - theta^2) + (2^2 - dz^2) + (2^2 - dtheta^2)
LfB = jacobian(B, x) * fx;
LgB = jacobian(B, x) * gx;
LffB = jacobian(LfB, x) * fx;
LgfB = jacobian(LfB, x) * gx;
LfffB = jacobian(LffB, x) * fx
LgffB = jacobian(LffB, x) * gx
dddB = LfffB + LgffB * u;