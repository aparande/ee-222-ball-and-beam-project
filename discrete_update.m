function x_new = discrete_update(x, u)
    dt = .01;
    x_new = x + ball_and_beam_dynamics(0, x, u) * dt;
end