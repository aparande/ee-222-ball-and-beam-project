function compute_lqr()
        out = lin_dyn(0, 0, 0, 0);
        A_lin = out(1:4, 1:4);
        B_lin = out(1:4, 5);
        Q = diag([10, 1, .15, .01]) * 100;
        R = 1;
        [K, ~, ~] = lqr(A_lin, B_lin, Q, R);
        save("lqr_gain.mat", "K");
end

