function K = LQR_motor(q1, q2, R)
    A = [0 1; 0 -40];
    B = [0; 1];
    Q = diag([q1, q2]);
    K = lqr(A, B, Q, R);
end