function compute_lqr()
    A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    B = [0; 0; 0; 1];
    Q = diag([1000, 1000, 1000, 1000]);
    R = 1;
    [K, ~, ~] = lqr(A, B, Q, R);
    save('lqr_gain.mat', 'K');
end

