function z = kernel(x1, x2)
    sigma_n = 1e-4;
    l = 0.1;
    l_ = 1/l;
    L2_inv = diag([l_, l_, l_, l_]);
%     L2_inv = diag([l_, l_, l_, l_, l_, l_]);
    z = sigma_n^2 * exp(-0.5 * (x1 - x2)' * L2_inv * (x1 - x2));
end