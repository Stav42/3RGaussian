function [mu, rho] = uncertainty(xk, H)
    if isempty(H)
        mu = [0,0];
        rho = 0.01;
        return
    end
    s = size(H);
    len = s(1);
    x = H(:, 1:4); y = H(:, 5:6);
%     x = H(:, 1:6); y = H(:, 7:8);
    kn = zeros(1, len);
    Kn = zeros(len);
    for i=1:len
        kn(1, i) = kernel(xk, x(i, :)');
    end
    for i=1:len
        for j=i:len
            val = kernel(x(i, :)', x(j, :)');
            Kn(i, j) = val;
            Kn(j, i) = val;
        end
    end

    %% Hyperparameter tuning using Marginal Log Likelihood

    sigma_w = 0.01;
    Kn_ = Kn + eye(len) * sigma_w^2;
    mu = kn * (Kn_ \ y);
    sigma = kernel(xk, xk) - kn * (Kn_ \ kn');
    rho1 = max(abs(mu(1) + 3 * sigma), abs(mu(1) - 3 * sigma));
    rho2 = max(abs(mu(2) + 3 * sigma), abs(mu(2) - 3 * sigma));

    rho = norm([rho1; rho2]);
end