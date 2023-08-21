close, clear, clc

% Actual and erred params
m1 = 1; m2 = 1; l1 = 1; l2 = 1;
l1_ = l1 * 0.95; l2_ = l2 * 0.95;

% Anonymous functions for Mass Matrix, Non-linear, and Gravity terms
[M, C, G] = DOUBLE_PENDULUM_EOM(m1, m2, l1, l2);
[M_, C_, G_] = DOUBLE_PENDULUM_EOM(m1, m2, l1_, l2_);

% First order ODE form
f = @(x, u) [x(3:4); M(x(1:2)) \ (u - C(x(1:2), x(3:4)) * x(3:4) - G(x(1:2)))];

%% Initial conditions
x0 = [0; 0; 0.0; 0];

%% Desired trajectory
xd = @(t) [0.1*sin(t); 0.3 * sin(2*t); 0.1 * cos(t); 0.3 * 2 * cos(2 * t)];
xdd = @(t) [-0.1 * sin(t); -0.3 * 4 * sin(2 * t)];

%% Kp and Kd from joint level LQR
k1 = 26.5948; k2 = 13.4173;
K = [k1, 0, k2, 0; 0, k1, 0, k2];

%% ID controllers: 1) u_id: Actual Params 2) u_id_: Different params
u_id = @(x, t) M(x(1:2)) * (xdd(t) + K * (xd(t) - x)) + C(x(1:2), x(3:4)) * x(3:4) + G(x(1:2));
u_id_ = @(x, t, neta_) M_(x(1:2)) * (xdd(t) - neta_ + K * (xd(t) - x)) + C_(x(1:2), x(3:4)) * x(3:4) + G_(x(1:2));


%% Robust controller term calculation
A = [zeros(2,2),eye(2);-K];
B = [zeros(2,2); eye(2)];
Q = 0.01 * eye(4);
P = care(A, [], Q);
BTP = B' * P;
rho = 1;
eps = 0.00001;
r = @(x, t) (norm(BTP * (xd(t) - x)) > eps) .* (-rho * BTP * (xd(t) - x) / norm(BTP * (xd(t) - x))) ...
          + (norm(BTP * (xd(t) - x)) <= eps) .* (-rho * BTP * (xd(t) - x) / eps);


%% Forward integration (Simulation)
% [t, y] = ode45(@(t, y) f(y, u_id_(y, t) + r(y, t)), [0 100], x0);

% H = zeros(100, 6);
H = zeros(0, 6);

dt = 1e-3; T = 10;
tspan=0:dt:T; t = tspan';
Y = zeros(length(tspan), 4);
Y(1, :) = x0';
y_k = x0;
neta = zeros(1, 2);
for k=1:length(tspan)-1
    tk = tspan(k);
    if k > 1
        [neta, sigma] = uncertainty(y_k, H);
    end
    y_kplus1 = rk4singlestep(@(t, y) f(y, u_id_(y, t, neta')), dt, tk, y_k);
    qdd = (y_kplus1(3:4) - y_k(3:4)) / dt;

    neta_k = qdd - xdd(tk);
    
    H = [H; [y_kplus1', neta_k']];
    if length(H) > 100
        H = H(2:end, :);
    end

    Y(k + 1, :) = y_kplus1';
    y_k = y_kplus1;
end


%% Calculating the trajectory error
xd_ = xd(t')';
err = xd_ - Y;

%% Plotting and Visualization
plot(t, xd_(:, 1), t, Y(:, 1))
plot(t, err(:, 1))
plot(t, xd_(:, 2), t, Y(:, 2))
plot(t, err(:, 2))
% figure
% plot(t, y(:, 3), t, y(:, 4))

function [mu, sigma] = uncertainty(xk, H)
    if isempty(H)
        mu = [0,0];
        sigma = 1;
        return
    end
    s = size(H);
    len = s(1);
    x = H(:, 1:4); y = H(:, 5:6);
    kn = zeros(1, len);
    Kn = zeros(len);
    for i=1:len
        kn(1, i) = kernel(xk, x(i, :)');
%         kn(2, i) = kernel(xk, x(i, :)');
    end
    for i=1:len
        for j=i:len
            val = kernel(x(i, :)', x(j, :)');
            Kn(i, j) = val;
            Kn(j, i) = val;
        end
    end
    sigma_w = 0.001;
    Kn_ = Kn + eye(len) * sigma_w^2;
%     disp(size(kn))
%     disp(size(kn * Kn_))
%     disp(size(kn * (Kn_ \ y)))
    mu = kn * (Kn_ \ y);
    sigma = kernel(xk, xk) - kn * (Kn_ \ kn');
end

function z = kernel(x1, x2)
    sigma_n = 1;
    l = 0.1;
    l_ = 1/l;
    L2_inv = inv(diag([l_, l_, l_, l_]));
    z = sigma_n^2 * exp(-0.5 * (x1 - x2)' * L2_inv * (x1 - x2));
end
