%% System definitions

F = [1.1 0   0;
       0 0 1.2;
      -1 1   0];
G = [ 0 1;
      1 1;
     -1 0];

H = [0.7; 0.5; -0.7];
E1 = [0.4 0.5 -0.6];
E2 = [0.4 -0.4];

%% GUARANTEED COST CONTROL (XIE 1993)

Q = eye(3);
R = eye(2);

k_max = 1e4;
e_min = 1e-3;
e_max = 15e-1;
step = 1e-3;

e_interval = e_min:step:e_max;

S_array = NaN * ones(size(F, 1), size(F, 1), size(e_interval, 2));
S_cost = NaN * ones(size(e_interval, 2), 1);

for i = 1:size(e_interval, 2)
    e = e_interval(i);

    V = rand(3);
    X = Q;

    for k = 1 : k_max
        X_last = X;
        einv = e ^ -1;

        X = F' * X * F + ...
            e * X * H / (eye(size(H,2)) + e * H' * X * H) * H' * X - ...
            (F' * X * G + einv * E1'*E2 ) / (R + G' * X * G + einv * (E2' * E2)) * (G' * X * F + einv * E2' * E1) + ...
            einv * (E1'*E1) + Q;
        X = (X' + X) / 2;

        if norm(X_last - X) < 1e-4
            break
        end
    end
    
    if k >= k_max
        break
    end
    
    S = inv(inv(X) + e * (H * H'));
    S_array(:, :, i) = S;
    S_cost(i) = trace(S);
end

[~, idx] = min(S_cost);
e = e_interval(idx);
einv = e ^ -1;
S = S_array(:, :, idx);
X = inv(inv(S) - e * (H * H'));
K = (R + G' * X * G + einv * (E2' * E2)) \ (G' * X * F + einv * E2' * E1);

%% Problem variables
T = 10;

p_norm = Inf;
pd_norm = 1 / (1 - 1 / p_norm);

x = sdpvar(size(F,2), T + 1);
v = sdpvar(size(G,2), T);
u = - K * x(:, 1:T) + v;

%% Objective Functional
R_bar = R + G' * X * G + einv * (E2' * E2);

objective = x(:,1)' * S * x(:,1);
for i = 1:T
    objective = objective + v(:,i)' * R_bar * v(:,i);
end

%% Nil-potent Controller
Knp = sym('K', [size(G,2), size(F,2)], 'real');
np_order = size(F,2) - size(G,2) + 1;

Knp = double(subs(Knp, solve((F - G * Knp) ^ np_order == 0, Knp)));
F_tilda = F - G * Knp;

%% Robust constraints
% -1 <= x2 <= 1
Ak = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
Bk = zeros(6,2);
ck = - ones(6,1);

rho = @(i)(norm((E1 - E2 * K) * F_tilda^i * H, p_norm));

c = eye(T,T);
for k = 2:T
    for i = 1:k-1
        for j = 0:k-i-1
            c(k,i) = c(k,i) + rho(j) * c(k - j - 1, i);
        end
    end
end
c(abs(c) < 1e-10) = 0;

phi = sdpvar(1, T);
for k = 1:T
    phi(k) = norm((E1 - E2 * K) * x(:,k) + E2 * v(:,k), pd_norm);
end

phi_bar = phi * c';

A_tilda = Ak - Bk * K;

factor = zeros(T,T, size(Ak, 1));
for i = 1:size(Ak, 1)
    for k = 2:T
        for j = 1:k-1
            factor(k, j, i) = ...
                norm(A_tilda(i,:) * F_tilda ^ (k - j - 1) * H, pd_norm);
        end
    end
end
factor(abs(factor) < 1e-10) = 0;

Phi = sdpvar(size(Ak, 1), T);
for i = 1:size(Ak, 1)
    Phi(i, :) = phi_bar * factor(:, :, i)';
end

constraints = [x(:,2:end) == (F - G * K) * x(:,1:end-1) + G * v];
for k = 1:T
    constraints = [constraints;
        A_tilda * x(:,k) + Bk * v(:,k) + ck + Phi(:,k) <= 0];
end

%% YALMIP Controller
ops = sdpsettings('verbose',0);
controller = optimizer(constraints,objective,ops,x(:,1),v(:,1));

%% Simple test
N = 20;
X = NaN * ones(N+1, size(F,2));
if ~exist('Delta')
    Delta = 2 * rand(N, size(H,2), size(H,2)) - 1;
end
U = NaN * ones(N, size(G,2));
V = NaN * ones(N, size(G,2));

X(1,:) = [1;1;1];

for i = 1:N
    tic
    V(i,:) = controller{X(i,:)'}';
    toc
    U(i,:) = - X(i,:) * K' + V(i,:);
    
    delta = reshape(Delta(1, :, :), [size(H,2), size(H,2)]);
    X(i+1,:) = ((F + H * delta * E1) * X(i,:)' + ...
                (G + H * delta * E2) * U(i,:)')';
end