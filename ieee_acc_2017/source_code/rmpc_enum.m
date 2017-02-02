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

%% Problem variables
T = 10;

p_norm = 2;
pd_norm = 1 / (1 - 1 / p_norm);

x0 = sdpvar(size(F,2), 1);
u = sdpvar(size(G,2), T);

%% Objective functional & constraints
Q = eye(3);
R = eye(2);
[~, P] = dlqr(F,G,Q,R);

Ak = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
Bk = zeros(6,2);
ck = - ones(6,1);

%% QP problem construction

x = x0;
objective = x' * Q * x;
constraints = [];
for i = 1:T
    x_next = sdpvar(size(F,2), 2 * size(x, 2));
    
    x_p = (F + H * E1) * x + (G + H * E2) * u(:,i) * ones(1,size(x,2));
    x_n = (F - H * E1) * x + (G - H * E2) * u(:,i) * ones(1,size(x,2));
    x = x_next;
    
    constraints = [constraints; 
                       x == [x_p, x_n];
                       Ak * x + (Bk * u(:,1) + ck) * ones(1, size(x, 2)) <= 0];
    
    v = sdpvar(size(x, 2), 1);
    for j = 1:size(x, 2)
        if i < T
            constraints = [constraints; x(:,j)' * Q * x(:,j) - v(j) <= 0];
        else
            constraints = [constraints; x(:,j)' * S * x(:,j) - v(j) <= 0];
        end
    end
    objective = objective + norm(v, Inf) + u(:,i)' * R * u(:,i);
end

%% YALMIP controller
ops = sdpsettings('solver', 'mosek', 'verbose', 0);
controller = optimizer(constraints,objective,ops,x0,u(:,1));

%% Simple test
N = 20;
X1 = NaN * ones(N+1, size(F,2));
if ~exist('Delta')
    Delta = 2 * rand(N, size(H,2), size(H,2)) - 1;
end
U1 = NaN * ones(N, size(G,2));
V1 = NaN * ones(N, size(G,2));

controller{zeros(size(F,2), 1)};

X1(1,:) = [1;1;1];

for i = 1:N
    tic
    U1(i,:) = controller{X(i,:)'}';
    toc
    
    delta = reshape(Delta(1, :, :), [size(H,2), size(H,2)]);
    X1(i+1,:) = ((F + H * delta * E1) * X(i,:)' + ...
                 (G + H * delta * E2) * U(i,:)')';
end

%% Timing test
M = 1000;

rmpc_time = zeros(M,1);

for i = 1:M
    tic
    controller{0.5*rand(size(F,2), 1)};
    rmpc_time(i) = toc;
end