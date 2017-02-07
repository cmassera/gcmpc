%% Add GCMPC to path
addpath('../')

%% System definitions
F = [1.1 0   0;
       0 0 1.2;
      -1 1   0];
G = [ 0 1;
      1 1;
     -1 0];
Gr = [1; 
      0; 
      0];

%% Disturbance definitions
H = [0.7; 0.5; -0.7];
E1 = [0.4 0.5 -0.6];
E2 = [0.4 -0.4];
E3 = [-0.2];

%%
D1 = [0, 0, 1];
D2 = [0, 0];
gamma = 4;

%% Cost definitions
Q = eye(3);
R = eye(2);

%% Constraint definitions
Ak = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
Bk = zeros(6,2);
ck = - ones(6,1);

%% Initialize class
gcmpc = GCMPC;
gcmpc.set_system(F, G, Gr);
gcmpc.set_disturbance(H, E1, E2, E3);
gcmpc.set_reference_performance(D1, D2, gamma);
gcmpc.set_cost(Q, R);
gcmpc.set_constraint(Ak, Bk, ck);

%% Generate linear controller
T = 10;
controller = gcmpc.generate(T);

%% Simple test
N = 20;
X = NaN * ones(N+1, size(F,2));
if ~exist('Delta')
    Delta = 2 * rand(N, size(H,2), size(H,2)) - 1;
end
U = NaN * ones(N, size(G,2));

controller{zeros(size(F,2), 1)};

X(1,:) = [1;1;1];

for i = 1:N
    tic
    U(i,:) = controller{X(i,:)'}';
    toc
    
    delta = reshape(Delta(1, :, :), [size(H,2), size(H,2)]);
    X(i+1,:) = ((F + H * delta * E1) * X(i,:)' + ...
                (G + H * delta * E2) * U(i,:)')';
end