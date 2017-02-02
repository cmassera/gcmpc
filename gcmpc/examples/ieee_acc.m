%% Add GCMPC to path
addpath('../')

%% System definitions
F = [1.1 0   0;
       0 0 1.2;
      -1 1   0];
G = [ 0 1;
      1 1;
     -1 0];

%% Disturbance definitions
H = [0.7; 0.5; -0.7];
E1 = [0.4 0.5 -0.6];
E2 = [0.4 -0.4];

%% Cost definitions
Q = eye(3);
R = eye(2);

%% Initialize class
gcmpc = GCMPC;
gcmpc.set_system(F, G);
gcmpc.set_disturbance(H, E1, E2);
gcmpc.set_cost(Q, R);

%% Generate linear controller
gcmpc.calculate_gcc()

%% Generate nil-potent controller
gcmpc.calculate_nilpotent()