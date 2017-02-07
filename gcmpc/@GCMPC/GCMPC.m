classdef GCMPC < handle
% GCMPC This class defines the GCMPC problem and generates the controller for it.
    
    % TODO: Make all this private
    properties(SetAccess=private, GetAccess=public, Hidden)
        % System matrices 
        %     x_k+1 = A x_k + Bu u_k + Br r_k
        a   = [];
        b_u = [];
        b_r = [];
        
        % Disturbance matrices 
        %     x_k+1 = A x_k + Bu u_k + Br r_k + Bw w_k
        %     w_k = Delta (Cy x_k + Dyu u_k)
        b_w = [];
        c_y = [];
        d_y_u = [];
        d_y_r = [];
        
        % Cost matrices
        %     | Q  N | = | Cz'  Cz   Cz' Dz |
        %     | N' R |   | Dzu' Cz  Dzu' Dzu |
        c_z = [];
        d_z_u = [];
        
        % Constraint matrices
        %     Hx x_k + Hu u_k + Hr r_k + g <= 0
        h_x = [];
        h_u = [];
        h_r = [];
        g = [];
        
        % Reference performance matrices
        %     p_k = Cp x_k + Dpu u_k
        %     || p(s) / r(s) ||_Inf < perf_gamma
        c_p = [];
        d_p_u = [];
        perf_gamma = 0;
        
        % System dimensions
        n_x = 0;  % Number of states
        n_u = 0;  % Number of control inputs
        n_r = 0;  % Number of reference inputs
        n_p = 0;  % Number of referemce performance inputs
        n_w = 0;  % Number of disturbance inputs
        n_y = 0;  % Number of disturbance outputs
        n_z = 0;  % Number of cost outputs (positive eigenvalues of [Q N; N' R])
        n_c = 0;  % Number of constraints
        n_t = 0;  % GCMPC horizon length
        
        % Linear GCC results
        gcc = struct('k', [], 'p', [], 'x', [], 'r_bar', [], 'l', []);
        
        % Nil potent controller results
        np = struct('k', [], 'a_cl', []);
        
        % Optimization problem structure
        opt = struct('objective', 0, 'constraint', [], ...
                     'variable', struct('x', [], 'v', [], 'u', [], 'r', []), ...
                     'controller', 0);
        
        % Boolean flags to check if all requirements have been met
        is_system_set                = false;
        is_reference_set             = false;
        is_reference_performance_set = false;
        is_disturbance_set           = false;
        is_cost_set                  = false;
        is_constraint_set            = false;
        is_constraint_soft           = false;
        is_gcc_set                   = false;
        is_nilpotent_set             = false;
        
        % Constants
        kPosDefTest = 1e-8;    % Minimum eigenvalue to consider matrix Positive-Definite
        kSymTest = 1e-8;       % Maximum difference to transpose for symmetric 
        kZeroTest = 1e-8;      % Maximum absolute value to be considered zero
        kSlackWeight = 1e4;    % Slack weight for soft constraints
        kSdpSolver = 'mosek';  % Default SDP solver
        kQpSolver = 'mosek';   % Default (QC)QP solver
    end
    
    properties(SetAccess=public, GetAccess=public)
        % Options structure
        options = struct();
    end
    
    methods
        function obj = GCMPC()
            % Check if YALMIP is installed
            try
                yalmip('clear');
            catch
                error('YALMIP not found, please install it first')
            end
            
            % Set default solvers, you can change this later
            obj.options.solver_sdp = obj.kSdpSolver;
            obj.options.solver_qp = obj.kQpSolver;
        end
    end
end

