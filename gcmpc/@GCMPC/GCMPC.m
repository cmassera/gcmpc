classdef GCMPC < handle
% GCMPC This class defines the GCMPC problem and generates the controller for it.
    
    % TODO: Make all this private
    properties(SetAccess=private, GetAccess=public, Hidden)
        % System matrices 
        %     x_k+1 = A x_k + Bu u_k
        a   = [];
        b_u = [];
        
        % Disturbance matrices 
        %     x_k+1 = A x_k + Bu u_k + Bw w_k
        %     w_k = Delta (Cy x_k + Dy u_k)
        b_w = [];
        c_y = [];
        d_y = [];
        
        % Cost matrices
        %     | Q  N | = | Cz' Cz  Cz' Dz |
        %     | N' R |   | Dz' Cz  Dz' Dz |
        c_z = [];
        d_z = [];
        
        % Constraint matrices
        %     H x_k <= e
        h = [];
        e = [];
        
        % System dimensions
        n_x = 0;
        n_u = 0;
        n_w = 0;
        n_y = 0;
        n_z = 0;
        
        % Boolean flags to check if all requirements have been met
        is_system_set      = false;
        is_disturbance_set = false;
        is_cost_set        = false;
        is_constraint_set  = false;
        
        % Constants
        kPosDefTest = 1e-8;  % Minimum eigenvalue to consider matrix Positive-Definite
        kSymTest = 1e-8;     % maximum difference to transpose for symmetric 
    end
end

