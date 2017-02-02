function obj = calculate_gcc(obj)
%CALCULATE_GCC This function calculates the linear Guaranteed Cost Controller for the nominal region
%
%    Input(s):
%    (1) obj - GCMPC class instance
%
%    Author(s):
%    (1) Carlos M. Massera

    % Check depedencies
    if ~obj.is_system_set
        error('System matrices not set, define them before the generating GCC')
    end
    
    if ~obj.is_disturbance_set
        error('Disturbance matrices not set, define them before the generating GCC')
    end
    
    if ~obj.is_cost_set
        error('Cost matrices not set, define them before the generating GCC')
    end
    
    % Define LMI variables
    p_inv = sdpvar(obj.n_x, obj.n_x);            % P^(-1)
    k_p_inv = sdpvar(obj.n_u, obj.n_x, 'full');  % K P^(-1)
    s = sdpvar(obj.n_x, obj.n_x);                % S >= P
    e = sdpvar();                                % GCC S-Procedure variable
    
    % Define GCC robustness requirement LMI
    gcc_lmi = blkvar;
    gcc_lmi(1,1) = - eye(obj.n_z);
    gcc_lmi(1,4) = obj.c_z * p_inv - obj.d_z * k_p_inv;
    gcc_lmi(2,2) = - e * eye(obj.n_y);
    gcc_lmi(2,4) = obj.c_y * p_inv - obj.d_y * k_p_inv;
    gcc_lmi(3,3) = - p_inv + e * (obj.b_w * obj.b_w');
    gcc_lmi(3,4) = obj.a * p_inv - obj.b_u * k_p_inv;
    gcc_lmi(4,4) = - p_inv;
    
    % Define GCC cost LMI
    cost_lmi = blkvar;
    cost_lmi(1,1) = - p_inv;
    cost_lmi(1,2) = eye(obj.n_x);
    cost_lmi(2,2) = - s;
    
    % Define YALMIP optimization problem
    constraints = [gcc_lmi <= 0; 
                   cost_lmi <= 0];
    objective = trace(s);
    options = sdpsettings('solver', obj.options.solver_sdp, 'verbose', 0);
    
    % Solve optimization problem
    solve_out = optimize(constraints, objective, options);
    
    if solve_out.problem ~= 0
        error('SDP solver did not converge, please check if your problem is correct');
    end
    
    % S-Procedure variable
    e = value(e);
    
    % GCC cost matrix
    obj.gcc.p = inv(value(p_inv));
    % GCC gain matrix
    obj.gcc.k = value(k_p_inv) * obj.gcc.p;
    % Suboptimal gains, needed to get r_bar
    obj.gcc.x = inv(value(p_inv) - e * (obj.b_w * obj.b_w'));
    % For a controller u = - K x + v, the new cost matrix is v' * r_bar * v
    obj.gcc.r_bar = (obj.d_z' * obj.d_z) + ...
                    e^(-1) * (obj.d_y' * obj.d_y) + ...
                    obj.b_u' * obj.gcc.x * obj.b_u;
    % Set calculation flag
    obj.is_gcc_set = true;
end

