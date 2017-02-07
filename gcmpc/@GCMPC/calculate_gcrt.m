function obj = calculate_gcrt(obj)
%CALCULATE_GCRT This function calculates the linear GCC for a LTI system with reference input
%
%    Input(s):
%    (1) obj - GCMPC class instance
%
%    Author(s):
%    (1) Carlos M. Massera

    % Check depedencies
    if ~obj.is_system_set
        error('System matrices not set, define them before the generating GCRT')
    end
    
    if ~obj.is_disturbance_set
        error('Disturbance matrices not set, define them before the generating GCRT')
    end
    
    if ~obj.is_cost_set
        error('Cost matrices not set, define them before the generating GCRT')
    end
    
    if ~obj.is_reference_set
        error('This system does not have a reference input, use GCC instead of GCRT')
    end
    
    if ~obj.is_reference_performance_set
        error('Reference performance matrices not set, define them before the generating GCRT')
    end
    
    % Define LMI variables
    p_inv = sdpvar(obj.n_x, obj.n_x);            % P^(-1)
    k_p_inv = sdpvar(obj.n_u, obj.n_x, 'full');  % K P^(-1)
    l_e_r = sdpvar(obj.n_u, obj.n_r, 'full');    % L e_r
    s = sdpvar(obj.n_x, obj.n_x);                % S >= P
    e_w = sdpvar();                              % Uncertainty S-Procedure variable
    e_r = sdpvar();                              % Reference S-Procedure variable
    
    % Define GCC robustness requirement LMI
    gcrt_lmi = blkvar;
    gcrt_lmi(1,1) = - eye(obj.n_z);
    gcrt_lmi(1,5) = obj.c_z * p_inv - obj.d_z_u * k_p_inv;
    gcrt_lmi(2,2) = - e_r * eye(obj.n_p);
    gcrt_lmi(2,5) = obj.c_p * p_inv - obj.d_p_u * k_p_inv;
    gcrt_lmi(3,3) = - e_w * eye(obj.n_y);
    gcrt_lmi(3,5) = obj.c_y * p_inv - obj.d_y_u * k_p_inv;
    gcrt_lmi(3,6) = obj.d_y_r * e_r - obj.d_y_u * l_e_r;
    gcrt_lmi(4,4) = - p_inv + e_w * (obj.b_w * obj.b_w');
    gcrt_lmi(4,5) = obj.a * p_inv - obj.b_u * k_p_inv;
    gcrt_lmi(4,6) = obj.b_r * e_r - obj.b_u * l_e_r;
    gcrt_lmi(5,5) = - p_inv;
    gcrt_lmi(6,6) = - e_r * obj.perf_gamma ^ 2 * eye(obj.n_r);

    % Define GCC cost LMI
    cost_lmi = blkvar;
    cost_lmi(1,1) = - p_inv;
    cost_lmi(1,2) = eye(obj.n_x);
    cost_lmi(2,2) = - s;
    
    % Define YALMIP optimization problem
    constraints = [gcrt_lmi <= 0; 
                   cost_lmi <= 0];
    objective = trace(s);
    options = sdpsettings('solver', obj.options.solver_sdp, 'verbose', 1);
    
    % Solve optimization problem
    solve_out = optimize(constraints, objective, options);
    
    if solve_out.problem ~= 0
        error('SDP solver did not converge, please check if your problem is correct');
    end
    
    % S-Procedure variables
    e_w = value(e_w);
    e_r = value(e_r);
    
    % GCRT cost matrix
    obj.gcc.p = inv(value(p_inv));
    % GCRT gain matrix
    obj.gcc.k = value(k_p_inv) * obj.gcc.p;
    % GCRT feed-forward gain matrix
    obj.gcc.l = value(l_e_r) / e_r;
    % Suboptimal gains, needed to get r_bar
    obj.gcc.x = inv(value(p_inv) - e_w * (obj.b_w * obj.b_w'));
    % For a controller u = - K x - L r + v, the new cost matrix is v' * r_bar * v
    obj.gcc.r_bar = (obj.d_z_u' * obj.d_z_u) + ...
                    e_w^(-1) * (obj.d_y_u' * obj.d_y_u) + ...
                    obj.b_u' * obj.gcc.x * obj.b_u;
    % Set calculation flag
    obj.is_gcc_set = true;
end

