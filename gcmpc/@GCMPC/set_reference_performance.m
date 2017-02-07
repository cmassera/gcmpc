function obj = set_reference_performance(obj, c_p, d_p_u, gamma)
%SET_DISTURBANCE Set disturbance matrices Bw, Cy and Dy
%
%    Input(s):
%    (1) obj   - GCMPC class instance
%    (3) c_p   - State to performance output matrix
%    (4) d_p_u - Control input to performance output matrix
%    (5) gamma - Maximum H infinity norm allowed
%
%    Author(s):
%    (1) Carlos M. Massera

    if ~obj.is_system_set
        error('System matrices not set, define them before the disturbances')
    end
    
    if ~obj.is_reference_set
        error('Reference matrices not set, define a system with references to use this function')
    end
    
    % Get disturbance sizes
    n_p = size(c_p, 1);
    
    % Check matricies consistency
    if size(c_p, 2) ~= obj.n_x
        error('Cp matrix size does not match A matrix');
    end
    
    if size(d_p_u, 1) ~= n_p
        error('Dpu matrix size does not match Cy matrix');
    end
    
    if size(d_p_u, 2) ~= obj.n_u
        error('Dpu matrix size does not match Bu matrix');
    end
    
    if gamma <= 0
        error('Performance requirement is ill-defined')
    end

    % Set instance variables
    obj.c_p = c_p;
    obj.d_p_u = d_p_u;
    obj.n_p = n_p;
    obj.perf_gamma = gamma;
    obj.is_reference_performance_set = true;
end

