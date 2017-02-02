function obj = set_system(obj, a, b_u)
%SET_SYSTEM Defines the system matrices A and B for the generated controller
%
%    Input(s):
%    (1) obj - GCMPC class instance
%    (2) a   - State transition matrix
%    (3) b_u - Control input matrix
%
%    Author(s):
%    (1) Carlos M. Massera

    if obj.is_system_set
        warning('System definition is replaced, make sure your code is correct')
    end

    % Get system sizes
    n_x = size(a, 2);
    n_u = size(b_u, 2);
    
    % Check matrix consistency
    if size(a, 1) ~= n_x
        error('A matrix is not square');
    end
    
    if size(b_u, 1) ~= n_x
        error('Bu matrix size does not match A matrix');
    end
    
    % Set instance variables
    obj.a = a;
    obj.b_u = b_u;
    obj.n_x = n_x;
    obj.n_u = n_u;
    obj.is_system_set = true;

end

