function obj = set_system(obj, a, b_u, b_r)
%SET_SYSTEM Defines the system matrices A and B for the generated controller
%
%    Input(s):
%    (1) obj - GCMPC class instance
%    (2) a   - State transition matrix
%    (3) b_u - Control input matrix
%    (4) b_r - Reference input matrix (optional)
%
%    Author(s):
%    (1) Carlos M. Massera

    if obj.is_system_set
        warning('System definition is replaced, make sure your code is correct')
    end
    
    if nargin <= 3
        obj.is_reference_set = false;
    else
        obj.is_reference_set = true;
    end

    % Get system sizes
    n_x = size(a, 2);
    n_u = size(b_u, 2);
    
    if obj.is_reference_set
        n_r = size(b_r, 2);
    end
    
    % Check matrix consistency
    if size(a, 1) ~= n_x
        error('A matrix is not square');
    end
    
    if size(b_u, 1) ~= n_x
        error('Bu matrix size does not match A matrix');
    end
    
    if obj.is_reference_set && (size(b_r, 1) ~= n_x)
        error('Br matrix size does not match A matrix');
    end
    
    % Set instance variables
    obj.a = a;
    obj.b_u = b_u;
    obj.n_x = n_x;
    obj.n_u = n_u;
    obj.is_system_set = true;
    
    if obj.is_reference_set
        obj.n_r = n_r;
        obj.b_r = b_r;
    end

end

