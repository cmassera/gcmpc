function obj = set_disturbance(obj, b_w, c_y, d_y_u, d_y_r)
%SET_DISTURBANCE Set disturbance matrices Bw, Cy and Dy
%
%    Input(s):
%    (1) obj   - GCMPC class instance
%    (2) b_w   - Disturbance input matrix
%    (3) c_y   - State to disturbance output matrix
%    (4) d_y_u - Control input to disturbance output matrix
%    (5) d_y_r - Reference input to disturbance output matrix (optional)
%
%    Author(s):
%    (1) Carlos M. Massera

    if ~obj.is_system_set
        error('System matrices not set, define them before the disturbances')
    end
    
    if obj.is_disturbance_set
        warning('Disturbance definition is replaced, make sure your code is correct')
    end
    
    if obj.is_reference_set && (nargin <= 4)
        warning('This system has a reference but no disturbance matrix was defined, assuming zero')
        d_y_r = zeros(size(c_y, 1), obj.n_r);
    end
    
    % Get disturbance sizes
    n_w = size(b_w, 2);
    n_y = size(c_y, 1);
    
    % Check matricies consistency
    if size(b_w, 1) ~= obj.n_x
        error('Bw matrix size does not match A matrix');
    end
    
    if size(c_y, 2) ~= obj.n_x
        error('Cy matrix size does not match A matrix');
    end
    
    if size(d_y_u, 1) ~= n_y
        error('Dyu matrix size does not match Cy matrix');
    end
    
    if size(d_y_u, 2) ~= obj.n_u
        error('Dyu matrix size does not match Bu matrix');
    end
    
    if obj.is_reference_set && (size(d_y_r, 1) ~= n_y)
        error('Dyr matrix size does not match Cy matrix');
    end
    
    if obj.is_reference_set && (size(d_y_r, 2) ~= obj.n_r)
        error('Dyr matrix size does not match Br matrix');
    end

    % Set instance variables
    obj.b_w = b_w;
    obj.c_y = c_y;
    obj.d_y_u = d_y_u;
    obj.n_w = n_w;
    obj.n_y = n_y;
    obj.is_disturbance_set = true;
    
    if obj.is_reference_set
        obj.d_y_r = d_y_r;
    end
end

