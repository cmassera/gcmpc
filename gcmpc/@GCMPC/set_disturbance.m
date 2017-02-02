function obj = set_disturbance(obj, b_w, c_y, d_y)
%SET_DISTURBANCE Set disturbance matrices Bw, Cy and Dy
%
%    Input(s):
%    (1) obj - GCMPC class instance
%    (2) b_w - Disturbance input matrix
%    (3) c_y - State to disturbance output matrix
%    (4) d_y - Control input to disturbance output matrix
%
%    Author(s):
%    (1) Carlos M. Massera

    if ~obj.is_system_set
        error('System matrices not set, define them before the disturbances')
    end
    
    if obj.is_disturbance_set
         warning('Disturbance definition is replaced, make sure your code is correct')
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
    
    if size(d_y, 1) ~= n_y
        error('Dy matrix size does not match Cy matrix');
    end
    
    if size(d_y, 2) ~= obj.n_u
        error('Dy matrix size does not match Bu matrix');
    end

    % Set instance variables
    obj.b_w = b_w;
    obj.c_y = c_y;
    obj.d_y = d_y;
    obj.n_w = n_w;
    obj.n_y = n_y;
    obj.is_disturbance_set = true;
end

