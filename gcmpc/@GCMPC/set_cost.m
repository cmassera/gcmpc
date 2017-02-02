function obj = set_cost(obj, q, r, n)
%SET_SYSTEM Defines the system matrices A and B for the generated controller
%
%    Input(s):
%    (1) obj - GCMPC class instance
%    (2) q   - State cost matrix
%    (3) r   - Control input cost matrix
%    (4) n   - State/input cross cost matrix (optional)
%
%    Author(s):
%    (1) Carlos M. Massera

    if ~obj.is_system_set
        error('System matrices not set, define them before the disturbances')
    end
    
    if obj.is_cost_set
         warning('Cost definition is replaced, make sure your code is correct')
    end
    
    if nargin <= 3
        n = zeros(obj.n_x, obj.n_u);
    end
    
    % Check matrix consistency
    if any(size(q) ~= obj.n_x)
        error('Q matrix size does not match A matrix');
    end
    
    if any(size(r) ~= obj.n_u)
        error('R matrix size does not match B matrix');
    end
    
    if any(size(n) ~= [obj.n_x, obj.n_u])
        error('N matrix size does not match A and B matrices');
    end
    
    if any(eig(r) < obj.kPosDefTest)
        error('R matrix is not positive definite');
    end
    
    % Set weight matrix
    w = [q  n;
         n' r];
     
    if any(any(abs(w - w') >= obj.kSymTest))
        error('Weight is not symmetric');
    end
    
    if any(eig(w) < -obj.kPosDefTest)
        error('Weight is not positive semi-definite');
    end
    
    % Decompose weight matrix
    [~, s, vt] = svd(w);
    mask = (diag(s) >= obj.kPosDefTest);

    w_half = sqrt(s(mask, mask)) * vt(:, mask)';
	c_z = w_half(:,1:obj.n_x);
    d_z = w_half(:,obj.n_x+1:end);
    
    % Set instance variables
    obj.c_z = c_z;
    obj.d_z = d_z;
    obj.n_z = size(c_z, 1);
    obj.is_cost_set = true;
end

