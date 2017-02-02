function obj = calculate_nilpotent(obj)
%CALCULATE_NILPOTENT Calculates a nil-potent controller for the system (A, Bu)
%
%    Input(s):
%    (1) obj - GCMPC class instance
%
%    Author(s):
%    (1) Carlos M. Massera

    % Define nil-potent controller matrix
    k_np = sym('k', [obj.n_u, obj.n_x], 'real');

    % Calculate nil-potent controller order
    np_order = obj.n_x - obj.n_u + 1;

    % Nil-potent condition
    constraint = ((obj.a - obj.b_u * k_np) ^ np_order == 0);
    
    % Get resulting gain matrix
    obj.np.k = double(subs(k_np, solve(constraint, k_np)));
    % And resulting closed loop system
    obj.np.a_cl = obj.a - obj.b_u * obj.np.k;

    obj.is_nilpotent_set = true;

end

