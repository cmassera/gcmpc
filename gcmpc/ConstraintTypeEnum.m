classdef ConstraintTypeEnum < int8
    %CONSTRAINTTYPEENUM Enum that defines the type of constraint for the GCMPC
    
    enumeration
        null(1), ...
        standard(2), ...
        robust_invariant(3)
    end
    
end

