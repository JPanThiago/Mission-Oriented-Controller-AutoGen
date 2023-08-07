%-----------------------------%
%--- Skew-symmetric matrix ---%
%-----------------------------%
function y  = skew(x)
    if isvector(x) && numel(x) == 3
        y = [0 -x(3) x(2); 
             x(3) 0 -x(1); 
             -x(2) x(1) 0];
    end
end
