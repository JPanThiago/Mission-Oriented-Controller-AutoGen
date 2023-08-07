%--------------------------------------------%
%--- The dynamics of the damping pendulum ---%
%--------------------------------------------%
function Y = dynamic_DP(t, X, UExe)
    theta = X(1);
    dtheta = X(2);
    
    g = 9.8;
    l = 1;
    m = 1;
    b = 1;
    Y = [dtheta; -g / l * sin(theta) - b * dtheta / (m * l * l) + cos(theta) * UExe(1, 1) / (m * l)];
end
