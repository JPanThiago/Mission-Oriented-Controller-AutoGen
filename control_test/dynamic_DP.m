function [ Y ] = dynamic_DP(t, X, UExe)
    % dynamics of the damping pendulum
    th1 = X(1);
    dth1 = X(2);
    
    g = 9.8;
    l1 = 1;
    m1 = 1;
    b = 1;
    Y = [dth1; -g/l1*sin(th1)-b*dth1/(m1*l1*l1)+cos(th1)*UExe(1,1)/(m1*l1)];
end