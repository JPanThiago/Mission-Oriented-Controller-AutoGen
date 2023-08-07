%-------------------------------%
%--- The dynamics of the AUV ---%
%-------------------------------%
function Y = dynamic_AUV(t, X, UExe)
    phi = X(3);
    u = X(4);
    v = X(5);
    r = X(6);
    
    Xu = 4;
    Du = 0.4;
    Yv = 5;
    Dv = 0.4;
    Nr = 1;
    Dr = 0.2;
    m11 = 5.2789;
    m22 = 7.2789;
    m33 = 0.0891;
    
    A1 = [cos(phi), -sin(phi), 0;
          sin(phi),  cos(phi), 0;
                 0,         0, 1];
    A2 = [(-Xu - Du * abs(u)) / m11,                         0,              m22 / m11 * v;
                                  0, (-Yv - Dv * abs(v)) / m22,             -m11 / m22 * u;
                     -m22 / m33 * v,             m11 / m33 * u, (-Nr - Dr * abs(r)) / m33];         
    A = [zeros(3, 3),  A1; 
         zeros(3, 3), A2];
    B = zeros(6, 2);
    B(4, 1) = 1 / m11;
    B(6, 2) = 1 / m33;
    
    Y = A * X + B * UExe;
end
