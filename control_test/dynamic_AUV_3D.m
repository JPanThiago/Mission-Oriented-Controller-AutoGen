%-------------------------------------------%
%--- The dynamics of the AUV for 3D task ---%
%-------------------------------------------%
function Y = dynamic_AUV_3D(t, X, UExe)
    phi = X(4);
    theta = X(5);
    psi = X(6);
    u = X(7);
    v = X(8);
    w = X(9);
    p = X(10);
    q = X(11);
    r = X(12);
    
    Xu = 4;
    Du = 0.4;
    Yv = 5;
    Dv = 0.4;
    Zw = 5;
    Dw = 0.8;
    Kp = 5;
    Dp = 1;
    Mq = 5;
    Dq = 1;
    Nr = 1;
    Dr = 0.2;
    m11 = 5.2789;
    m22 = 7.2789;
    m33 = 9;
    m44 = 13;
    m55 = 16;
    m66 = 0.0891;
    
    cphi = cos(phi);
    sphi = sin(phi);
    ctheta = cos(theta);
    stheta = sin(theta);
    cpsi = cos(psi);
    spsi = sin(psi);
    
    C = zeros(12, 12);
    C(1, 7) = cpsi * ctheta;
    C(1, 8) = -spsi * cphi + cpsi * stheta * sphi;
    C(1, 9) = spsi * sphi + cpsi * stheta * cphi;
    C(2, 7) = spsi * ctheta;
    C(2, 8) = cpsi * cphi + spsi * stheta * sphi;
    C(2, 9) = -cpsi * sphi + spsi * stheta * cphi;
    C(3, 7) = -stheta;
    C(3, 8) = ctheta * sphi;
    C(3, 9) = ctheta * cphi;
    C(4, 10) = 1;
    C(4, 11) = sphi * stheta / ctheta;
    C(4, 12) = cphi * stheta / ctheta;
    C(5, 11) = cphi;
    C(5, 12) = -sphi;
    C(6, 11) = sphi / ctheta;
    C(6, 12) = cphi / ctheta;

    C(7, 7) = (-Xu - Du * abs(u)) / m11;
    C(7, 11) = -m33 * w / m11;
    C(7, 12) = m22 * v / m11;
    C(8, 8) = (-Yv - Dv * abs(v)) / m22;
    C(8, 10) = m33 * w / m22;
    C(8, 12) = -m11 * u / m22;
    C(9, 9) = (-Zw - Dw * abs(w)) / m33;
    C(9, 10) = - m22 * v / m33;
    C(9, 11) = m11 * u / m33;

    C(10, 8) = -m33 * w / m44;
    C(10, 9) = m22 * v / m44;
    C(10, 10) = (-Kp - Dp * abs(p)) / m44;
    C(10, 11) = -m66 * r / m44;
    C(10, 12) = m55 * q / m44;

    C(11, 7) = m33 * w / m55;
    C(11, 9) = -m11 * u / m55;
    C(11, 10) = m66 * r / m55;
    C(11, 11) = (-Mq - Dq * abs(q)) / m55;
    C(11, 12) = -m44 * p / m55;

    C(12, 7) = -m22 * v / m66;
    C(12, 8) = m11 * u / m66;
    C(12, 10) = -m55 * q / m66;
    C(12, 11) = m44 * p / m66;
    C(12, 12) = (-Nr - Dr * abs(r)) / m66;
    
    B = zeros(12, 3);
    B(7, 1) = 1 / m11;
    B(9, 2) = 1 / m33;
    B(12, 3) = 1 / m66;
    
    Y = C * X + B * UExe;
end
