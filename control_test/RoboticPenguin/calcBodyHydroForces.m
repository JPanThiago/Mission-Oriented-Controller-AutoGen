%--------------------------------------------------------%
%--- Body hydrodynamics of the penguin-inspired robot ---%
%--------------------------------------------------------%
function F = calcBodyHydroForces(U, W, Cd, Ab, Cl, Cm, pitch)
    %{
    U:      linear velocity of body w.r.t body-frame
    F:      6 * 1 force-torque vector
    Fl:     lift force
    Fd:     drag force
    alpha:  attack angle
    pitch:  pitch angle
    %}
    global waterDensity jjy;
    F = zeros(6, 1);
    alpha = 0;
    if abs(U(1)) > 0
        alpha = pitch + atan(U(3) / U(1));
    end

    U1 = norm(U);
    if U1 == 0
        U2 = zeros(3, 1);
    else
        U2 = U / norm(U);
    end

    Sb = U2.'* Ab * U2;
    W1 = norm(W);
    if W1 == 0
        W2 = zeros(3, 1);
    else
        W2 = W / norm(W);
    end

    Fd = - 0.5 * waterDensity * Sb * U1^2 * Cd * U2; 
    Fl = - 0.5 * waterDensity * Sb * U1^2 * Cl * cross(U2, jjy) * alpha;
    Md = - Cm * W1^2 * W2 ;
    F(1 : 3) = Fd + Fl;
    F(4 : 6) = Md;
end


