function x = HydroDynamic_RoboticPenguin(t, x, u, Cdx, Clx, Cmx, cap, cdp,cah, cdh, cab, cdb, car, f, amp_heave, amp_pitch, Ts)
    % calculate the hydrodynamics of the penguin-inspired robot
    global glideParas ;
    global left right U_old W_old;
    roll = x(4);
    pitch = x(5);
    yaw = x(6);
    U = [x(7); x(8); x(9)]; % linear velocity w.r.t body-frame
    W = [x(10); x(11); x(12)]; % angular velocity w.r.t body-frame

    % rotation angle, angular velocity and angular acceleration of each joint
    lp = u(1);
    rp = u(2);
    lt = u(3);
    rt = u(4);
    tdu = u(5);
    tlr = u(6);

    dlp = 2 * pi * amp_heave * pi / 180 * cos(2 * pi * f * t);
    drp = -2 * pi * amp_heave * pi / 180 * cos(2 * pi * f * t);
    dlt = -2 * pi * amp_pitch * pi / 180 * sin(2 * pi * f * t);
    drt = -2 * pi * amp_pitch * pi / 180 * sin(2 * pi * f * t);

    ddlp = -2 * pi * 2 * pi * amp_heave * pi / 180 * sin(2 * pi * f * t);
    ddrp = 2 * pi * 2 * pi * amp_heave * pi / 180 * sin(2 * pi * f * t);
    ddlt = -2 * pi * 2 * pi * amp_pitch * pi / 180* cos(2 * pi * f * t);
    ddrt = -2 * pi * 2 * pi * amp_pitch * pi / 180* cos(2 * pi * f * t);

    Acce = [U - U_old; W - W_old] / Ts;
    U_old = U;
    W_old = W;
    
    % lift-drag coefficient matrix
    Cd = Cdx * eye(3);
    Cl = Clx * eye(3);
    Cm = Cmx * eye(3);
    
    % action point
    lPt1 = [0.026889; -0.036; 0];
    lPt2 = [0; -0.102965; 0];
    lPt3 = [-0.019154; 0; 0];
    rPt1 = [0.026889; 0.036; 0];
    rPt2 = [0; 0.102965; 0];
    rPt3 = [-0.019154; 0; 0];
    rrm = rPt1 + CvtMatX(-rp) * (rPt2 + CvtMatY(-rt) * rPt3);
    llm = lPt1 + CvtMatX(-lp) * (lPt2 + CvtMatY(-lt) * lPt3);

    %% calculate hydrodynamics
    % hydrodynamics w.r.t body-frame
    bodyHydroFrc = calcBodyHydroForces(U, W, Cd, glideParas.Ab, Cl, Cm, pitch);
    leftWingHydroFrc = calcWingHydroForces(U, W, llm, lp, lt, dlp, dlt, ddlp, ddlt, left, Acce, cap, cdp,cah, cdh, cab, cdb, car);
    rightWingHydroFrc = calcWingHydroForces(U, W, rrm, rp, rt, drp, drt, ddrp, ddrt, right, Acce, cap, cdp,cah, cdh, cab, cdb, car);
    tailWingHydroFrc = calcTailHydroForces(tdu, tlr, U, W);
    Fhydro = bodyHydroFrc + leftWingHydroFrc + rightWingHydroFrc + tailWingHydroFrc;

    %% dynamic equation
    % dynamic evolution of the penguin-inspired robot
    Rb2g = CvtMatZ(yaw) * CvtMatY(pitch) * CvtMatX(roll);
    Rw = cvtBW2IW(roll, pitch);
    Txx = zeros(6, 6);
    Txx(1 : 3, 1 : 3) = Rb2g;
    Txx(4 : 6, 4 : 6) = Rw;

    if t > 0
        F = Fhydro(1 : 3) - cross(W, glideParas.mall * U);
        M = Fhydro(4 : 6) - cross(W, glideParas.J * W);
    else 
        F = zeros(3, 1);
        M = zeros(3, 1);
    end

    dx(1 : 6) = Txx * [U; W];
    dx(7 : 9) = F / glideParas.mall;
    dx(10 : 12) = glideParas.J \ M;

    gAcc = (Rb2g * skew(x(10 : 12))) * (x(7 : 9)) + Rb2g * (dx(7 : 9))';
    x(1 : 3) = x(1 : 3) + (dx(1 : 3))' * Ts + 0.5 * gAcc * Ts * Ts;
    x(4 : 6) = x(4 : 6) + (dx(4 : 6))' * Ts;
    x(7 : 12) = x(7 : 12) + (dx(7 : 12))' * Ts;
end
