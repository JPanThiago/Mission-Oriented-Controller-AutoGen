%----------------------------------%
%--- The function of linear MPC ---%
%----------------------------------%
function [U, delta_inte, delta] = LMPC(robot, XLift, U, Np, ALift, BLift, CLift, t, jt, X_t, kp, ki, kd, delta_inte, delta_old, phi_ref_old, Ts, QQ, RR, PP)
    %% Basic Parameters
    Nx = length(XLift); % the number of state variables
    W_row = 0; % the weight of the relaxation factor
    if strcmp(robot, 'DP')
        Nu = 1; % the number of input variables
        Ny = 2; % the number of output variables
        theta1_ref = zeros(Np + 1, 1);
        dtheta1_ref = zeros(Np + 1, 1);
        delta = X_t - XLift(1 : 2, 1);
    elseif strcmp(robot, 'TDVPT')
        Nu = 2; % the number of input variables
        Ny = 2; % the number of output variables
        x_ref = zeros(Np + 1, 1);
        y_ref = zeros(Np + 1, 1);
        delta = X_t - XLift(1 : 2, 1);
        w = 0.5;
    elseif strcmp(robot, 'SoftRobot')
        Nu = 3; % the number of input variables
        Ny = 2; % the number of output variables
        delta = X_t(1 : 2, 1) - XLift(1 : 2, 1);
    elseif strcmp(robot, 'AUV')
        Nu = 2; % the number of input variables
        Ny = 1; % the number of output variables
        phi_ref = zeros(Np + 1, 1);
        w = 0.05;
        px = XLift(1);
        py = XLift(2);
        phi = X_t(3);
        s1 = (X_t(1) - px) * cos(X_t(3)) + (X_t(2) - py) * sin(X_t(3));
        e1 = -(X_t(1) - px) * sin(X_t(3)) + (X_t(2) - py) * cos(X_t(3));
        delta = atan2(e1, s1) + phi - XLift(3);
    elseif strcmp(robot, 'AUV_3D')
        Nu = 3; % the number of input variables
        Ny = 2; % the number of output variables
        z_ref = zeros(Np + 1, 1);
        phi_ref = zeros(Np + 1, 1);
        w = 0.05;
        h = 1;
        px = XLift(1);
        py = XLift(2);
        pz = XLift(3);
        phi = X_t(6);
        s1 = (X_t(1) - px) * cos(phi) + (X_t(2) - py) * sin(phi);
        e1 = -(X_t(1) - px) * sin(phi) + (X_t(2) - py) * cos(phi);
        delta = [X_t(3) - pz; atan2(e1, s1) + phi - XLift(6)];
    else
        if strcmp(robot, 'RoboticPenguin')
            ALift_tem = ALift;
            BLift_tem = BLift;
            ALift = zeros(length(ALift_tem) + 3);
            ALift(4 : end, 4 : end) = ALift_tem;
            ALift(1 : 3, 4 : 6) = [cos(XLift(3)), -sin(XLift(3)), 0;
                               sin(XLift(3)), cos(XLift(3)), 0;
                               0, 0, 1 ] * Ts;
            ALift(1 : 3, 1 : 3) = ALift(1 : 3, 1 : 3) + eye(3);
            BLift = zeros(size(BLift_tem, 1) + 3, size(BLift_tem, 2));
            BLift(4 : end, :) = BLift_tem;
        end
        Nu = 1; % the number of input variables
        Ny = 1; % the number of output variables
        phi_ref = zeros(Np + 1, 1);
        w = 0.05;
        px = XLift(1);
        py = XLift(2);
        phi = X_t(3);
        s1 = (X_t(1) - px) * cos(X_t(3)) + (X_t(2) - py) * sin(X_t(3));
        e1 = -(X_t(1) - px) * sin(X_t(3)) + (X_t(2) - py) * cos(X_t(3));
        delta = atan2(e1, s1) + phi - XLift(3);
    end
    delta_inte = delta_inte + delta;
    delta_diff = (delta - delta_old) / Ts;
    
    
    %% Generate Reference Trajectory
    if strcmp(robot, 'DP')
        for p = 1 : 1 : Np
            if jt == 1
                Xtarget = 10 / 180 * pi * cos(t + Ts * p - Ts) + 5 / 180 * pi * cos((t + Ts * p - Ts) * 2) - 15 / 180 * pi;
                Ytarget = -10 / 180 * pi * sin(t + Ts * p - Ts) - 5 / 180 * pi * sin((t + Ts * p - Ts) * 2) * 2;
            elseif jt == 2
                Xtarget = 10 / 180 * pi * cos(t + Ts * p - Ts) + 10 / 180 * pi * cos((t + Ts * p - Ts) * 3) - 20 / 180 * pi;
                Ytarget = -10 / 180 * pi * sin(t + Ts * p - Ts) - 30 / 180 * pi * sin((t + Ts * p - Ts) * 3);
            else
                Xtarget = 10 / 180 * pi * cos(t + Ts * p - Ts) + 10 / 180 * pi * cos((t + Ts * p - Ts) * 1.5) - 20 / 180 * pi;
                Ytarget = -10 / 180 * pi * sin(t + Ts * p - Ts) - 15 / 180 * pi * sin((t + Ts * p - Ts) * 1.5);
            end
            theta1_ref(p, 1) = Xtarget + (ki * delta_inte(1, 1) + kd * delta_diff(1, 1) + kp * delta(1, 1)) / Np * (Np - p + 1);
            dtheta1_ref(p, 1) = Ytarget + (ki * delta_inte(2, 1) + kd * delta_diff(2, 1) + kp * delta(2, 1)) / Np * (Np - p + 1);
        end
        ref = [theta1_ref(1 : Np), dtheta1_ref(1 : Np)];
        Yr = reshape(ref', [Np * size(ref, 2), 1]);
    elseif strcmp(robot, 'TDVPT')
        for p = 1 : 1 : Np
            if jt == 1
                a = 5;
                b = 10;
                x_ref(p, 1) = (a - b) * cos(w * (t + Ts * p - Ts)) + b * cos(w * (t + Ts * p - Ts) * (a/b - 1)) - a;
                y_ref(p, 1) = (a - b) * sin(w * (t + Ts * p - Ts)) - b * sin(w * (t + Ts * p - Ts) * (a/b - 1));
            elseif jt == 2  
                a = 4;
                b = 12;
                x_ref(p, 1) = (a - b) * cos(w * (t +  Ts * p - Ts)) + b * cos(w * (t +  Ts * p - Ts) * (a/b - 1)) - a;
                y_ref(p, 1) = (a - b) * sin(w * (t +  Ts * p - Ts)) - b * sin(w * (t +  Ts * p - Ts) * (a/b - 1));
            elseif jt == 3
                a = 3;
                b = 12;
                x_ref(p, 1) = (a - b) * cos(w * (t +  Ts * p - Ts)) + b * cos(w * (t +  Ts * p - Ts) * (a/b - 1)) - a;
                y_ref(p, 1) = (a - b) * sin(w * (t +  Ts * p - Ts)) - b * sin(w * (t +  Ts * p - Ts) * (a/b - 1));
            elseif jt == 4
                a = 18;
                b = 3;
                x_ref(p, 1) = (a - b) * cos(w * (t +  Ts * p - Ts)) + b * cos(w * (t +  Ts * p - Ts) * (a/b - 1)) - a;
                y_ref(p, 1) = (a - b) * sin(w * (t +  Ts * p - Ts)) - b * sin(w * (t +  Ts * p - Ts) * (a/b - 1));
            elseif jt == 5
                R1 = 5;
                r2 = 3;
                d1 = 5;
                x_ref(p, 1) = 1 * ((R1 - r2) * cos(w * (t +  Ts * p - Ts)) + d1 * cos((R1 - r2) / r2 * (w * (t +  Ts * p - Ts))));
                y_ref(p, 1) = 1 * ((R1 - r2) * sin(w * (t +  Ts * p - Ts)) - d1 * sin((R1 - r2) / r2 * (w * (t +  Ts * p - Ts))));
            else
                a = 2;
                b = 1;
                c = 2;
                d = 1;
                x_ref(p, 1) = 10 * (cos(a * w * (t +  Ts * p - Ts)) - cos(b * w * (t +  Ts * p - Ts))^3);
                y_ref(p, 1) = 10 * (sin(c * w * (t +  Ts * p - Ts)) - sin(d * w * (t +  Ts * p - Ts))^3);   
            end
        end
        for p = 1 : 1 : Np
            x_ref(p, 1) = x_ref(p, 1) + (kp * delta(1, 1) + ki * delta_inte(1, 1) + kd * delta_diff(1, 1)) / Np * (Np - p + 1);
            y_ref(p, 1) = y_ref(p, 1) + (kp * delta(2, 1) + ki * delta_inte(2, 1) + kd * delta_diff(2, 1)) / Np * (Np - p + 1);
        end
        ref = [x_ref(1 : Np), y_ref(1 : Np)];
        Yr = reshape(ref' , [Np * size(ref, 2), 1]);
    elseif strcmp(robot, 'SoftRobot')
        Yr = X_t;
        for i = 1 : Np
            Yr(2 * i - 1 : 2 * i, 1) = Yr(2 * i - 1 : 2 * i, 1) + (kp * (Yr(1 : 2, 1) - XLift(1 : 2, 1)) +  ki * delta_inte + kd * delta_diff) / Np * (Np - i + 1);
        end
    elseif strcmp(robot, 'AUV')
        phi_ref(1,1) = phi + ki * delta_inte + kp * delta + kd * delta_diff;
        for p = 1 : 1 : Np - 1
            if jt == 1
                phi_tem = atan2(- sin(w * (t + p * Ts)), cos(w * (t + p * Ts)));
            elseif jt == 2
                if sin(t + p * Ts) == 0
                    if cos(t + p * Ts) == 1
                        phi_tem = pi / 2;
                    else
                        phi_tem = 3 * pi / 2;
                    end
                else
                    phi_tem= atan2(2 * (cos(w * 2 * (t + p * Ts))), - 3 * sin(w * (t + p * Ts)));
                end
            else
                a = 2;
                b = 1;
                c = 2;
                d = 1;
                phi_tem = atan2(c * cos(c * w * (t + p * Ts)) - 3 * d * sin(d * w * (t + p * Ts)) * sin(d * w * (t + p * Ts)) * cos(d * w * (t + p * Ts)), - a * sin(a * w * (t + p * Ts)) + 3 * b * cos(b * w * (t + p * Ts)) * cos(b * w * (t + p * Ts)) * sin(b * w * (t + p * Ts)));
            end
            while phi_tem - phi_ref_old < - pi
                phi_tem = phi_tem + 2 * pi;
            end
            while phi_tem - phi_ref_old > pi
                phi_tem = phi_tem - 2 * pi;
            end
            phi_ref(p + 1, 1) =  phi_tem + (ki * delta_inte + kp * delta + kd * delta_diff) / Np * (Np - p);
        end
        ref = phi_ref(1 : Np);
        Yr = reshape(ref', [Np * size(ref, 2), 1]);
    elseif strcmp(robot, 'AUV_3D')
        z_ref(1,1) = X_t(3) + ki * delta_inte(1, 1) + kp * delta(1, 1) + kd * delta_diff(1, 1);
        phi_ref(1,1) = phi + ki * delta_inte(2, 1) + kp * delta(2, 1) + kd * delta_diff(2, 1);
        for p = 1 : 1 : Np - 1
            phi_tem = atan2(- sin(w * (t + p * Ts)), cos(w * (t + p * Ts)));
            while phi_tem - phi_ref_old < - pi
                phi_tem = phi_tem + 2 * pi;
            end
            while phi_tem - phi_ref_old > pi
                phi_tem = phi_tem - 2 * pi;
            end
            z_ref(p + 1, 1) = X_t(3) + h * w * (p * Ts) + (ki * delta_inte(1, 1) + kp * delta(1, 1) + kd * delta_diff(1, 1)) / Np * (Np - p);
            phi_ref(p + 1, 1) =  phi_tem + (ki * delta_inte(2, 1) + kp * delta(2, 1) + kd * delta_diff(2, 1)) / Np * (Np - p);
        end
        ref = [z_ref(1 : Np), phi_ref(1 : Np)];
        Yr = reshape(ref', [Np * size(ref, 2), 1]);
    else
        phi_ref(1,1) = phi + ki * delta_inte + kp * delta + kd * delta_diff;
        for p = 1 : 1 : Np - 1
            if jt == 1
                phi_tem = atan2(-sin(w * (t + p * Ts)), cos(w * (t + p * Ts)));
            else
                if sin(t + p * Ts) == 0
                    if cos(t + p * Ts) == 1
                        phi_tem = pi / 2;
                    else
                        phi_tem = 3 * pi / 2;
                    end
                else
                    dx = - 1.8 * sin(w * (t + p * Ts)) * (1 + sin(w * (t + p * Ts)) * sin(w * (t + p * Ts))) - 2 * 1.8  * cos(w * (t + p * Ts))^2 * sin(w * (t + p * Ts));
                    dy = 2.6 * (cos(w * (t + p * Ts))^2 - sin(w * (t + p * Ts))^2) * (1 + sin(w * (t + p * Ts)) * sin(w * (t + p * Ts))) - 2 * 2.6 * sin(w * (t + p * Ts))^2 * cos(w * (t + p * Ts))^2;
                    phi_tem = atan2(dy, dx);
                end
            end
            while phi_tem - phi_ref_old < - pi
                phi_tem = phi_tem + 2 * pi;
            end
            while phi_tem - phi_ref_old > pi
                phi_tem = phi_tem - 2 * pi;
            end
            phi_ref(p + 1, 1) =  phi_tem + (ki * delta_inte + kp * delta + kd * delta_diff) / Np * (Np - p);
        end
        ref = phi_ref(1 : Np);
        Yr = reshape(ref', [Np * size(ref, 2), 1]);
    end
    
    %% Calculate the Weight Matrix
    Q_cell = cell(Np, Np);
    for i = 1 : 1 : Np
        for j = 1 : 1 : Np
            if i == j
                Q_cell{i, j} = QQ * eye(Ny);
            else 
                Q_cell{i, j} = zeros(Ny, Ny);               
            end
        end 
    end 
    Q_cell{Np, Np} = RR * eye(Ny);
    R_cell = cell(Np, Np);
    for i = 1 : 1 : Np
        for j = 1 : 1 : Np
            if i == j
                R_cell{i, j} = PP * eye(Nu);
            else
                R_cell{i, j} = zeros(Nu, Nu);
            end
        end
    end
    
    %% Construct State Augmentation Matrix
    A_cell = cell(2, 2);
    for i = 1 : 1 : Np
        A_cell{1, 2} = BLift;
        A_cell{2, 1} = zeros(Nu, Nx);
        A_cell{2, 2} = eye(Nu);
        A_cell{1, 1} = ALift;
        A = cell2mat(A_cell);
    end
    B_cell = cell(2, 1);
    B_cell{1, 1} = BLift;
    B_cell{2, 1} = eye(Nu);
    B = cell2mat(B_cell);
    if strcmp(robot, 'AUV') || strcmp(robot, 'RoboticPenguin') || strcmp(robot, 'RoboticPenguin_full')
        C = zeros(1, length(A));
        C(1, 3) = 1;
    elseif strcmp(robot, 'AUV_3D')
        C = zeros(2, length(A));
        C(1, 3) = 1;
        C(2, 6) = 1;
    else
        CAug = zeros(Ny, Nu);
        C = [CLift, CAug];
    end

    %% Calculate the Matrix Employed to Solve the Linear MPC
    PSI_cell = cell(Np, 1);
    THETA_cell = cell(Np, Np);
    THETA_cell{1, 1} = eye(Nx + Nu);
    PSI_cell{1, 1} = A;
    for i = 2 : 1 : Np
        THETA_cell{i, 1} = A * THETA_cell{i - 1, 1};
        PSI_cell{i, 1} = A * PSI_cell{i-1, 1};
    end
    for j = 2 : 1 : Np
        for i = 1 : 1 : Np
            if j > i
                THETA_cell{i, j} = zeros(Nx + Nu, Nx + Nu);
            else
                THETA_cell{i, j} = THETA_cell{i - 1, j - 1};
            end
        end
    end
    for i = 1 : 1 : Np
        PSI_cell{i, 1} = C * PSI_cell{i, 1};
        for j = 1 : 1 : Np
            THETA_cell{i, j} = C * THETA_cell{i, j} * B;
        end
    end
    
    PSI = cell2mat(PSI_cell);
    THETA = cell2mat(THETA_cell);
    Q = cell2mat(Q_cell);
    RR = cell2mat(R_cell);
    H_cell = cell(2, 2);
    H_cell{1, 1} = THETA' * Q * THETA + RR;
    H_cell{1, 2} = zeros(Nu * Np, 1);
    H_cell{2, 1} = zeros(1, Nu * Np);
    H_cell{2, 2} = W_row;
    H = 2 * cell2mat(H_cell);
    
    kesi = zeros(Nx + Nu, 1);
    kesi(1 : Nx) = XLift;
    kesi(Nx + 1 : end) = U;
    error = PSI * kesi - Yr;
    f_cell = cell(1, 2);
    f_cell{1, 1} = 2 * error' * Q * THETA;
    f_cell{1, 2} = 0;
    f = cell2mat(f_cell);  
    
    %% Determine Constraints
    % input constraints
    A_t = zeros(Np, Np);
    for p = 1 : 1 : Np
        for q = 1 : 1 : Np
            if q <= p 
                A_t(p, q) = 1;
            else 
                A_t(p, q) = 0;
            end
        end 
    end 
    A_I = kron(A_t, eye(Nu));
    Ut = kron(ones(Np, 1), U);
    if strcmp(robot, 'DP')
        umin = [-8];
        umax = [8];
        delta_umin = [-1];
        delta_umax = [1];
    elseif strcmp(robot, 'TDVPT')
        umin = [-90; -90];
        umax = [90; 90];
        delta_umin = [-10; -10];
        delta_umax = [10; 10];
    elseif strcmp(robot, 'SoftRobot')
        umin = [0; 0; 0];
        umax = [10; 10; 10];
        delta_umin = [-3; -3; -3];
        delta_umax = [3; 3; 3];
    elseif strcmp(robot, 'AUV')
        umin = [0.2; -3];
        umax = [0.2; 3];
        delta_umin = [0; -1];
        delta_umax = [0; 1];
    elseif strcmp(robot, 'AUV_3D')
        umin = [0.2; -0.2; -3];
        umax = [0.2; 0.2; 3];
        delta_umin = [0; -0.1; -1];
        delta_umax = [0; 0.1; 1];
    else
        umin = [-90];
        umax = [90];
        delta_umin = [-45];
        delta_umax = [45];
    end
    Umin = kron(ones(Np, 1), umin);
    Umax = kron(ones(Np, 1), umax);
    A_cons_cell = {A_I zeros(Nu * Np, 1); - A_I zeros(Nu * Np, 1)};
    b_cons_cell = {Umax - Ut; - Umin + Ut};
    A_cons = cell2mat(A_cons_cell);
    b_cons = cell2mat(b_cons_cell);
    
    delta_Umin = kron(ones(Np, 1), delta_umin);
    delta_Umax = kron(ones(Np, 1), delta_umax);
    lb = [delta_Umin; 0]; % including the input increment and relaxation factor
    ub = [delta_Umax; 10]; % including the input increment and relaxation factor
    
    %% Start Solving
    opts = optimset('Display','off');
    [X, ~, exitflag] = quadprog(H, f, A_cons, b_cons, [], [], lb, ub, [], opts);
    if exitflag == 1
        if strcmp(robot, 'DP')
            U(1) = U(1) + X(1);
        elseif strcmp(robot, 'TDVPT')
            U = U + X(1 : 2, 1);
        elseif strcmp(robot, 'SoftRobot')
            U = U + X(1 : 3, 1);
        elseif strcmp(robot, 'AUV')
            U(2) = U(2) + X(2);
        elseif strcmp(robot, 'AUV_3D')
            U(2) = U(2) + X(2);
            U(3) = U(3) + X(3);
        else
            U = U + X(1);
        end
    end
end
