%---------------------------------------------------%
%--- The control function of the AUV for 3D task ---%
%---------------------------------------------------%
function [sim, loss_avg, loss_avg_un] = control_AUV_3D(budget, underopt)
    if nargin < 1
        budget = 27;
        underopt = 'No';
    end
    warning off
    
    %% Preparation
    % import model
    if strcmp(underopt, 'No')
        model = load(['model_koopman\', 'model_AUV_3D.mat']);
    else
        model = load(['model_koopman_tem\', 'model_AUV_3D.mat']);
    end
    A = model.koopman_model.model.A;
    B = model.koopman_model.model.B;
    C = model.koopman_model.model.C;
    Fun = model.koopman_model.lift.full;
    n = model.koopman_model.params.nd;
    
    loss = 0;
    Ts = 0.05;
    tspan = [0 Ts];
    
    % determine controller parameters
    if strcmp(underopt, 'No')
        % the controller parameters obtained after underopt
        kp = 6;
        ki = 0;
        kd = 0;
        Np = 8;
        QQ = 1500;
        RR = 700;
        PP = 16;
    else
        Parameter_controller = load('parameter_controller.txt');
        kp = Parameter_controller(1);
        ki = Parameter_controller(2);
        kd = Parameter_controller(3);
        Np = Parameter_controller(4);
        QQ = Parameter_controller(5);
        RR = Parameter_controller(6);
        PP = Parameter_controller(7);
    end

    %% Example_Optimized
    % initialization
    targetnum = 4500;
    targetnum = floor(targetnum / 27 * budget);
    w = 0.05;
    
    U = [0.2; 0; 0];
    X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    phi_ref_old = 0;
    delta_inte = [0; 0];
    delta = [0; 0];
    ttlast = 0;
    XOri = zeros((n + 1) * length(X) + n, 1);
    
    sim.ee = [];
    sim.U = [];
    sim.X = [];
    sim.Xt = [];
    sim.tt = [];

    % start robot tasks
    for i = 1 : targetnum
        t = Ts * (i - 1);

        % determine the target state based on the line-of-sight guidance
        XOri = SRec(((X'))', i, XOri, n);
        XLift = Fun(XOri);
        [tt, ee] = LOS('AUV_3D', XLift(1), XLift(2), XLift(3), ttlast, 0);
        if isempty(tt)
            tt = ttlast;
        end
        ttlast = tt;  
        
        R = 1.5;
        h = 1;
        x0 = 0;
        y0 = -R;
        z0 = 0;
        x_ref = R * sin(w * tt) + x0;
        y_ref = R * cos(w * tt) + y0;
        z_ref = h * w * tt + z0;
        phi_ref = atan2(- sin(w * tt), cos(w * tt));
        while phi_ref - phi_ref_old < - pi
            phi_ref = phi_ref + 2 * pi;
        end
        while phi_ref - phi_ref_old > pi
            phi_ref = phi_ref - 2 * pi;
        end
        X_t = [x_ref, y_ref, z_ref, 0, 0, phi_ref];

        % solve input
        [U, delta_inte, delta] = LMPC('AUV_3D', XLift, U, Np, A, B, C, tt, 0, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, Ts, QQ, RR, PP);
        phi_ref_old = phi_ref;

        % update robot states
        [~, y] = ode23(@(t, y) dynamic_AUV_3D(t, y, U), tspan, X);
        X = y(end, :)';

        % store robot state information
        sim.ee = [sim.ee; ee];
        sim.X = [sim.X; X'];
        sim.Xt = [sim.Xt; X_t];
        sim.U = [sim.U; U'];
        sim.tt = [sim.tt; t + Ts];

        % check if the task is completed in advance
        if X(1, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 1000
            break;
        end
    end
    sim.loss = sum(abs(sim.ee)) / i;
    loss = loss + sim.loss;
    loss_avg = loss;
    loss_avg_un = 0;
    
    %% Example_Unoptimized
    if strcmp(underopt, 'No')
        % Preparation for unoptimized Koopman-based MPC test
        model = load(['model_environment\', 'AUV_3D_type-poly_degree-2_delay-1.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.05;
        tspan = [0 Ts];

        % determine controller parameters
        kp = 0;
        ki = 0;
        kd = 0;
        Np = 10;
        QQ = 1000;
        RR = 1000;
        PP = 1;

        % initialization
        U = [0.2; 0; 0];
        X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
        phi_ref_old = 0;
        delta_inte = [0; 0];
        delta = [0; 0];
        ttlast = 0;
        XOri = zeros((n + 1) * length(X) + n, 1);

        sim.ee_un = [];
        sim.U_un = [];
        sim.X_un = [];
        sim.Xt_un = [];
        sim.tt_un = [];

        % start robot tasks
        for i = 1 : targetnum
            t = Ts * (i - 1);

            % determine the target state based on the line-of-sight guidance
            XOri = SRec(((X'))', i, XOri, n);
            XLift = Fun(XOri);
            [tt, ee] = LOS('AUV_3D', XLift(1), XLift(2), XLift(3), ttlast, 0);
            if isempty(tt)
                tt = ttlast;
            end
            ttlast = tt;    

            R = 1.5;
            h = 1;
            x0 = 0;
            y0 = -R;
            z0 = 0;
            x_ref = R * sin(w * tt) + x0;
            y_ref = R * cos(w * tt) + y0;
            z_ref = h * w * tt + z0;
            phi_ref = atan2(- sin(w * tt), cos(w * tt));
            while phi_ref - phi_ref_old < - pi
                phi_ref = phi_ref + 2 * pi;
            end
            while phi_ref - phi_ref_old > pi
                phi_ref = phi_ref - 2 * pi;
            end
            X_t = [x_ref, y_ref, z_ref, 0, 0, phi_ref];

            % solve input
            [U, delta_inte, delta] = LMPC('AUV_3D', XLift, U, Np, A, B, C, tt, 0, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, Ts, QQ, RR, PP);
            phi_ref_old = phi_ref;

            % update robot states
            [~, y] = ode23(@(t, y) dynamic_AUV_3D(t, y, U), tspan, X);
            X = y(end, :)';

            % store robot state information
            sim.ee_un = [sim.ee_un; ee];
            sim.X_un = [sim.X_un; X'];
            sim.Xt_un = [sim.Xt_un; X_t];
            sim.U_un = [sim.U_un; U'];
            sim.tt_un = [sim.tt_un; t + Ts];

            % check if the task is completed in advance
            if X(1, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 1000
                break;
            end
        end
        sim.loss_un = sum(abs(sim.ee_un)) / i;
        loss = loss + sim.loss_un;
        loss_avg_un = loss;
    end
end
