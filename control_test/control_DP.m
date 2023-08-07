%----------------------------------------------------%
%--- The control function of the damping pendulum ---%
%----------------------------------------------------%
function [sim, loss_avg, loss_avg_un, loss_avg_noise] = control_DP(budget, underopt, datanoise, noise, failurecase)
    if nargin < 1
        budget = 27;
        underopt = 'No';
        datanoise = 'No';
        noise = 'Yes';
        failurecase = 'No';
    end
    warning off
    %% Preparation
    % import model
    if strcmp(underopt, 'No')
        model = load(['model_koopman\', 'model_DP.mat']);
    else
        model = load(['model_koopman_tem\', 'model_DP.mat']);
    end
    A = model.koopman_model.model.A;
    B = model.koopman_model.model.B;
    C = model.koopman_model.model.C;
    Fun = model.koopman_model.lift.full;
    n = model.koopman_model.params.nd;
    
    loss = 0;
    Ts = 0.02;
    tspan = [0 Ts];
    
    % the controller parameters obtained after underopt
    if strcmp(underopt, 'No')
        kp = 1.3;
        ki = 0; 
        kd = 0.013;
        QQ = 2000;
        RR = 900;
        PP = 8;
        Np = 8;
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
    targetnum = [315, 315, 629];
    targetnum = floor(targetnum / 27 * budget);
    [sim, ~, ~, ~, ~] = store();
    struct_name = {'t1' 't2' 't3'};
    for jt = 1 : 3
        U = 0;
        delta_inte = [0; 0];
        delta = [0; 0];
        X = [0; 0];
        XOri = zeros((n+1)*length(X) + n, 1);
        
        loopnum = targetnum(1, jt);
        for i = 1 : loopnum
            t = Ts * (i - 1);
            % determination of target state
            if jt == 1
                Xtarget = 10 / 180 * pi * cos(t) + 5 / 180 * pi * cos(t * 2) - 15 / 180 * pi;
                Ytarget = -10 / 180 * pi * sin(t) -5 / 180 * pi * sin(t * 2) * 2;
            elseif jt == 2
                Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 3) - 20 / 180 * pi;
                Ytarget = -10 / 180 * pi * sin(t) - 30 / 180 * pi * sin(t * 3);
            else
                Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 1.5) - 20 / 180 * pi;
                Ytarget = -10 / 180 * pi * sin(t) - 15 / 180 * pi * sin(t * 1.5);
            end
            X_t = [Xtarget; Ytarget];
            
            XOri = SRec(((X'))', i, XOri, n);
            XLift = Fun(XOri);
            [U, delta_inte, delta] = LMPC('DP', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
            [~, y] = ode45(@(t,y) dynamic_DP(t, y, U), tspan, X);
            X = y(end, :)';
            
            [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
            sim.(struct_name{1, jt}).ee = [sim.(struct_name{1, jt}).ee; v];
            sim.(struct_name{1, jt}).X = [ sim.(struct_name{1, jt}).X ; X'];
            sim.(struct_name{1, jt}).U = [ sim.(struct_name{1, jt}).U ; U'];
            sim.(struct_name{1, jt}).t = [sim.(struct_name{1, jt}).t ; t + Ts];
        end
        sim.(struct_name{1, jt}).loss = sum(sim.(struct_name{1, jt}).ee) / i;
        loss = loss + sim.(struct_name{1, jt}).loss;
    end
    loss_avg = loss / jt;
    loss_avg_un = 0;
    loss_avg_noise = 0;
    
    %% Example_Unoptimized
    if strcmp(underopt, 'No')
       % Preparation for unoptimized Koopman-based MPC test
        model = load(['model_environment\', 'DP_type-poly_degree-2_delay-1.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.02;
        tspan = [0 Ts];

        kp = 0;
        ki = 0; 
        kd = 0;
        QQ = 1000;
        RR = 1000;
        PP = 1;
        Np = 10;

        for jt = 1 : 3
            U = 0;
            delta_inte = [0; 0];
            delta = [0; 0];
            X = [0; 0];
            XOri = zeros((n+1)*length(X) + n, 1);
            
            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determination of target state
                if jt == 1
                    Xtarget = 10 / 180 * pi * cos(t) + 5 / 180 * pi * cos(t * 2) - 15 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) -5 / 180 * pi * sin(t * 2) * 2;
                elseif jt == 2
                    Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 3) - 20 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) - 30 / 180 * pi * sin(t * 3);
                else
                    Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 1.5) - 20 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) - 15 / 180 * pi * sin(t * 1.5);
                end
                X_t = [Xtarget; Ytarget];

                XOri = SRec(((X'))', i, XOri, n);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('DP', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                [~, y] = ode45(@(t,y) dynamic_DP(t, y, U), tspan, X);
                X = y(end, :)';

                [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_un = [sim.(struct_name{1, jt}).ee_un; v];
                sim.(struct_name{1, jt}).X_un = [ sim.(struct_name{1, jt}).X_un; X'];
                sim.(struct_name{1, jt}).U_un = [ sim.(struct_name{1, jt}).U_un; U'];
                sim.(struct_name{1, jt}).t_un = [sim.(struct_name{1, jt}).t_un; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_un = sum(sim.(struct_name{1, jt}).ee_un) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_un;
        end
        loss_avg_un = loss / jt;
    end
    
    %% Example with Data Noise
    if strcmp(datanoise, 'Yes')
        model = load(['model_koopman\', 'model_DP_noise.mat']);

        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.02;
        tspan = [0 Ts];
        
        kp = 1.4;
        ki = 0.75; 
        kd = 0.007;
        QQ = 1600;
        RR = 1900;
        PP = 5;
        Np = 15;
        
        for jt = 1 : 3
            U = 0;
            delta_inte = [0; 0];
            delta = [0; 0];
            X = [0; 0];
            XOri = zeros((n+1)*length(X) + n, 1);

            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);
                % determination of target state
                if jt == 1
                    Xtarget = 10 / 180 * pi * cos(t) + 5 / 180 * pi * cos(t * 2) - 15 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) -5 / 180 * pi * sin(t * 2) * 2;
                elseif jt == 2
                    Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 3) - 20 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) - 30 / 180 * pi * sin(t * 3);
                else
                    Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 1.5) - 20 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) - 15 / 180 * pi * sin(t * 1.5);
                end
                X_t = [Xtarget; Ytarget];

                XOri = SRec(((X'))', i, XOri, n);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('DP', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                [~, y] = ode45(@(t,y) dynamic_DP(t, y, U), tspan, X);
                X = y(end, :)';

                [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; v];
                sim.(struct_name{1, jt}).X_noise = [ sim.(struct_name{1, jt}).X_noise ; X'];
                sim.(struct_name{1, jt}).U_noise = [ sim.(struct_name{1, jt}).U_noise ; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise ; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_noise = sum(sim.(struct_name{1, jt}).ee_noise) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
    
    %% Example with Environmental Disturbance
    if strcmp(noise, 'Yes')
        rng(1)
        model = load(['model_koopman\', 'model_DP.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.02;
        tspan = [0 Ts];

        kp = 1.3;
        ki = 0; 
        kd = 0.013;
        QQ = 2000;
        RR = 900;
        PP = 8;
        Np = 8;
        for jt = 1 : 3
            U = 0;
            delta_inte = [0; 0];
            delta = [0; 0];
            X = [0; 0];
            XOri = zeros((n+1)*length(X) + n, 1);

            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);
                % determination of target state
                if jt == 1
                    Xtarget = 10 / 180 * pi * cos(t) + 5 / 180 * pi * cos(t * 2) - 15 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) -5 / 180 * pi * sin(t * 2) * 2;
                elseif jt == 2
                    Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 3) - 20 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) - 30 / 180 * pi * sin(t * 3);
                else
                    Xtarget = 10 / 180 * pi * cos(t) + 10 / 180 * pi * cos(t * 1.5) - 20 / 180 * pi;
                    Ytarget = -10 / 180 * pi * sin(t) - 15 / 180 * pi * sin(t * 1.5);
                end
                X_t = [Xtarget; Ytarget];

                XOri = SRec(((X'))', i, XOri, n);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('DP', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                [~, y] = ode45(@(t,y) dynamic_DP(t, y, U), tspan, X);
                if strcmp(failurecase, 'No')
                    amp = 0.000125; % The unit is radians, corresponding to 0.007°
                else
                    amp = 0.0025; % The unit is radians, corresponding to 0.143°
                end
                X = y(end, :)' + randn(2,1) * amp; 

                [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; v];
                sim.(struct_name{1, jt}).X_noise = [ sim.(struct_name{1, jt}).X_noise ; X'];
                sim.(struct_name{1, jt}).U_noise = [ sim.(struct_name{1, jt}).U_noise ; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise ; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_noise = sum(sim.(struct_name{1, jt}).ee_noise) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
    
end
