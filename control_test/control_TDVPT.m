function [sim, loss_avg, loss_avg_un, loss_avg_noise] = control_TDVPT(num, budget, underopt, datanoise, noise, failurecase)
    if nargin < 2
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
        model = load(['model_koopman\', 'model_TDVPT.mat']);
    else
        model = load(['model_koopman_tem\', 'model_TDVPT.mat']);
    end
    A = model.koopman_model.model.A;
    B = model.koopman_model.model.B;
    C = model.koopman_model.model.C;
    Fun = model.koopman_model.lift.full;
    n = model.koopman_model.params.nd;
    model_env = load(['model_environment\', 'TDVPT_type-poly_degree-2_delay-2.mat']);
    DynA = model_env.koopman_model.model.A;
    DynB = model_env.koopman_model.model.B;
    DynC = model_env.koopman_model.model.C;
    Fun2 = model_env.koopman_model.lift.full;
    nd = model_env.koopman_model.params.nd;
    
    loss = 0;
    Ts = 0.02;
    
    % the controller parameters obtained after underopt
    if strcmp(underopt, 'No')
        kp = 0.004;
        ki = 0.015;
        kd = 0.0005;
        Np = 15;
        QQ = 800;
        RR = 300;
        PP = 0;
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
    
    % import Gaussian noise 
    dnoise = load(['model_environment\noise\', 'dis_TDVPT.mat']);
    dis1 = dnoise.dis_TDVPT.dis1;
    dis2 = dnoise.dis_TDVPT.dis2;
    dis3 = dnoise.dis_TDVPT.dis3;
    dis4 = dnoise.dis_TDVPT.dis4;
    dis5 = dnoise.dis_TDVPT.dis5;
    dis6 = dnoise.dis_TDVPT.dis6;

    %% Example_Optimized
    w = 0.5;
    targetnum = [1257, 1885, 2514, 629, 1885, 629];
    targetnum = floor(targetnum / 27 * budget);
    [~, sim, ~, ~, ~] = store();
    struct_name = {'t1' 't2' 't3' 't4' 't5' 't6'};

    for jt = 1 : num
        U = [0; 0];
        X = [0; 0];
        delta_inte = [0; 0];
        delta = [0; 0];
        XOri = zeros((n + 1) * length(X) + n, 1);
        X2 = zeros((nd + 1) * length(X) + nd, 1);
        
        loopnum = targetnum(1, jt);
        for i = 1 : loopnum
            t = Ts * (i - 1);
            
            % determination of target state
            if jt == 1
                Disgaus = dis1(i, :)';
                Discons = [-0.3; -1];
                a = 5;
                b = 10;
                Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
            elseif jt == 2
                Disgaus = dis2(i, :)';
                Discons = [0.2; 1];
                a = 4;
                b = 12;
                Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
            elseif jt == 3  
                Disgaus = dis3(i, :)';
                Discons = [-0.5; 0.9];
                a = 3;
                b = 12;
                Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
            elseif jt == 4
                Disgaus = dis4(i, :)';
                Discons = [-0.5; -0.6];
                a = 18;
                b = 3;
                Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
            elseif jt == 5
                Disgaus = dis5(i, :)';
                Discons = [0.8; -0.5];
                R1 = 5;
                r2 = 3;
                d1 = 5;
                Xtarget = 1 * ((R1 - r2) * cos(w * t) + d1 * cos((R1 - r2) / r2 * (w * t)));
                Ytarget = 1 * ((R1 - r2) * sin(w * t) - d1 * sin((R1 - r2) / r2 * (w * t)));
            else
                Disgaus = dis6(i, :)';
                Discons = [0.1; 0.2];
                a = 2;
                b = 1;
                c = 2;
                d = 1;
                Xtarget = 10 * (cos(a * w * t) - cos(b * w * t)^3);
                Ytarget = 10 * (sin(c * w * t) - sin(d * w * t)^3);   
            end
            X_t = [Xtarget; Ytarget];
            
            X2 = SRec(((X'))', i, X2, nd);
            XOri = SRec(((X'))', i, XOri, n);
            X2Lift = Fun2(X2);
            XLift = Fun(XOri);
            [U, delta_inte, delta] = LMPC('TDVPT', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
            X = dynamic(X2Lift, U, DynA, DynB, DynC) + Discons + Disgaus;
            
            [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
            sim.(struct_name{1, jt}).ee = [sim.(struct_name{1, jt}).ee; v];
            sim.(struct_name{1, jt}).X = [sim.(struct_name{1, jt}).X; X'];
            sim.(struct_name{1, jt}).U = [sim.(struct_name{1, jt}).U; U'];
            sim.(struct_name{1, jt}).t = [sim.(struct_name{1, jt}).t; t + Ts];
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
        model = load(['model_environment\', 'TDVPT_type-poly_degree-3_delay-2.mat'] );
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.02;

        kp = 0;
        ki = 0;
        kd = 0;
        Np = 10;
        QQ = 1000;
        RR = 1000;
        PP = 1;

        for jt = 1 : num
            U = [0; 0];
            X = [0; 0];
            delta_inte = [0; 0];
            delta = [0; 0];
            XOri = zeros((n + 1) * length(X) + n, 1);
            X2 = zeros((nd + 1) * length(X) + nd, 1);

            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determination of target state
                if jt == 1
                    Disgaus = dis1(i, :)';
                    Discons = [-0.3; -1];
                    a = 5;
                    b = 10;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 2
                    Disgaus = dis2(i, :)';
                    Discons = [0.2; 1];
                    a = 4;
                    b = 12;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 3  
                    Disgaus = dis3(i, :)';
                    Discons = [-0.5; 0.9];
                    a = 3;
                    b = 12;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 4
                    Disgaus = dis4(i, :)';
                    Discons = [-0.5; -0.6];
                    a = 18;
                    b = 3;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 5
                    Disgaus = dis5(i, :)';
                    Discons = [0.8; -0.5];
                    R1 = 5;
                    r2 = 3;
                    d1 = 5;
                    Xtarget = 1 * ((R1 - r2) * cos(w * t) + d1 * cos((R1 - r2) / r2 * (w * t)));
                    Ytarget = 1 * ((R1 - r2) * sin(w * t) - d1 * sin((R1 - r2) / r2 * (w * t)));
                else
                    Disgaus = dis6(i, :)';
                    Discons = [0.1; 0.2];
                    a = 2;
                    b = 1;
                    c = 2;
                    d = 1;
                    Xtarget = 10 * (cos(a * w * t) - cos(b * w * t)^3);
                    Ytarget = 10 * (sin(c * w * t) - sin(d * w * t)^3);   
                end
                X_t = [Xtarget; Ytarget];

                X2 = SRec(((X'))', i, X2, nd);
                XOri = SRec(((X'))', i, XOri, n);
                X2Lift = Fun2(X2);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('TDVPT', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                X = dynamic(X2Lift, U, DynA, DynB, DynC) + Discons + Disgaus;

                [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_un = [sim.(struct_name{1, jt}).ee_un; v];
                sim.(struct_name{1, jt}).X_un = [sim.(struct_name{1, jt}).X_un; X'];
                sim.(struct_name{1, jt}).U_un = [sim.(struct_name{1, jt}).U_un; U'];
                sim.(struct_name{1, jt}).t_un = [sim.(struct_name{1, jt}).t_un; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_un = sum(sim.(struct_name{1, jt}).ee_un) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_un;
        end
        loss_avg_un = loss / jt; 
    end
    
    %% Example with Data Noise
    if strcmp(datanoise, 'Yes')
        % Preparation for Koopman-based MPC test with data noise
        model = load(['model_koopman\', 'model_TDVPT_noise.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.02;

        kp = 0.003;
        ki = 0.015;
        kd = 0.002;
        Np = 11;
        QQ = 1900;
        RR = 300;
        PP = 9;

        for jt = 1 : num
            U = [0; 0];
            X = [0; 0];
            delta_inte = [0; 0];
            delta = [0; 0];
            XOri = zeros((n + 1) * length(X) + n, 1);
            X2 = zeros((nd + 1) * length(X) + nd, 1);

            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determination of target state
                if jt == 1
                    Disgaus = dis1(i, :)';
                    Discons = [-0.3; -1];
                    a = 5;
                    b = 10;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 2
                    Disgaus = dis2(i, :)';
                    Discons = [0.2; 1];
                    a = 4;
                    b = 12;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 3  
                    Disgaus = dis3(i, :)';
                    Discons = [-0.5; 0.9];
                    a = 3;
                    b = 12;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 4
                    Disgaus = dis4(i, :)';
                    Discons = [-0.5; -0.6];
                    a = 18;
                    b = 3;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 5
                    Disgaus = dis5(i, :)';
                    Discons = [0.8; -0.5];
                    R1 = 5;
                    r2 = 3;
                    d1 = 5;
                    Xtarget = 1 * ((R1 - r2) * cos(w * t) + d1 * cos((R1 - r2) / r2 * (w * t)));
                    Ytarget = 1 * ((R1 - r2) * sin(w * t) - d1 * sin((R1 - r2) / r2 * (w * t)));
                else
                    Disgaus = dis6(i, :)';
                    Discons = [0.1; 0.2];
                    a = 2;
                    b = 1;
                    c = 2;
                    d = 1;
                    Xtarget = 10 * (cos(a * w * t) - cos(b * w * t)^3);
                    Ytarget = 10 * (sin(c * w * t) - sin(d * w * t)^3);   
                end
                X_t = [Xtarget; Ytarget];

                X2 = SRec(((X'))', i, X2, nd);
                XOri = SRec(((X'))', i, XOri, n);
                X2Lift = Fun2(X2);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('TDVPT', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                X = dynamic(X2Lift, U, DynA, DynB, DynC) + Discons + Disgaus;

                [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; v];
                sim.(struct_name{1, jt}).X_noise = [sim.(struct_name{1, jt}).X_noise; X'];
                sim.(struct_name{1, jt}).U_noise = [sim.(struct_name{1, jt}).U_noise; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_noise = sum(sim.(struct_name{1, jt}).ee_noise) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
    
    %% Example with Environmental Disturbance
    if strcmp(noise, 'Yes')
        rng(1)
        model = load(['model_koopman\', 'model_TDVPT.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;
        
        loss = 0;
        Ts = 0.02;

        kp = 0.004;
        ki = 0.015;
        kd = 0.0005;
        Np = 15;
        QQ = 800;
        RR = 300;
        PP = 0;
        
        for jt = 1 : num
            U = [0; 0];
            X = [0; 0];
            delta_inte = [0; 0];
            delta = [0; 0];
            XOri = zeros((n + 1) * length(X) + n, 1);
            X2 = zeros((nd + 1) * length(X) + nd, 1);

            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determination of target state
                if jt == 1
                    Disgaus = dis1(i, :)';
                    Discons = [-0.3; -1];
                    a = 5;
                    b = 10;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 2
                    Disgaus = dis2(i, :)';
                    Discons = [0.2; 1];
                    a = 4;
                    b = 12;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 3  
                    Disgaus = dis3(i, :)';
                    Discons = [-0.5; 0.9];
                    a = 3;
                    b = 12;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 4
                    Disgaus = dis4(i, :)';
                    Discons = [-0.5; -0.6];
                    a = 18;
                    b = 3;
                    Xtarget = (a - b) * cos(w * t) + b * cos(w * t * (a / b - 1)) - a;
                    Ytarget = (a - b) * sin(w * t) - b * sin(w * t * (a / b - 1));
                elseif jt == 5
                    Disgaus = dis5(i, :)';
                    Discons = [0.8; -0.5];
                    R1 = 5;
                    r2 = 3;
                    d1 = 5;
                    Xtarget = 1 * ((R1 - r2) * cos(w * t) + d1 * cos((R1 - r2) / r2 * (w * t)));
                    Ytarget = 1 * ((R1 - r2) * sin(w * t) - d1 * sin((R1 - r2) / r2 * (w * t)));
                else
                    Disgaus = dis6(i, :)';
                    Discons = [0.1; 0.2];
                    a = 2;
                    b = 1;
                    c = 2;
                    d = 1;
                    Xtarget = 10 * (cos(a * w * t) - cos(b * w * t)^3);
                    Ytarget = 10 * (sin(c * w * t) - sin(d * w * t)^3);   
                end
                X_t = [Xtarget; Ytarget];

                X2 = SRec(((X'))', i, X2, nd);
                XOri = SRec(((X'))', i, XOri, n);
                X2Lift = Fun2(X2);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('TDVPT', XLift, U, Np, A, B, C, t, jt, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                if strcmp(failurecase, 'No')
                    amp = 0.25;
                else
                    amp = 2.5;
                end
                X = dynamic(X2Lift, U, DynA, DynB, DynC) + Discons + Disgaus + amp * randn(2, 1);

                [v, ~] = min(sqrt((sim.(struct_name{1, jt}).Xt(:, 1) - X(1, 1)).^2 + (sim.(struct_name{1, jt}).Xt(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; v];
                sim.(struct_name{1, jt}).X_noise = [sim.(struct_name{1, jt}).X_noise; X'];
                sim.(struct_name{1, jt}).U_noise = [sim.(struct_name{1, jt}).U_noise; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_noise = sum(sim.(struct_name{1, jt}).ee_noise) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
end