%---------------------------------------%
%--- The control function of the AUV ---%
%---------------------------------------%
function [sim, loss_avg, loss_avg_un, loss_avg_noise] = control_AUV(budget, underopt, datanoise, noise, failurecase)
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
        model = load(['model_koopman\', 'model_AUV.mat']);
    else
        model = load(['model_koopman_tem\', 'model_AUV.mat']);
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
        kp = 3.5;
        ki = 0.07;
        kd = 1;
        Np = 11;
        QQ = 1600;
        RR = 1000;
        PP = 13;
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
    targetnum = [3500, 7500, 10000];
    targetnum = floor(targetnum / 27 * budget);
    [~, ~, ~, sim, ~] = store();
    struct_name = {'t1' 't2' 't3'};
    w = 0.05;
    for jt = 1 : 3
        if jt == 1
            U = [0.2; 0];
            X = [0; 0; 0; 0; 0; 0];
            phi_ref_old = 0;
        elseif jt == 2
            U = [0.2; 0];
            X = [3; 0; pi/2; 0; 0; 0];
            phi_ref_old = pi/2;
        else
            U = [0.2; 0];
            X = [0; 0; pi/2; 0; 0; 0];
            phi_ref_old = pi/2;
        end
        delta_inte = 0;
        delta = 0;
        ttlast = 0;
        XOri = zeros((n + 1) * length(X) + n, 1);
        
        % start robot tasks
        loopnum = targetnum(1, jt);
        for i = 1 : loopnum
            t = Ts * (i - 1);
            
            % determine the target state by the line-of-sight guidance
            XOri = SRec(((X'))', i, XOri, n);
            XLift = Fun(XOri);
            [tt, ee] = LOS('AUV', XLift(1), XLift(2), 0, ttlast, jt);
            if isempty(tt)
                break;
            end
            ttlast = tt;          
            if jt == 1
                R = 1;
                x0 = 0;
                y0 = -R;
                x_ref = R * sin(w * tt) + x0;
                y_ref = R * cos(w * tt) + y0;
                phi_ref = atan2(- sin(w * tt), cos(w * tt));
            elseif jt == 2
                x_ref = 3 * cos(w * tt);
                y_ref = sin(w * 2 * tt);
                if sin(tt) == 0
                    if cos(tt) == 1
                        phi_ref = pi / 2;
                    else
                        phi_ref = 3 * pi / 2;
                    end
                else
                    phi_ref= atan2(2 * (cos(w * 2 * tt)), - 3 * sin(w * tt));
                end
            else
                R = 1.5;
                a = 2;
                b = 1;
                c = 2;
                d = 1;
                x_ref = R * (cos(a * w * tt) - cos(b * w * tt).^3);
                y_ref = R * (sin(c * w * tt) - sin(d * w * tt).^3); 
                phi_ref= atan2(c * cos(c * w * tt)-3 * d * sin(d * w * tt) * sin(d * w * tt) * cos(d * w * tt), - a * sin(a * w * tt) + 3 * b * cos(b * w * tt) * cos(b * w * tt) * sin(b * w * tt));
            end
            while phi_ref - phi_ref_old < - pi
                phi_ref = phi_ref + 2 * pi;
            end
            while phi_ref - phi_ref_old > pi
                phi_ref = phi_ref - 2 * pi;
            end
            X_t = [x_ref, y_ref, phi_ref];
            
            % solve input
            [U, delta_inte, delta] = LMPC('AUV', XLift, U, Np, A, B, C, tt, jt, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, Ts, QQ, RR, PP);
            phi_ref_old = phi_ref;
            
            % update robot states
            [~, y] = ode23(@(t,y) dynamic_AUV(t, y, U), tspan, X);
            X = y(end, :)';
            
            % store robot state information
            sim.(struct_name{1, jt}).ee = [sim.(struct_name{1, jt}).ee; ee];
            sim.(struct_name{1, jt}).X = [sim.(struct_name{1, jt}).X; X'];
            sim.(struct_name{1, jt}).U = [sim.(struct_name{1, jt}).U; U'];
            sim.(struct_name{1, jt}).tt = [sim.(struct_name{1, jt}).t; t + Ts];
            
            % check if the task is completed in advance
            if jt == 1
                if X(1, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 1000
                    break;
                end
            elseif jt == 2
                if X(2, 1) > 0 && sqrt((X(1, 1) - 3)^2 + X(2, 1)^2) < 0.05 && i > 4000
                    break;
                end
            else
                if X(2, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 4000
                    break;
                end
            end
        end
        sim.(struct_name{1, jt}).loss = sum(abs(sim.(struct_name{1, jt}).ee)) / i;
        loss = loss + sim.(struct_name{1, jt}).loss;
    end
    loss_avg = loss / jt;
    loss_avg_un = 0;
    loss_avg_noise = 0;
    
    %% Example_Unoptimized
    if strcmp(underopt, 'No')
        % Preparation for unoptimized Koopman-based MPC test
        model = load(['model_environment\', 'AUV_type-poly_degree-2_delay-1.mat'] );
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
        QQ = 1000;
        RR = 1000;
        PP = 1;
        Np = 10;
        
        % initialization
        for jt = 1 : 3 
            if jt == 1
                U = [0.2; 0];
                X = [0; 0; 0; 0; 0; 0];
                phi_ref_old = 0;
            elseif jt == 2
                U = [0.2; 0];
                X = [3; 0; pi/2; 0; 0; 0];
                phi_ref_old = pi/2;
            else
                U = [0.2; 0];
                X = [0; 0; pi/2; 0; 0; 0];
                phi_ref_old = pi/2;
            end
            delta_inte = 0;
            delta = 0;
            ttlast = 0;
            XOri = zeros((n + 1) * length(X) + n, 1);
            
            % start robot tasks
            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determine the target state by the line-of-sight guidance
                XOri = SRec(((X'))', i, XOri, n);
                XLift = Fun(XOri);
                [tt, ee] = LOS('AUV', XLift(1), XLift(2), 0, ttlast, jt);
                if isempty(tt)
                    break;
                end
                ttlast = tt;
                if jt == 1
                    R = 1;
                    x0 = 0;
                    y0 = -R;
                    x_ref = R * sin(w * tt) + x0;
                    y_ref = R * cos(w * tt) + y0;
                    phi_ref = atan2(- sin(w * tt), cos(w * tt));
                elseif jt == 2
                    x_ref = 3 * cos(w * tt);
                    y_ref = sin(w * 2 * tt);
                    if sin(tt) == 0
                        if cos(tt) == 1
                            phi_ref = pi / 2;
                        else
                            phi_ref = 3 * pi / 2;
                        end
                    else
                        phi_ref= atan2(2 * (cos(w * 2 * tt)), - 3 * sin(w * tt));
                    end
                else
                    R = 1.5;
                    a = 2;
                    b = 1;
                    c = 2;
                    d = 1;
                    x_ref = R * (cos(a * w * tt) - cos(b * w * tt).^3);
                    y_ref = R * (sin(c * w * tt) - sin(d * w * tt).^3); 
                    phi_ref= atan2(c * cos(c * w * tt)-3 * d * sin(d * w * tt) * sin(d * w * tt) * cos(d * w * tt), - a * sin(a * w * tt) + 3 * b * cos(b * w * tt) * cos(b * w * tt) * sin(b * w * tt));
                end
                while phi_ref - phi_ref_old < - pi
                    phi_ref = phi_ref + 2 * pi;
                end
                while phi_ref - phi_ref_old > pi
                    phi_ref = phi_ref - 2 * pi;
                end
                X_t = [x_ref, y_ref, phi_ref];
                
                % solve input
                [U, delta_inte, delta] = LMPC('AUV', XLift, U, Np, A, B, C, tt, jt, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, Ts, QQ, RR, PP);
                phi_ref_old = phi_ref;
                
                % update robot states
                [~, y] = ode23(@(t,y) dynamic_AUV(t, y, U), tspan, X);
                X = y(end, :)';
                
                % store robot state information
                sim.(struct_name{1, jt}).ee_un = [sim.(struct_name{1, jt}).ee_un; ee];
                sim.(struct_name{1, jt}).X_un = [sim.(struct_name{1, jt}).X_un; X'];
                sim.(struct_name{1, jt}).U_un = [sim.(struct_name{1, jt}).U_un; U'];
                sim.(struct_name{1, jt}).tt_un = [sim.(struct_name{1, jt}).t_un; t + Ts];
                
                % check if the task is completed in advance
                if jt == 1
                    if X(1, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 1000
                        break;
                    end
                elseif jt == 2
                    if X(2, 1) > 0 && sqrt((X(1, 1) - 3)^2 + X(2, 1)^2) < 0.05 && i > 4000
                        break;
                    end
                else
                    if X(2, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 4000
                        break;
                    end
                end
            end
            sim.(struct_name{1, jt}).loss_un = sum(abs(sim.(struct_name{1, jt}).ee_un)) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_un;
        end
        loss_avg_un = loss / jt;
    end
    
    %% Example with Data Noise
    if strcmp(datanoise, 'Yes')
        % Preparation for optimized Koopman-based MPC test with sampling data noise
        model = load(['model_koopman\', 'model_AUV_noise.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;
        
        loss = 0;
        Ts = 0.05;
        tspan = [0 Ts];
        
        % determine controller parameters
        kp = 4.5;
        ki = 0.03;
        kd = 1;
        Np = 5;
        QQ = 1200;
        RR = 100;
        PP = 17;
        
        % initialization
        for jt = 1 : 3
            if jt == 1
                U = [0.2; 0];
                X = [0; 0; 0; 0; 0; 0];
                phi_ref_old = 0;
            elseif jt == 2
                U = [0.2; 0];
                X = [3; 0; pi/2; 0; 0; 0];
                phi_ref_old = pi/2;
            else
                U = [0.2; 0];
                X = [0; 0; pi/2; 0; 0; 0];
                phi_ref_old = pi/2;
            end
            delta_inte = 0;
            delta = 0;
            ttlast = 0;
            XOri = zeros((n + 1) * length(X) + n, 1);
            
            % start robot tasks
            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determine the target state by the line-of-sight guidance
                XOri = SRec(((X'))', i, XOri, n);
                XLift = Fun(XOri);
                [tt, ee] = LOS('AUV', XLift(1), XLift(2), 0, ttlast, jt);
                if isempty(tt)
                    tt = ttlast;
                end
                ttlast = tt;          
                if jt == 1
                    R = 1;
                    x0 = 0;
                    y0 = -R;
                    x_ref = R * sin(w * tt) + x0;
                    y_ref = R * cos(w * tt) + y0;
                    phi_ref = atan2(- sin(w * tt), cos(w * tt));
                elseif jt == 2
                    x_ref = 3 * cos(w * tt);
                    y_ref = sin(w * 2 * tt);
                    if sin(tt) == 0
                        if cos(tt) == 1
                            phi_ref = pi / 2;
                        else
                            phi_ref = 3 * pi / 2;
                        end
                    else
                        phi_ref= atan2(2 * (cos(w * 2 * tt)), - 3 * sin(w * tt));
                    end
                else
                    R = 1.5;
                    a = 2;
                    b = 1;
                    c = 2;
                    d = 1;
                    x_ref = R * (cos(a * w * tt) - cos(b * w * tt).^3);
                    y_ref = R * (sin(c * w * tt) - sin(d * w * tt).^3); 
                    phi_ref= atan2(c * cos(c * w * tt)-3 * d * sin(d * w * tt) * sin(d * w * tt) * cos(d * w * tt), - a * sin(a * w * tt) + 3 * b * cos(b * w * tt) * cos(b * w * tt) * sin(b * w * tt));
                end
                while phi_ref - phi_ref_old < - pi
                    phi_ref = phi_ref + 2 * pi;
                end
                while phi_ref - phi_ref_old > pi
                    phi_ref = phi_ref - 2 * pi;
                end
                X_t = [x_ref, y_ref, phi_ref];
                
                % solve input
                [U, delta_inte, delta] = LMPC('AUV', XLift, U, Np, A, B, C, tt, jt, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, Ts, QQ, RR, PP);
                phi_ref_old = phi_ref;
                
                % update robot states
                [~, y] = ode23(@(t,y) dynamic_AUV(t, y, U), tspan, X);
                X = y(end, :)';
                
                % store robot state information
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; ee];
                sim.(struct_name{1, jt}).X_noise = [sim.(struct_name{1, jt}).X_noise; X'];
                sim.(struct_name{1, jt}).U_noise = [sim.(struct_name{1, jt}).U_noise; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise; t + Ts];
                
                % check if the task is completed in advance
                if jt == 1
                    if X(1, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 1000
                        break;
                    end
                elseif jt == 2
                    if X(2, 1) > 0 && sqrt((X(1, 1) - 3)^2 + X(2, 1)^2) < 0.05 && i > 4000
                        break;
                    end
                else
                    if X(2, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 4000
                        break;
                    end
                end
            end
            sim.(struct_name{1, jt}).loss_noise = sum(abs(sim.(struct_name{1, jt}).ee_noise)) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
    
    %% Example with Environmental Disturbance
    if strcmp(noise, 'Yes')
        % Preparation for optimized Koopman-based MPC test with environmental disturbance
        rng(1)
        model = load(['model_koopman\', 'model_AUV.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.05;
        tspan = [0 Ts];
        
        % determine controller parameters
        kp = 3.5;
        ki = 0.07;
        kd = 1;
        Np = 11;
        QQ = 1600;
        RR = 1000;
        PP = 13;
        
        % initialization
        for jt = 1 : 3
            if jt == 1
                U = [0.2; 0];
                X = [0; 0; 0; 0; 0; 0];
                phi_ref_old = 0;
            elseif jt == 2
                U = [0.2; 0];
                X = [3; 0; pi/2; 0; 0; 0];
                phi_ref_old = pi/2;
            else
                U = [0.2; 0];
                X = [0; 0; pi/2; 0; 0; 0];
                phi_ref_old = pi/2;
            end
            delta_inte = 0;
            delta = 0;
            ttlast = 0;
            XOri = zeros((n + 1) * length(X) + n, 1);
            
            % start robot tasks
            loopnum = targetnum(1, jt);
            for i = 1 : loopnum
                t = Ts * (i - 1);

                % determine the target state by the line-of-sight guidance
                XOri = SRec(((X'))', i, XOri, n);
                XLift = Fun(XOri);
                [tt, ee] = LOS('AUV', XLift(1), XLift(2), 0, ttlast, jt);
                if isempty(tt)
                    tt = ttlast;
                end
                if abs(tt - ttlast) > 5
                    tt = ttlast;
                end
                ttlast = tt;          
                if jt == 1
                    R = 1;
                    x0 = 0;
                    y0 = -R;
                    x_ref = R * sin(w * tt) + x0;
                    y_ref = R * cos(w * tt) + y0;
                    phi_ref = atan2(- sin(w * tt), cos(w * tt));
                elseif jt == 2
                    x_ref = 3 * cos(w * tt);
                    y_ref = sin(w * 2 * tt);
                    if sin(tt) == 0
                        if cos(tt) == 1
                            phi_ref = pi / 2;
                        else
                            phi_ref = 3 * pi / 2;
                        end
                    else
                        phi_ref= atan2(2 * (cos(w * 2 * tt)), - 3 * sin(w * tt));
                    end
                else
                    R = 1.5;
                    a = 2;
                    b = 1;
                    c = 2;
                    d = 1;
                    x_ref = R * (cos(a * w * tt) - cos(b * w * tt).^3);
                    y_ref = R * (sin(c * w * tt) - sin(d * w * tt).^3); 
                    phi_ref= atan2(c * cos(c * w * tt)-3 * d * sin(d * w * tt) * sin(d * w * tt) * cos(d * w * tt), - a * sin(a * w * tt) + 3 * b * cos(b * w * tt) * cos(b * w * tt) * sin(b * w * tt));
                end
                while phi_ref - phi_ref_old < - pi
                    phi_ref = phi_ref + 2 * pi;
                end
                while phi_ref - phi_ref_old > pi
                    phi_ref = phi_ref - 2 * pi;
                end
                X_t = [x_ref, y_ref, phi_ref];
                
                % solve input
                [U, delta_inte, delta] = LMPC('AUV', XLift, U, Np, A, B, C, tt, jt, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, Ts, QQ, RR, PP);
                phi_ref_old = phi_ref;
                
                % update robot states with environmental disturbance
                [~, y] = ode23(@(t,y) dynamic_AUV(t, y, U), tspan, X);
                if strcmp(failurecase, 'No')
                    amp = 0.003;
                else
                    amp = 0.03;
                end
                X = y(end, :)'+ amp * randn(6, 1);
                
                % store robot state information
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; ee];
                sim.(struct_name{1, jt}).X_noise = [sim.(struct_name{1, jt}).X_noise; X'];
                sim.(struct_name{1, jt}).U_noise = [sim.(struct_name{1, jt}).U_noise; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise; t + Ts];
                
                % check if the task is completed in advance
                if jt == 1
                    if X(1, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 1000
                        break;
                    end
                elseif jt == 2
                    if X(2, 1) > 0 && sqrt((X(1, 1) - 3)^2 + X(2, 1)^2) < 0.05 && i > 4000
                        break;
                    end
                else
                    if X(2, 1) > 0 && sqrt(X(1, 1)^2 + X(2, 1)^2) < 0.05 && i > 4000
                        break;
                    end
                end
            end
            sim.(struct_name{1, jt}).loss_noise = sum(abs(sim.(struct_name{1, jt}).ee_noise)) / i;
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
end
