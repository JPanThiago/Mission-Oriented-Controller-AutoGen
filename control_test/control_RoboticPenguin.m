%------------------------------------------------------------------------------------------------------------------------%
%--- The control function of the penguin-inspired robot based on the three-state model after dimensionality reduction ---%
%------------------------------------------------------------------------------------------------------------------------%
function [sim, loss_avg] = control_RoboticPenguin(budget, underopt)
    if nargin < 1
        budget = 27;
        underopt = 'No';
    end
    warning off
    run('RP_Initialization.m');
    
    %% Preparation
    % import model
    if strcmp(underopt, 'No')
        model = load(['model_koopman\', 'model_RoboticPenguin.mat']);
    else
        model = load(['model_koopman_tem\', 'model_RoboticPenguin.mat']);
    end
    A = model.koopman_model.model.A;
    B = model.koopman_model.model.B;
    C = model.koopman_model.model.C;
    Fun = model.koopman_model.lift.full;
    n = model.koopman_model.params.nd;
    
    loss = 0;
    Ts = 0.01;
    
    % determine controller parameters
    if strcmp(underopt, 'No')
        % the controller parameters obtained after underopt
        kp = 3;
        ki = 1.1;
        kd = 135;
        Np = 13;
        QQ = 1700;
        RR = 800;
        PP = 1;
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
    targetnum = [8000, 14000];
    targetnum = floor(targetnum / 27 * budget);
    [~, ~, ~, ~, sim] = store();
    struct_name = {'t1' 't2'};
    w = 0.05;
    for jt = 1 : 2
        delta_inte = 0;
        delta = 0;
        ttlast = 0;
        U = 0;
        Vel = [0; 0; 0];
        VelOri = zeros((n + 1) * length(Vel) + n, 1);
        x = zeros(12,1);
        num = 30;
        if jt == 1
            x(2) = 1;
            XTem = [0; 1 * num; 0];
            phi_ref_old = 0;
        else
            x(1) = 1.95;
            x(6) = pi/2;
            XTem = [1.95 * num; 0; pi/2 * num];
            phi_ref_old = pi/2;
        end

        % start robot tasks
        for i = 1 : targetnum(1, jt)
            t = Ts * (i - 1);
            if mod(i - 1, num) == 0
                Vel = Vel / num;
                XTem = XTem / num;
                TT = Ts * num;

                % determine the target state based on the line-of-sight guidance
                VelOri = SRec(((Vel'))', i, VelOri, n);
                XLift = [XTem; Fun(VelOri)];
                [tt, ee] = LOS('RoboticPenguin', XLift(1), XLift(2), 0, ttlast, jt);
                if isempty(tt)
                    tt = ttlast;
                end
                ttlast = tt;
                if jt == 1
                    x_ref = sin(w * tt);
                    y_ref = cos(w * tt);
                    phi_ref = atan2(-sin(w * tt), cos(w * tt));
                else
                    x_ref = 1.8 * cos(w * tt) / (1 + sin(w * tt) * sin(w * tt)) + 0.15;
                    y_ref = 2.6 * sin(w * tt) * cos(w * tt) / (1 + sin(w * tt) * sin(w * tt));
                    if sin(tt) == 0
                        if cos(tt) == 1
                            phi_ref = pi / 2;
                        else
                            phi_ref = 3 * pi / 2;
                        end
                    else
                        dx = - 1.8 * sin(w * tt) * (1 + sin(w * tt) * sin(w * tt)) - 2 * 1.8  * cos(w * tt)^2 * sin(w * tt);
                        dy = 2.6 * (cos(w * tt)^2 - sin(w * tt)^2) * (1 + sin(w * tt) * sin(w * tt)) - 2 * 2.6 * sin(w * tt)^2 * cos(w * tt)^2;
                        phi_ref = atan2(dy, dx);
                    end
                end
                while phi_ref - phi_ref_old < - pi
                    phi_ref = phi_ref + 2 * pi;
                end
                while phi_ref - phi_ref_old > pi
                    phi_ref = phi_ref - 2 * pi;
                end
                X_t = [x_ref, y_ref, phi_ref];

                % solve input
                [U, delta_inte, delta] = LMPC('RoboticPenguin', XLift, U, Np, A, B, C, tt, jt, X_t, kp, ki, kd, delta_inte, delta, phi_ref_old, TT, QQ, RR, PP);
                phi_ref_old = phi_ref;
                Vel = [0; 0; 0];
                XTem = [0; 0; 0];
            end

            % update robot states
            x = Dynamic_RoboticPenguin(U, t, x, Ts);
            X = [x(1); x(2); x(6); x(7); x(8); x(12)];

            % store robot state information
            sim.(struct_name{1, jt}).ee = [sim.(struct_name{1, jt}).ee; ee];
            sim.(struct_name{1, jt}).X = [sim.(struct_name{1, jt}).X; X'];
            sim.(struct_name{1, jt}).U = [sim.(struct_name{1, jt}).U; U'];
            sim.(struct_name{1, jt}).t = [sim.(struct_name{1, jt}).t; t + TT];

            Vel = Vel + X(4 : 6);
            XTem = XTem + X(1 : 3);

            % check if the task is completed in advance
            if jt == 1
                if X(1, 1) > 0 && sqrt(X(1, 1)^2 + (X(2, 1) - 1)^2)  < 0.15 && i > 2000
                    break;
                end
            else
                if X(2, 1) > 0 && sqrt(X(2, 1)^2 + (X(1, 1) - 1.95)^2)  < 0.15 && i > 7000
                    break;
                end
            end
        end
        sim.(struct_name{1, jt}).loss = sum(abs(sim.(struct_name{1, jt}).ee)) / i;
        loss = loss + sim.(struct_name{1, jt}).loss;
    end
    loss_avg = loss / jt;
end
