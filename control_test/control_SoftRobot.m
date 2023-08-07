%----------------------------------------------%
%--- The control function of the soft robot ---%
%----------------------------------------------%
function [sim, loss_avg, loss_avg_un, loss_avg_noise] = control_softrobot(budget, underopt, datanoise, noise, failurecase)
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
        model = load(['model_koopman\', 'model_SoftRobot.mat']);
    else
        model = load(['model_koopman_tem\', 'model_SoftRobot.mat']);
    end
    A = model.koopman_model.model.A;
    B = model.koopman_model.model.B;
    C = model.koopman_model.model.C;
    Fun = model.koopman_model.lift.full;
    n = model.koopman_model.params.nd;
    model_env = load(['model_environment\', 'SoftRobot_type-poly_degree-2_delay-2.mat']);
    DynA = model_env.koopman_model.model.A;
    DynB = model_env.koopman_model.model.B;
    DynC = model_env.koopman_model.model.C;
    Fun2 = model_env.koopman_model.lift.full;
    nd = model_env.koopman_model.params.nd;

    loss = 0;
    Ts = 0.083023892317701;

    % the controller parameters obtained after underopt
    if strcmp(underopt, 'No')
        kp = 0.06;
        ki = 0.15;
        kd = 0;
        Np = 12;
        QQ = 1300;
        RR = 1700;
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
    dnoise = load(['model_environment\noise\', 'dis_softrobot.mat']);
    disstar = dnoise.dis_softrobot.star;
    dispac = dnoise.dis_softrobot.pacman;
    disbloc = dnoise.dis_softrobot.blockM;

    %% Example_Optimized
    struct_name = {'pacman' 'star' 'blockM'};
    [~, ~, sim, ~, ~] = store();
    address = {'pacman_c11_r3_90sec.mat' 'star_c11_8x8_120sec.mat' 'blockM_c11_6x6_180sec.mat'};
    
    for jt = 1 : 3
        temp = load(['ref_trajectories\', address{1, jt}]);
        ref = temp.ref;
        ref_Ts = resample_ref(ref);
        ref_sc = ref_Ts;
        
        U = [0; 0; 0];
        X = [1; 1];
        XOri = ones((n + 1) * length(X) + n, 1);
        XDyn = ones((nd + 1) * length(X) + nd, 1);
        k = 1;
        delta_inte = [0; 0];
        delta = [0; 0];
        
        while k < floor(size(ref_sc, 1) / 27 * budget)
            t = k * Ts;

            % determination of target state
            if k + Np <= size(ref_sc, 1)
                ref = ref_sc(k : k + Np, :);
            else
                ref = ref_sc(k : end, :); 
            end 
            if size( ref , 2 ) ~= 2
                error('Reference trajectory is not the correct dimension');
            elseif size( ref , 1 ) > Np
                ref = ref( 1 : Np , : ); % remove points over horizon
            elseif size( ref , 1 ) < Np
                ref_temp = kron( ones( Np , 1 ) , ref(end,:) );
                ref_temp( 1 : size(ref,1) , : ) = ref;
                ref = ref_temp; % repeat last point for remainer of horizon
            end
            X_t = reshape(ref', [Np * size(ref, 2), 1]);
            
            XDyn = SRec(((X'))', k, XDyn, nd);
            XOri = SRec(((X'))', k, XOri, n);
            XDynLift = Fun2(XDyn);
            XLift = Fun(XOri);
            [U, delta_inte, delta] = LMPC('SoftRobot', XLift, U, Np, A, B, C, t, 0, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
            X = dynamic(XDynLift, U, DynA, DynB, DynC);
            
            if jt == 1
                Disgaus = dispac(k, :)';
                Discons = [0.2; -0.3];
            elseif jt == 2
                Disgaus = disstar(k, :)';
                Discons = [-0.2; -0.2];
            else
                Disgaus = disbloc(k, :)';
                Discons = [0.1; 0.2];
            end
            X = X + Discons + Disgaus;
            
            k = k + 1;
            [v, ~] = min(sqrt((ref_sc(:, 1) - X(1, 1)).^2 + (ref_sc(:, 2) - X(2, 1)).^2));
            sim.(struct_name{1, jt}).ee = [sim.(struct_name{1, jt}).ee; v];
            sim.(struct_name{1, jt}).X = [sim.(struct_name{1, jt}).X; X'];
            sim.(struct_name{1, jt}).U = [sim.(struct_name{1, jt}).U; U'];
            sim.(struct_name{1, jt}).t = [sim.(struct_name{1, jt}).t; t + Ts];
        end
        sim.(struct_name{1, jt}).Xt = ref_sc;
        sim.(struct_name{1, jt}).loss = sum(sim.(struct_name{1, jt}).ee) / (k - 1);
        loss = loss + sim.(struct_name{1, jt}).loss;
    end
    loss_avg = loss / jt;
    loss_avg_un = 0;
    loss_avg_noise = 0;
    
    %% Example_Unoptimized
    if strcmp(underopt, 'No')
        % Preparation for unoptimized Koopman-based MPC test
        model = load(['model_environment\', 'SoftRobot_type-poly_degree-3_delay-2.mat'] );
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;

        loss = 0;
        Ts = 0.083023892317701;

        kp = 0;
        ki = 0;
        kd = 0;
        Np = 10;
        QQ = 1000;
        RR = 1000;
        PP = 1;

        for jt = 1 : 3
            temp = load(['ref_trajectories\', address{1, jt}]);
            ref = temp.ref;
            ref_Ts = resample_ref(ref);
            ref_sc = ref_Ts;
            U = [0; 0; 0];
            X = [1; 1];
            XOri = ones((n + 1) * length(X) + n, 1);
            XDyn = ones((nd + 1) * length(X) + nd, 1);
            k = 1;
            delta_inte = [0; 0];
            delta = [0; 0];

            while k < floor(size(ref_sc, 1))
                t = k * Ts;

                % determination of target state
                if k + Np <= size(ref_sc, 1)
                    ref = ref_sc(k : k + Np, :);
                else
                    ref = ref_sc(k : end, :); 
                end 
                if size( ref , 2 ) ~= 2
                    error('Reference trajectory is not the correct dimension');
                elseif size( ref , 1 ) > Np
                    ref = ref( 1 : Np , : ); % remove points over horizon
                elseif size( ref , 1 ) < Np
                    ref_temp = kron( ones( Np , 1 ) , ref(end,:) );
                    ref_temp( 1 : size(ref,1) , : ) = ref;
                    ref = ref_temp; % repeat last point for remainer of horizon
                end
                X_t = reshape(ref', [Np * size(ref, 2), 1]);

                XDyn = SRec(((X'))', k, XDyn, nd);
                XOri = SRec(((X'))', k, XOri, n);
                XDynLift = Fun2(XDyn);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('SoftRobot', XLift, U, Np, A, B, C, t, 0, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                X = dynamic(XDynLift, U, DynA, DynB, DynC);

                if jt == 1
                    Disgaus = dispac(k, :)';
                    Discons = [0.2; -0.3];
                elseif jt == 2
                    Disgaus = disstar(k, :)';
                    Discons = [-0.2; -0.2];
                else
                    Disgaus = disbloc(k, :)';
                    Discons = [0.1; 0.2];
                end
                X = X + Discons + Disgaus;

                k = k + 1;
                [v, ~] = min(sqrt((ref_sc(:, 1) - X(1, 1)).^2 + (ref_sc(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_un = [sim.(struct_name{1, jt}).ee_un; v];
                sim.(struct_name{1, jt}).X_un = [sim.(struct_name{1, jt}).X_un; X'];
                sim.(struct_name{1, jt}).U_un = [sim.(struct_name{1, jt}).U_un; U'];
                sim.(struct_name{1, jt}).t_un = [sim.(struct_name{1, jt}).t_un; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_un = sum(sim.(struct_name{1, jt}).ee_un) / (k - 1);
            loss = loss + sim.(struct_name{1, jt}).loss_un;
        end
        loss_avg_un = loss / 3;
    end
    
    %% Example with Data Noise
    if strcmp(datanoise, 'Yes')
        model = load(['model_koopman\', 'model_SoftRobot_noise.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;
        
        loss = 0;
        Ts = 0.083023892317701;
        
        kp = 0.065;
        ki = 0.15;
        kd = 0.014;
        Np = 15;
        QQ = 1900;
        RR = 300;
        PP = 2;
        
        for jt = 1 : 3
            temp = load(['ref_trajectories\', address{1, jt}]);
            ref = temp.ref;
            ref_Ts = resample_ref(ref);
            ref_sc = ref_Ts;

            U = [0; 0; 0];
            X = [1; 1];
            XOri = ones((n + 1) * length(X) + n, 1);
            XDyn = ones((nd + 1) * length(X) + nd, 1);
            k = 1;
            delta_inte = [0; 0];
            delta = [0; 0];

            while k < floor(size(ref_sc, 1) / 27 * budget)
                t = k * Ts;

                % determination of target state
                if k + Np <= size(ref_sc, 1)
                    ref = ref_sc(k : k + Np, :);
                else
                    ref = ref_sc(k : end, :); 
                end 
                if size( ref , 2 ) ~= 2
                    error('Reference trajectory is not the correct dimension');
                elseif size( ref , 1 ) > Np
                    ref = ref( 1 : Np , : ); % remove points over horizon
                elseif size( ref , 1 ) < Np
                    ref_temp = kron( ones( Np , 1 ) , ref(end,:) );
                    ref_temp( 1 : size(ref,1) , : ) = ref;
                    ref = ref_temp; % repeat last point for remainer of horizon
                end
                X_t = reshape(ref', [Np * size(ref, 2), 1]);

                XDyn = SRec(((X'))', k, XDyn, nd);
                XOri = SRec(((X'))', k, XOri, n);
                XDynLift = Fun2(XDyn);
                XLift = Fun(XOri);
                [U, delta_inte, delta] = LMPC('SoftRobot', XLift, U, Np, A, B, C, t, 0, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                X = dynamic(XDynLift, U, DynA, DynB, DynC);

                if jt == 1
                    Disgaus = dispac(k, :)';
                    Discons = [0.2; -0.3];
                elseif jt == 2
                    Disgaus = disstar(k, :)';
                    Discons = [-0.2; -0.2];
                else
                    Disgaus = disbloc(k, :)';
                    Discons = [0.1; 0.2];
                end
                X = X + Discons + Disgaus;

                k = k + 1;
                [v, ~] = min(sqrt((ref_sc(:, 1) - X(1, 1)).^2 + (ref_sc(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; v];
                sim.(struct_name{1, jt}).X_noise = [sim.(struct_name{1, jt}).X_noise; X'];
                sim.(struct_name{1, jt}).U_noise = [sim.(struct_name{1, jt}).U_noise; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_noise = sum(sim.(struct_name{1, jt}).ee_noise) / (k - 1);
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
    
    %% Example with Environmental Disturbance
    if strcmp(noise, 'Yes')
        rng(1)
        model = load(['model_koopman\', 'model_SoftRobot.mat']);
        A = model.koopman_model.model.A;
        B = model.koopman_model.model.B;
        C = model.koopman_model.model.C;
        Fun = model.koopman_model.lift.full;
        n = model.koopman_model.params.nd;
        
        loss = 0;
        Ts = 0.083023892317701;
        
        kp = 0.06;
        ki = 0.15;
        kd = 0;
        Np = 12;
        QQ = 1300;
        RR = 1700;
        PP = 0;
        
        for jt = 1 : 3
            temp = load(['ref_trajectories\', address{1, jt}]);
            ref = temp.ref;
            ref_Ts = resample_ref(ref);
            ref_sc = ref_Ts;

            U = [0; 0; 0];
            X = [1; 1];
            XOri = ones((n + 1) * length(X) + n, 1);
            XDyn = ones((nd + 1) * length(X) + nd, 1);
            k = 1;
            delta_inte = [0; 0];
            delta = [0; 0];

            while k < floor(size(ref_sc, 1) / 27 * budget)
                t = k * Ts;

                % determination of target state
                if k + Np <= size(ref_sc, 1)
                    ref = ref_sc(k : k + Np, :);
                else
                    ref = ref_sc(k : end, :); 
                end 
                if size( ref , 2 ) ~= 2
                    error('Reference trajectory is not the correct dimension');
                elseif size( ref , 1 ) > Np
                    ref = ref( 1 : Np , : ); % remove points over horizon
                elseif size( ref , 1 ) < Np
                    ref_temp = kron( ones( Np , 1 ) , ref(end,:) );
                    ref_temp( 1 : size(ref,1) , : ) = ref;
                    ref = ref_temp; % repeat last point for remainer of horizon
                end
                X_t = reshape(ref', [Np * size(ref, 2), 1]);

                XDyn = SRec(((X'))', k, XDyn, nd);
                XOri = SRec(((X'))', k, XOri, n);
                XDynLift = Fun2(XDyn);
                XLift = Fun(XOri);

                [U, delta_inte, delta] = LMPC('SoftRobot', XLift, U, Np, A, B, C, t, 0, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
                X = dynamic(XDynLift, U, DynA, DynB, DynC);

                if jt == 1
                    Disgaus = dispac(k, :)';
                    Discons = [0.2; -0.3];
                elseif jt == 2
                    Disgaus = disstar(k, :)';
                    Discons = [-0.2; -0.2];
                else
                    Disgaus = disbloc(k, :)';
                    Discons = [0.1; 0.2];
                end
                if strcmp(failurecase, 'No')
                    amp = 0.05;
                else
                    amp = 0.3;
                end
                X = X + Discons + Disgaus + amp * randn(2, 1);

                k = k + 1;
                [v, ~] = min(sqrt((ref_sc(:, 1) - X(1, 1)).^2 + (ref_sc(:, 2) - X(2, 1)).^2));
                sim.(struct_name{1, jt}).ee_noise = [sim.(struct_name{1, jt}).ee_noise; v];
                sim.(struct_name{1, jt}).X_noise = [sim.(struct_name{1, jt}).X_noise; X'];
                sim.(struct_name{1, jt}).U_noise = [sim.(struct_name{1, jt}).U_noise; U'];
                sim.(struct_name{1, jt}).t_noise = [sim.(struct_name{1, jt}).t_noise; t + Ts];
            end
            sim.(struct_name{1, jt}).loss_noise = sum(sim.(struct_name{1, jt}).ee_noise) / (k - 1);
            loss = loss + sim.(struct_name{1, jt}).loss_noise;
        end
        loss_avg_noise = loss / jt;
    end
    
end
