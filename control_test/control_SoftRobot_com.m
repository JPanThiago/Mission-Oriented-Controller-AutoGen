%---------------------------------------------------------------------------------------------------------------------------------------%
%--- The control function of the soft robot where the Koopman-based control model is identical to the Koopman-based interaction model---%
%---------------------------------------------------------------------------------------------------------------------------------------%
function [sim, loss_avg_un] = control_SoftRobot_com()
    %% Example of the soft robot for comparison
    % Preparation
    struct_name = {'pacman' 'star' 'blockM'};
    [~, ~, sim, ~, ~] = store();
    address = {'pacman_c11_r3_90sec.mat' 'star_c11_8x8_120sec.mat' 'blockM_c11_6x6_180sec.mat'};
    
    % import model
    model = load(['model_environment\', 'SoftRobot_type-poly_degree-2_delay-2.mat'] );
    A = model.koopman_model.model.A;
    B = model.koopman_model.model.B;
    C = model.koopman_model.model.C;
    Fun = model.koopman_model.lift.full;
    n = model.koopman_model.params.nd;
    DynA = A;
    DynB = B;
    DynC = C;
    Fun2 = Fun;
    nd = n;

    loss = 0;
    Ts = 0.083023892317701;

    % determine controller parameters
    kp = 0;
    ki = 0;
    kd = 0;
    Np = 10;
    QQ = 1000;
    RR = 1000;
    PP = 1;

    % initialization
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

        % start robot tasks
        while k < floor(size(ref_sc, 1))
            t = k * Ts;

            % determine the target state
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

            % solve input
            XDyn = SRec(((X'))', k, XDyn, nd);
            XOri = SRec(((X'))', k, XOri, n);
            XDynLift = Fun2(XDyn);
            XLift = Fun(XOri);
            [U, delta_inte, delta] = LMPC('SoftRobot', XLift, U, Np, A, B, C, t, 0, X_t, kp, ki, kd, delta_inte, delta, 0, Ts, QQ, RR, PP);
            
            % update robot states
            X = dynamic(XDynLift, U, DynA, DynB, DynC);

            k = k + 1;

            % store robot state information
            [v, ~] = min(sqrt((ref_sc(:, 1) - X(1, 1)).^2 + (ref_sc(:, 2) - X(2, 1)).^2));
            sim.(struct_name{1, jt}).ee_un = [sim.(struct_name{1, jt}).ee_un; v];
            sim.(struct_name{1, jt}).X_un = [sim.(struct_name{1, jt}).X_un; X'];
            sim.(struct_name{1, jt}).U_un = [sim.(struct_name{1, jt}).U_un; U'];
            sim.(struct_name{1, jt}).t_un = [sim.(struct_name{1, jt}).t_un; t + Ts];
        end
        sim.(struct_name{1, jt}).Xt = ref_sc;
        sim.(struct_name{1, jt}).loss_un = sum(sim.(struct_name{1, jt}).ee_un) / (k - 1);
        loss = loss + sim.(struct_name{1, jt}).loss_un;
    end
    loss_avg_un = loss / jt;
end
