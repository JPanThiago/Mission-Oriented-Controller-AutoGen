%--------------------------------------------------------%
%--- Tail hydrodynamics of the penguin-inspired robot ---%
%--------------------------------------------------------%
function F = calcTailHydroForces(tud, tlr, U, W)
    %{
    tud:  the angle of the tail turning up and down
    tlr:  the angle of the tail turning left and right
    U:  linear velocity of body w.r.t body-frame
    W:  angular velocity of body w.r.t body-frame
    F:  6 * 1 force-torque vector
    %}
    % import the weights of the back-propagation neural network
    wih_cfd1 = load('BP/wih_cfd1.txt');
    hidden_biascfd1 = load('BP/hidden_biascfd1.txt');
    who_cfd1 = load('BP/who_cfd1.txt');
    final_cfd1 = load('BP/final_cfd1.txt');
    wih_cfd2 = load('BP/wih_cfd2.txt');
    hidden_biascfd2 = load('BP/hidden_biascfd2.txt');
    who_cfd2 = load('BP/who_cfd2.txt');
    final_cfd2 = load('BP/final_cfd2.txt');

    wih1 = load('BP/wih1.txt');
    hidden_bias1 = load('BP/hidden_bias1.txt');
    who1 = load('BP/who1.txt');
    final_bias1 = load('BP/final_bias1.txt');
    wih2 = load('BP/wih2.txt');
    hidden_bias2 = load('BP/hidden_bias2.txt');
    who2 = load('BP/who2.txt');
    final_bias2 = load('BP/final_bias2.txt');
    wih3 = load('BP/wih3.txt');
    hidden_bias3 = load('BP/hidden_bias3.txt');
    who3 = load('BP/who3.txt');
    final_bias3 = load('BP/final_bias3.txt');

    % calculate tail rotation angle based on back-propagation neural network
    hp2 = TendonJoint(tud / 1.57, wih1, hidden_bias1, who1, final_bias1);
    hp3 = TendonJoint(tud / 1.57, wih2, hidden_bias2, who2, final_bias2);
    hp4 = TendonJoint(tud / 1.57, wih3, hidden_bias3, who3, final_bias3);
    hy2 = TendonJoint(tlr / 1.57, wih1, hidden_bias1, who1, final_bias1);
    hy3 = TendonJoint(tlr / 1.57, wih2, hidden_bias2, who2, final_bias2);
    hy4 = TendonJoint(tlr / 1.57, wih3, hidden_bias3, who3, final_bias3);

    % action point
    r21 = [-0.191591; 0; 0];
    r2 = [-0.027882; 0; 0];
    r3 = [-0.023794; 0; 0];
    r4 = [-0.04953; 0; 0];

    % cross-sectional area
    S2=[0.014237 0 0;0 0.007442 0;0 0 0.007442];
    S3=[0.007793 0 0;0 0.004643 0;0 0 0.004643];
    S4=[0.002576 + 0.00155 * 2 0 0;0 0.001086 + 0.000731 + 0.001188 0;0 0 0.001086 + 0.000731 + 0.010616 * 2];

    % rotation matrix
    RB22 = CvtMatZ(-hy2) * CvtMatY(-hp2);
    RB23 = CvtMatZ(-hy3) * CvtMatY(-hp3);
    RB24 = CvtMatZ(-hy4) * CvtMatY(-hp4);
    RRB22 = CvtMatZ(hy2) * CvtMatY(hp2);
    RRB23 = CvtMatZ(hy3) * CvtMatY(hp3);
    RRB24 = CvtMatZ(hy4) * CvtMatY(hp4);

    % action point in motion
    r2com = r21 + RRB22 * r2;
    r3com = r21 + RRB22 * [-0.064; 0; 0] + RRB23 * r3;
    r4com = r21 + RRB22 * [-0.064; 0; 0] + RRB23 * [-0.06; 0; 0] + RRB24 * r4;

    V2 = RB22 * (U + skew(W) * r2com);
    V3 = RB23 * (U + skew(W) * r3com);
    V4 = RB24 * (U + skew(W) * r4com);

    F0 = zeros(6, 1);
    halpha0 = 0;
    if abs(V2(1)) > 0
        halpha0 = atan(V2(3) / V2(1));
    end
    F0(1 : 3, 1) = RB22' * S2 * [-Cd_TendonJoint(halpha0, wih_cfd1, hidden_biascfd1, who_cfd1, final_cfd1); 0; -Cd_TendonJoint(halpha0, wih_cfd2, hidden_biascfd2, who_cfd2, final_cfd2)] * V2' * V2;
    F0(4 : 6, 1) = cross(r2com, F0(1 : 3, 1));

    F1 = zeros(6, 1);
    halpha1 = 0;
    if abs(V3(1)) > 0
        halpha1 = atan(V3(3) / V3(1));
    end
    F1(1 : 3, 1) = RB23' * S3 * [-Cd_TendonJoint(halpha1, wih_cfd1, hidden_biascfd1, who_cfd1, final_cfd1); 0; -Cd_TendonJoint(halpha1, wih_cfd2, hidden_biascfd2, who_cfd2, final_cfd2)] * V3' * V3;
    F1(4 : 6, 1) = cross(r3com, F1(1 : 3, 1));

    F2 = zeros(6, 1);
    halpha2 = 0;
    if abs(V4(1)) > 0
        halpha2 = atan(V4(3) / V4(1));
    end
    F2(1 : 3, 1) = RB24' * S4 * [-Cd_TendonJoint(halpha2, wih_cfd1, hidden_biascfd1, who_cfd1, final_cfd1); 0; -Cd_TendonJoint(halpha2, wih_cfd2, hidden_biascfd2, who_cfd2, final_cfd2)] * V4' * V4;
    F2(4 : 6, 1) = cross(r4com, F2(1 : 3, 1));

    F = F0 + F1 + F2;
end



