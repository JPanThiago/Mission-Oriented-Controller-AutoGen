%-------------------------------------------------------------%
%--- Kinematics and dynamics of the penguin-inspired robot ---%
%-------------------------------------------------------------%
function x = Dynamic_RoboticPenguin(U, t, x, Ts)
    % parameters of the penguin-inspired robot after system identification
    Cdx = 13.5;
    Clx = 0.6;
    Cmx = 0.22;
    cap = 8.5;
    cdp = 30;
    cah = 0.1;
    cdh = 5.6;
    cab = 0.01;
    cdb = 0.01;
    car = 0.12;
    
    % motion parameters of the penguin-inspired robot
    f = 1;
    amp_heave = 20;
    amp_pitch = 30;

    Q1 = amp_heave * pi / 180 * sin(2 * pi * f * t);
    Q2 = -amp_heave * pi / 180 * sin(2 * pi * f * t);
    if U >= 0
        Q3 = amp_pitch * pi / 180 * cos(2 * pi * f * t );
        Q4 = amp_pitch * pi / 180 * cos(2 * pi * f * t + U * pi / 180);
        u = [Q1, Q2, Q3, Q4, 0, -U * pi / 180];
    else
        Q3 = amp_pitch * pi / 180 * cos(2 * pi * f * t - U * pi / 180);
        Q4 = amp_pitch * pi / 180 * cos(2 * pi * f * t);
        u = [Q1, Q2, Q3, Q4, 0, -U * pi / 180 ];
    end
    
    x = HydroDynamic_RoboticPenguin(t, x, u, Cdx, Clx, Cmx, cap, cdp,cah, cdh, cab, cdb, car, f, amp_heave, amp_pitch, Ts);
    x = [x(1); x(2); 0; 0; 0; x(6); x(7); x(8); 0; 0; 0; x(12)];
end














