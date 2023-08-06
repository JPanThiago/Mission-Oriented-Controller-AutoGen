function [sim1, sim2, sim3, sim4, sim5] = store()
    % Initialize the structure to store simulation results
    sim1 = struct;
    U = 0;
    X = [0; 0];
    sim1.t1.X = X';
    sim1.t1.U = U';
    sim1.t1.t = 0;
    sim1.t1.ee = 0;
    sim1.t2.X = X';
    sim1.t2.U = U';
    sim1.t2.t = 0;
    sim1.t2.ee = 0;
    sim1.t3.X = X';
    sim1.t3.U = U';
    sim1.t3.t = 0;
    sim1.t3.ee = 0;
    
    sim1.t1.X_un = X';
    sim1.t1.U_un = U';
    sim1.t1.t_un = 0;
    sim1.t1.ee_un = 0;
    sim1.t2.X_un = X';
    sim1.t2.U_un = U';
    sim1.t2.t_un = 0;
    sim1.t2.ee_un = 0;
    sim1.t3.X_un = X';
    sim1.t3.U_un = U';
    sim1.t3.t_un = 0;
    sim1.t3.ee_un = 0;
    
    sim1.t1.X_noise = X';
    sim1.t1.U_noise = U';
    sim1.t1.t_noise = 0;
    sim1.t1.ee_noise = 0;
    sim1.t2.X_noise = X';
    sim1.t2.U_noise = U';
    sim1.t2.t_noise = 0;
    sim1.t2.ee_noise = 0;
    sim1.t3.X_noise = X';
    sim1.t3.U_noise = U';
    sim1.t3.t_noise = 0;
    sim1.t3.ee_noise = 0;
    
    tt=0 : 0.01 : 6.4;
    x_ref = 10 / 180 * pi * cos(tt) + 5 / 180*pi * cos(tt * 2) - 15 / 180 * pi;
    y_ref = -10 / 180 * pi * sin(tt) - 10 / 180*pi * sin(tt * 2);
    sim1.t1.Xt = [x_ref; y_ref]';
    
    tt=0 : 0.01 : 6.4;
    x_ref = 10 / 180 * pi * cos(tt) + 10 / 180 * pi * cos(tt * 3) - 20 / 180 * pi;
    y_ref = -10 / 180 * pi * sin(tt) - 30 / 180 * pi * sin(tt * 3);
    sim1.t2.Xt = [x_ref; y_ref]';

    tt=0 : 0.01 : 12.8;
    x_ref = 10 / 180 * pi * cos(tt) + 10 / 180 * pi * cos(tt * 1.5) - 20 / 180 * pi;
    y_ref = -10 / 180 * pi * sin(tt) - 15 / 180 * pi * sin(tt * 1.5);
    sim1.t3.Xt = [x_ref; y_ref]';

    sim2 = struct;
    U = [0; 0];
    X = [0; 0];
    sim2.t1.X = X';
    sim2.t1.U = U';
    sim2.t1.t = 0;
    sim2.t1.ee = 0;
    sim2.t2.X = X';
    sim2.t2.U = U';
    sim2.t2.t = 0;
    sim2.t2.ee = 0;
    sim2.t3.X = X';
    sim2.t3.U = U';
    sim2.t3.t = 0;
    sim2.t3.ee = 0;
    sim2.t4.X = X';
    sim2.t4.U = U';
    sim2.t4.t = 0;
    sim2.t4.ee = 0;
    sim2.t5.X = X';
    sim2.t5.U = U';
    sim2.t5.t = 0;
    sim2.t5.ee = 0;
    sim2.t6.X = X';
    sim2.t6.U = U';
    sim2.t6.t = 0;
    sim2.t6.ee = 0;
    
    sim2.t1.X_un = X';
    sim2.t1.U_un = U';
    sim2.t1.t_un = 0;
    sim2.t1.ee_un = 0;
    sim2.t2.X_un = X';
    sim2.t2.U_un = U';
    sim2.t2.t_un = 0;
    sim2.t2.ee_un = 0;
    sim2.t3.X_un = X';
    sim2.t3.U_un = U';
    sim2.t3.t_un = 0;
    sim2.t3.ee_un = 0;
    sim2.t4.X_un = X';
    sim2.t4.U_un = U';
    sim2.t4.t_un = 0;
    sim2.t4.ee_un = 0;
    sim2.t5.X_un = X';
    sim2.t5.U_un = U';
    sim2.t5.t_un = 0;
    sim2.t5.ee_un = 0;
    sim2.t6.X_un = X';
    sim2.t6.U_un = U';
    sim2.t6.t_un = 0;
    sim2.t6.ee_un = 0;
    
    sim2.t1.X_noise = X';
    sim2.t1.U_noise = U';
    sim2.t1.t_noise = 0;
    sim2.t1.ee_noise = 0;
    sim2.t2.X_noise = X';
    sim2.t2.U_noise = U';
    sim2.t2.t_noise = 0;
    sim2.t2.ee_noise = 0;
    sim2.t3.X_noise = X';
    sim2.t3.U_noise = U';
    sim2.t3.t_noise = 0;
    sim2.t3.ee_noise = 0;
    sim2.t4.X_noise = X';
    sim2.t4.U_noise = U';
    sim2.t4.t_noise = 0;
    sim2.t4.ee_noise = 0;
    sim2.t5.X_noise = X';
    sim2.t5.U_noise = U';
    sim2.t5.t_noise = 0;
    sim2.t5.ee_noise = 0;
    sim2.t6.X_noise = X';
    sim2.t6.U_noise = U';
    sim2.t6.t_noise = 0;
    sim2.t6.ee_noise = 0;
    
    w = 0.5;
    tt = 0 : pi / 360 : 30 * pi;

    a = 5;
    b = 10;
    X1 = (a - b) * cos(w * tt) + b * cos(w * tt * (a / b - 1)) - a;
    Y1 = (a - b) * sin(w * tt) - b * sin(w * tt * (a / b - 1));
    sim2.t1.Xt = [X1; Y1]';

    a = 4;
    b = 12;
    X1 = (a - b) * cos(w * tt) + b * cos(w * tt * (a / b - 1)) - a;
    Y1 = (a - b) * sin(w * tt) - b * sin(w * tt * (a / b - 1));   
    sim2.t2.Xt = [X1; Y1]';

    a = 3;
    b = 12;
    X1 = (a - b) * cos(w * tt) + b * cos(w * tt * (a / b - 1)) - a;
    Y1 = (a - b) * sin(w * tt) - b * sin(w * tt * (a / b - 1));  
    sim2.t3.Xt = [X1; Y1]';

    a = 18;
    b = 3;
    X1 = (a - b) * cos(w * tt) + b * cos(w * tt * (a / b - 1)) - a;
    Y1 = (a - b) * sin(w * tt) - b * sin(w * tt * (a / b - 1));    
    sim2.t4.Xt = [X1; Y1]';

    R1 = 5;
    r2 = 3;
    d1 = 5;
    X1 = 2 * ((R1 - r2) * cos(w * tt) + d1 * cos((R1 - r2) / r2 * (w * tt)));
    Y1 = 2 * ((R1 - r2) * sin(w * tt) - d1 * sin((R1 - r2) / r2 * (w * tt))); 
    sim2.t5.Xt = [X1; Y1]';

    a = 2;
    b = 1;
    c = 2;
    d = 1;
    X1 = 10 * (cos(a * w * tt) - cos(b * w * tt).^3);
    Y1 = 10 * (sin(c * w * tt) - sin(d * w * tt).^3);    
    sim2.t6.Xt = [X1; Y1]';
    
    sim3 = struct;
    U = [0; 0; 0];
    X = [1; 1];
    sim3.pacman.X = X';
    sim3.pacman.U = U';
    sim3.pacman.t = 0;
    sim3.pacman.ee = 0;
    sim3.star.X = X';
    sim3.star.U = U';
    sim3.star.t = 0;
    sim3.star.ee = 0;
    sim3.blockM.X = X';
    sim3.blockM.U = U';
    sim3.blockM.t = 0;
    sim3.blockM.ee = 0;
    
    sim3.pacman.X_un = X';
    sim3.pacman.U_un = U';
    sim3.pacman.t_un = 0;
    sim3.pacman.ee_un = 0;
    sim3.star.X_un = X';
    sim3.star.U_un = U';
    sim3.star.t_un = 0;
    sim3.star.ee_un = 0;
    sim3.blockM.X_un = X';
    sim3.blockM.U_un = U';
    sim3.blockM.t_un = 0;
    sim3.blockM.ee_un = 0;
    
    sim3.pacman.X_noise = X';
    sim3.pacman.U_noise = U';
    sim3.pacman.t_noise = 0;
    sim3.pacman.ee_noise = 0;
    sim3.star.X_noise = X';
    sim3.star.U_noise = U';
    sim3.star.t_noise = 0;
    sim3.star.ee_noise = 0;
    sim3.blockM.X_noise = X';
    sim3.blockM.U_noise = U';
    sim3.blockM.t_noise = 0;
    sim3.blockM.ee_noise = 0;
    
    sim4 = struct;
    U = [0.2; 0];
    X = [0; 0; 0; 0; 0; 0];
    sim4.t1.X = X';
    sim4.t1.U = U';
    sim4.t1.t = 0;
    sim4.t1.ee = 0;
    sim4.t1.X_un = X';
    sim4.t1.U_un = U';
    sim4.t1.t_un = 0;
    sim4.t1.ee_un = 0;
    sim4.t1.X_noise = X';
    sim4.t1.U_noise = U';
    sim4.t1.t_noise = 0;
    sim4.t1.ee_noise = 0;
    U = [0.2; 0];
    X = [3; 0; pi/2; 0; 0; 0];
    sim4.t2.X = X';
    sim4.t2.U = U';
    sim4.t2.t = 0;
    sim4.t2.ee = 0;
    sim4.t2.X_un = X';
    sim4.t2.U_un = U';
    sim4.t2.t_un = 0;
    sim4.t2.ee_un = 0;
    sim4.t2.X_noise = X';
    sim4.t2.U_noise = U';
    sim4.t2.t_noise = 0;
    sim4.t2.ee_noise = 0;
    U = [0.2; 0];
    X = [0; 0; pi/2; 0; 0; 0];
    sim4.t3.X = X';
    sim4.t3.U = U';
    sim4.t3.t = 0;
    sim4.t3.ee = 0;
    sim4.t3.X_un = X';
    sim4.t3.U_un = U';
    sim4.t3.t_un = 0;
    sim4.t3.ee_un = 0;
    sim4.t3.X_noise = X';
    sim4.t3.U_noise = U';
    sim4.t3.t_noise = 0;
    sim4.t3.ee_noise = 0;
    
    R = 1;
    w = 0.05;
    x0 = 0;
    y0 = -R;
    tt = 0 : 0.01 : 125.6;
    X1 = R * sin(w * tt) + x0;
    Y1 = R * cos(w * tt) + y0;
    sim4.t1.Xt = [X1; Y1]';

    X1 = 3 * cos(w * tt);
    Y1 = sin(w * 2 * tt);
    sim4.t2.Xt = [X1; Y1]';
    
    R = 1.5;
    a = 2;
    b = 1;
    c = 2;
    d = 1;
    X1 = R * (cos(a * w * tt) - cos(b * w * tt).^3);
    Y1 = R * (sin(c * w * tt) - sin(d * w * tt).^3); 
    sim4.t3.Xt = [X1; Y1]';
    
    sim5 = struct;
    U = 0;
    X = [0; 1; 0; 0; 0; 0];
    sim5.t1.X = X';
    sim5.t1.U = U';
    sim5.t1.t = 0;
    sim5.t1.ee = 0;
    
    U = 0;
    X = [1.95; 0; pi/2; 0; 0; 0];
    sim5.t2.X = X';
    sim5.t2.U = U';
    sim5.t2.t = 0;
    sim5.t2.ee = 0;
    
    w = 0.05;
    tt = 0 : 0.01 : 125.6;
    X1 = sin(w * tt);
    Y1 = cos(w * tt);
    sim5.t1.Xt = [X1; Y1]';

    X1 = 1.8 * cos(w * tt) ./ (1 + sin(w * tt) .* sin(w * tt)) + 0.15;
    Y1 = 2.6 * sin(w * tt) .* cos(w * tt) ./ (1 + sin(w * tt) .* sin(w * tt));
    sim5.t2.Xt = [X1; Y1]';
end