%------------------------------%
%--- Line-of-sight guidance ---%
%------------------------------%
function [tt1, ee] = LOS(robot, px, py, pz, ttlast, jt)
    % determine the target state through the line-of-sight guidance
    if strcmp(robot, 'AUV')
        w = 0.05;
        rr = 0.05;
        tt = 0:0.01:125.6;
        if jt == 1
            R = 1;     
            x0 = 0;
            y0 = -R;
            x = R * sin(w * tt) + x0; 
            y = R * cos(w * tt) + y0;
        elseif jt == 2
            x = 3 * cos(w * tt);
            y = sin(w * 2 * tt);
        else
            R = 1.5;    
            a = 2;
            b = 1;
            c = 2;
            d = 1;
            x = R * (cos(a * w * tt) - cos(b * w * tt).^3);
            y = R * (sin(c * w * tt) - sin(d * w * tt).^3); 
        end
        r1 = sqrt((x - px).^2 + (y - py).^2); 
    elseif strcmp(robot, 'RoboticPenguin')
        w = 0.05;
        tt = 0:0.01:125.6;
        rr = 0.1;
        if jt == 1
            x = sin(w * tt);
            y = cos(w * tt);
        else
            x = 1.8 * cos(w * tt) ./ (1 + sin(w * tt).*sin(w * tt)) + 0.15;
            y = 2.6 * sin(w * tt) .* cos(w * tt) ./ (1 + sin(w * tt).*sin(w * tt));
        end
        r1 = sqrt((x - px).^2 + (y - py).^2); 
    else
        w = 0.05;
        h = 1;
        rr = 0.05;
        tt = 0:0.01:125.6;

        R = 1.5;     
        x0 = 0;
        y0 = -R;
        z0 = 0;
        x = R * sin(w * tt) + x0; 
        y = R * cos(w * tt) + y0;
        z = h * w * tt + z0;
        
        r1 = sqrt((x - px).^2 + (y - py).^2 +(z - pz).^2); 
    end
    
    ee = min(r1);
    if ee > rr
        rr = ee + rr;
    end
    tt0 = [tt(1) tt(end)];                
    R0 = [rr rr];
    [tt1, ~] = polyxpoly(tt, r1, tt0, R0);
    if length(tt1) == 2
        if abs(tt1(1) - tt1(2)) > 100
            if tt1(1) < tt1(2)
                tt1 = tt1(1);
            else
                tt1 = tt1(2);
            end
        elseif tt1(1) < tt1(2)
            tt1 = tt1(2);
        else
            tt1 = tt1(1);
        end
    end
    if length(tt1) > 2
        a = tt1 - ttlast;
        [row, col] = find(a > 0);
        maxa = tt1(row);
        tt1 = min(maxa);
    end
end
