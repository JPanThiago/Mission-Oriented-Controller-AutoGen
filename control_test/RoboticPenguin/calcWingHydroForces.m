%--------------------------------------------------------%
%--- Wing hydrodynamics of the penguin-inspired robot ---%
%--------------------------------------------------------%
function F = calcWingHydroForces(U, W, r, lp, lt, dlp, dlt, ddlp, ddlt, flag, Acce, cap, cdp,cah, cdh, cab, cdb, car)
    %{
    U:  linear velocity of body w.r.t body-frame
    W:  angular velocity of body w.r.t body-frame
    F:  6 * 1 force-torque vector
    r:  action point
    %}
    global waterDensity kkz iix Vp Ap kappa1 kappa2 varkappa1 varkappa2 beta;
    F = zeros(6, 1);

    RB2P = [cos(lt),  sin(lt) * sin(lp), -sin(lt) * cos(lp); 
                  0,            cos(lp),            sin(lp); 
            sin(lt), -cos(lt) * sin(lp), cos(lt) * cos(lp)];

    dRB2P = [dlt * -sin(lt), dlt * cos(lt) * sin(lp) + dlp * sin(lt) * cos(lp),  -dlt * cos(lt) * cos(lp) + dlp * sin(lt) * sin(lp); 
                          0,                                    -dlp * sin(lp),                                       dlp * cos(lp); 
              dlt * cos(lt), dlt * sin(lt) * sin(lp) - dlp * cos(lt) * cos(lp), -dlt * sin(lt) * cos(lp) - dlp * cos(lt) * sin(lp)];

    UU = RB2P * (U + cross(W, r));  
    AcceA = RB2P * (Acce(1 : 3) + cross(Acce(4 : 6),r)) + dRB2P * (U + cross(W, r));

    FHp = - waterDensity * cap * ddlt * Vp * kappa1 - 0.5 * waterDensity * cdp * abs(dlt) * dlt * Ap * kappa2;
    if flag == 2
        FHh = - waterDensity * cah * ddlp * Vp * varkappa1 * abs(cos(lt)) - 0.5 * waterDensity * cdh * abs(dlp) * dlp * varkappa2 *cos(lt)^2;
    else
        FHh = -( - waterDensity * cah * ddlp * Vp * varkappa1 * abs(cos(lt)) - 0.5 * waterDensity * cdh * abs(dlp) * dlp * varkappa2 *cos(lt)^2);
    end

    FHb = - waterDensity * cab * Vp * AcceA(3) - 0.5 * waterDensity * cdb * Ap * abs(UU(3)) * UU(3);
    FAr = 0.5 * waterDensity * car * Ap * dlp^2 * abs(cos(lt)) * beta;

    F(1 : 3) = RB2P' * (FHp + FHh + FHb) * kkz + FAr * iix;
    F(4 : 6) =  cross(r, F(1 : 3));
end
