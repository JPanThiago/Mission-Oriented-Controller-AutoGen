%---------------------------------------------------------------------------------------------%
%--- Convert angular velocity w.r.t body-frame to roll/pitch/yaw speed w.r.t inertia-frame ---%
%---------------------------------------------------------------------------------------------%
function R = cvtBW2IW(roll, pitch)
    sinR = sin(roll);
    cosR = cos(roll);

    tanP = tan(pitch);
    cosP = cos(pitch);

    R = [1, sinR * tanP,  cosR * tanP; 
         0,        cosR,        -sinR; 
         0, sinR / cosP, cosR / cosP];
end
