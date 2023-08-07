%-------------------------------------------------------%
%--- Calculate the rotation matrix around the x-axis ---%
%-------------------------------------------------------%
function rw2t = CvtMatX(angle)
    rw2t = [1,          0,            0; 
            0, cos(angle),  -sin(angle); 
            0, sin(angle),  cos(angle)];
end
