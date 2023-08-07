%-------------------------------------------------------%
%--- Calculate the rotation matrix around the y-axis ---%
%-------------------------------------------------------%
function rw2t = CvtMatY(angle)
    rw2t = [cos(angle), 0,  sin(angle); 
                     0, 1,           0; 
           -sin(angle), 0, cos(angle)];
end
