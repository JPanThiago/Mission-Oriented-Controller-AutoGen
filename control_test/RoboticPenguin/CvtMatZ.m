function rw2t = CvtMatZ(angle)
    % rotation matrix around the z-axis
    rw2t = [cos(angle), -sin(angle),  0; 
            sin(angle),  cos(angle),  0; 
                     0,           0, 1];
end