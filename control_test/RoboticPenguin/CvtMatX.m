function rw2t = CvtMatX(angle)
    % rotation matrix around the x-axis
    rw2t = [1,          0,            0; 
            0, cos(angle),  -sin(angle); 
            0, sin(angle),  cos(angle)];
end