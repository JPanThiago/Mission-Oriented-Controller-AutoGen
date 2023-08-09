%-------------------------------------------------------------------------------%
%--- The tail lift-drag coefficient based on back-propagation neural network ---%
%-------------------------------------------------------------------------------%
function final_outputs = Cd_TendonJoint(angle, wih_cfd, hidden_biascfd, who_cfd, final_cfd)
    angle = angle / 1.39626;
    hidden_inputs = wih_cfd * angle;
    hidden_outputs = activation_function(hidden_inputs + hidden_biascfd);
    final_inputs = who_cfd * hidden_outputs;
    final_outputs = final_inputs + final_cfd;
end

