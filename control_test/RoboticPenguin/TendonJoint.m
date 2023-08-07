%--------------------------------------------------------------------------%
%--- The tail rotation angle based on a back-propagation neural network ---%
%--------------------------------------------------------------------------%
function final_outputs = TendonJoint(angle,wih_cfd1, hidden_biascfd1, who_cfd1, final_cfd1)
    hidden_inputs = wih_cfd1 * angle;
    hidden_outputs = activation_function(hidden_inputs + hidden_biascfd1);
    final_inputs = who_cfd1 * hidden_outputs;
    final_outputs = final_inputs + final_cfd1;
    final_outputs = -final_outputs / 180 * pi;
end
