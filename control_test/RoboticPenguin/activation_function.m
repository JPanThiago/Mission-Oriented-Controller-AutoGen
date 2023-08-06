function final_outputs = activation_function(input)
    % the activation function for back-propagation neural networks
    final_outputs = input;
    for i = 1 : length(input)
        final_outputs(i) = 1 / (1.0 + exp(-1.0 * input(i)));
    end
end

