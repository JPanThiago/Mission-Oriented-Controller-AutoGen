function D = matrix_D(Parameter_lifting, Nx)
    % calculate the matrix D delivered by BOHB for Koopman operator optimization
    D = eye(length(Parameter_lifting) + Nx);
    D1 = diag(D);
    D1(Nx + 1 : length(Parameter_lifting) + Nx, 1) = Parameter_lifting';
    D(logical(eye(length(Parameter_lifting) + Nx))) = D1;
    i = find(sum(D, 1) == 0);
    D(i, :) = [];
end