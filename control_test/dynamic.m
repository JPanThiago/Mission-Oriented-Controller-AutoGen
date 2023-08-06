function [ X ] = dynamic(XLift, UExe, A, B, C)
    % dynamics based on the Koopman model
    X1 = A * XLift + B * UExe;
    X = C * X1;
end