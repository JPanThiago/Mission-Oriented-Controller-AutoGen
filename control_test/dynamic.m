%----------------------------------------------%
%--- The dynamics based on the Koopman model---%
%----------------------------------------------%
function [ X ] = dynamic(XLift, UExe, A, B, C)
    X1 = A * XLift + B * UExe;
    X = C * X1;
end
