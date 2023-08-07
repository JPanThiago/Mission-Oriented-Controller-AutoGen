%----------------------------------------%
%--- The Koopman model-based dynamics ---%
%----------------------------------------%
function X = dynamic(XLift, UExe, A, B, C)
    X1 = A * XLift + B * UExe;
    X = C * X1;
end
