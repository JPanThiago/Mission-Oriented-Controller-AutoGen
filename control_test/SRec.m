%--------------------------------------------------------------------------------%
%--- The function of organizing the data for calculating the lifting function ---%
%--------------------------------------------------------------------------------%
function [ XOri ] = SRec(X, i, XOri, n)
    % organize the data for calculating the lifting function
    if n > 0
        if i == 1
            XOri(1 : size(X, 1), 1) = X;
        elseif i > 1 && i <= n
            XOri(size(X, 1) + 1 : size(X, 1) * i, 1) = XOri(size(X, 1) * i - 2 * size(X, 1) + 1 : size(X, 1) * i - size(X, 1), 1);
            XOri(1 : size(X, 1), 1) = X;
        else
            XOri = [X; XOri(1 : size(X, 1) * n, 1)];
        end
    else
        XOri = X;
    end
end
