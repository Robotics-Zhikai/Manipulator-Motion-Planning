function [A,B,C] = GetLineABC(P1,P2)
    X1 = P1(1); Y1 = P1(2); 
    X2 = P2(1); Y2 = P2(2); 
    A = Y2 - Y1;
    B = X1 - X2;
    C = X2*Y1 - X1*Y2;
end