function  uk = runMpcController(Xk,XsetPoint,H,f_func,LB,UB)
    f = f_func(Xk,XsetPoint);
    U = quadprog(H,f,[],[],[],[],LB,UB);
    uk = U(1:4);
end