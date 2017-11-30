function [Xk,Uk] = fetch_optimising_func(optimising_func)
%% wirting MPC optimization function
for loop_index = 1:p
    Xk = control_function(Xk,Uk);
    X_term = transpose(Xk-Xdes)*Qx*Xk;
    U_term = transpose(Uk)*Qu*Uk;
    optimising_func = X_term + U_term;
    
    %plot(Xk)
    %hold on;
end
