 p = 10;
 m = 8;
 N = 10; %number of steps to control, undefined for online process
 fetchModel;
 
 %% setting Initial params
Xk = zeros(12,1);
Xdes = zeros(12,1); 
Xdes(:,1) = 2;
Qx = eye(12);
Qu = eye(4);

%% finding control inputs
for i = 1:N
    Xk_next = model_function(Xk_next,Uk);
    
    optimising_func = fetch_optimising_func(Xk_next,Uk);
    U = blah_func(optimising_func);
    Uk = U[0];
    
end