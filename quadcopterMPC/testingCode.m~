%Just for testing things. Refer master file for mpc implementation
clear all
%Controller parameters
P=50;
M=20;
Qx=diag([1 1 1 1 1 1 0 0 0 0 0 0]);
lambda=1;

%get the model
fetchModel;

%setpoints and current state
Xk  = [0 0 0 0 0 0 0 0 0 0 0 0]';
Xref= [0 0 10 0 0 0 0 0 0 0 0 0]';

%get MPC formulation in QP form
[H,f_func]=fetchQPformulation(Ad,Bd,Bias_discrete,Qx,lambda,P,M);
f=f_func(Xk,Xref);

LB=zeros(M*4,1);UB=1.0+zeros(M*4,1);
U = quadprog(H,f,[],[],[],[],LB,UB)