function [X_pred,P_pred] = predictFunc(Xk,uk,Pk,Q)
global Ad_function Bd_function

% Ad_numeric=double(subs(Ad,X_syms,Xk));
% Bd_numeric=double(subs(Bd,X_syms,Xk));
Ad_numeric=Ad_function(Xk(10),Xk(11),Xk(8),Xk(9));
Bd_numeric=Bd_function(Xk(10),Xk(11),Xk(8),Xk(9));
if Xk(3)<0.010
    bias=[0;0;0;0;0;0;0;0;0;0;0;0];
else
    bias=[0;0;0;0;0;9.81;0;0;0;0;0;0];
end
X_pred=Ad_numeric*Xk+Bd_numeric*uk*0+bias*0;
P_pred=Ad_numeric*Pk*Ad_numeric'+Q;

end