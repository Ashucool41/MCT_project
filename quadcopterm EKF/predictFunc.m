function [X_pred,P_pred] = predictFunc(Xk,uk,Pk,Q)
global Ad_function Bd_function BiasD_function

phi=Xk(10);
theta=Xk(11);
u1=uk(1);
u2=uk(2);
u3=uk(3);
u4=uk(4);
wy=Xk(8);
wz=Xk(9);
Ad_numeric=Ad_function(phi,theta,u1,u2,u3,u4,wy,wz)
Bd_numeric=Bd_function(phi,theta,u1,u2,u3,u4,wy,wz)
Bias_numeric=BiasD_function(phi,theta,u1,u2,u3,u4,wy,wz)
X_pred=Ad_numeric*Xk+Bd_numeric*uk+Bias_numeric;
P_pred=Ad_numeric*Pk*Ad_numeric'+Q;

end



%%HOW TO TEST
%Run this in command window to validate this function
%  Xk=zeros(12,1);uk=[.5,.5,.5,.5]';Pk=eye(12)*1;Q=eye(12)*0.00001;
%  predictFunc(Xk,uk,Pk,Q)