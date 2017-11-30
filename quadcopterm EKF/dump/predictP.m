%function to predict new covariance matrix using kalman filter 
%predictP(A,P_old,Q)
function P_pred=predictP(A,P_old,Q)
    P_pred=A*P_old*A'+Q;
    
end