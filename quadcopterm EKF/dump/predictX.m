%function to predict new state using kalman filter 
%predictX(A,x_old)
function x_pred=predictX(A,x_old)
    x_pred=A*x_old;
    
end