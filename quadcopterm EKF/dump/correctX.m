%function to correct state prediction with measurements using kalman filter
%correctX(k,X_pred,Y_measured,C)
function X_corrected=correctX(k,X_pred,Y_measured,C)
    Y_pred=C*X_pred;
    X_corrected=X_pred+k*(Y_measured-Y_pred);
end