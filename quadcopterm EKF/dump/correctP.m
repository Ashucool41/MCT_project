%function to correct Pcovariance prediction with measurements using kalman filter
%correctP(k,C,P_pred)
function P_corrected=correctP(k,C,P_pred)
    P_corrected=(eye(size(k*C))-k*C)*P_pred;
end