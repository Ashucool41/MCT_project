%script to calculate variance in measurements
clear all
close all

load static_test_with_sonar.mat% Not completely static!!
accxVar=var(IMU(1:450,6))% in SI square
accyVar=var(IMU(1:450,7))
acczVar=var(IMU(1:450,8))

accXmean=mean(IMU(1:450,6))
accYmean=mean(IMU(1:450,7))
accZmean=mean(IMU(1:450,8))

gyroXVar=var(IMU(1:450,3))% in deg/s square
gyroYVar=var(IMU(1:450,4))
gyroZVar=var(IMU(1:450,5))
gyroXmean=mean(IMU(1:450,3))
gyroYmean=mean(IMU(1:450,4))
gyroZmean=mean(IMU(1:450,5))

baroVar=var(BARO(1:200,3)) %meter^2
baroBias=mean(BARO(1:200,3)) %meter

sonarVar=var(RFND(1:400,3))
sonarBias=mean(RFND(1:400,3))

% for i=1:200
%     [gpsx(i),gpsy(i),gpsz(i)] = geodetic2ned(GPS(i,8),GPS(i,9),GPS(i,10),12.9916377000000,80.2311991000000,18.1800000000000,spheroid)
% end
% 
% gpsxVar = var(gpsx(1:200))
% gpsyVar = var(gpsy(1:200))
% gpszVar = var(gpsz(1:200))
save variance_and_bias_statictest
