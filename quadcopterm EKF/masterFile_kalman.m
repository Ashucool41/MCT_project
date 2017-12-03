%Master file for kalman filter. Execute this file to see the ekf at work
%This code is an EKF for Quadcopter that works on raw sensor data collected
    %Using PIXHAWK flight controller on an AeroClub Quadcopter. Since
    %pixhawk also has a kalman filter inside. we can filter data using raw
    %sensor data and compare with its onboard EKF's data. Please read
    %further to know about an important shortcoming in this way of rawdata
    %acquisition :(

%fetchMeasurements.m gets data at every loop from sorted_time....csv file
    %using its child functions
%Sensors used currently- sonar, barometer, IMU(gyro+acc-raw)
%All files with get_sensor name_.m are used to get sensor variance and bias
    %corrected readings in correct units.
%fetchModel_approx.m  gets the function handle used to compute jacobian by linearising the symbolic flight dynamic model(First principles) 

%NOTE: All the extra _*100 or something given to the variances are the extra
%given to reduce the importance of that particular measurement. Because
%sensor could behave different in flight compared to the test in static non
%flight condition. 

%NOTE: Pixhawk has access to its high speed sensor data. But it doesnot
%store all the raw data into its sd card. So we dont have the raw data at
%the same rate as pixhawk. we have data at 10times slower rates. So our EKF
%maynot be able to meet pixhawk level. However if it is implemented
%onboard, then we will get sensor readings at much higher rate:)

clear all
close all
global IMU_gyroVar IMU_accVar sonarVar sonarBias baroVar baroBias IMU_gyroBias IMU_accBias
global Ts % X_syms  Cbi Cbi_approx Ad Bd
%global x y z vx vy vz  wx wy wz phi theta psi %%These the 12 states taken
global sortedData home GPS_xyVar GPS_zVar
global theta_accVar roll_accVar

Ts = 0.010;  %sampling time in seconds

fetchModel_approx %script to get expression/symbolic matrix for B,jacobian() Ad=jacobian found
sortedData=csvread('Sorted_time_wise.csv'); % read the data log file
%% SETTINGS
home = [12.9917,80.2311,3.2300]; %first gps lat lon and alt for GPS data
IMU_gyroVar=2.3675e-04;%*N means multiplied by N times more than the static test vareince
IMU_accVar=0.03;
IMU_gyroBias=[-3.3102e-04;-9.9700e-04;-8.3456e-04];
IMU_accBias=[-0.1447;-0.3131; -9.5109];%Ground maynot be level initially. So better not to use the accelerometer's bias measured before the start of actual flight
sonarVar=3.8304e-04;
sonarBias=0 ; %.2782 is not bias. just that sensor is not in operational zone
baroVar=0.0915;
baroBias=0.5413;
theta_accVar=IMU_accVar/(9.81)^2; %determined using the formula used for angle estimation from accelerometer. refer getIMU.m
roll_accVar=IMU_accVar/(9.81)^2;
GPS_xyVar=0.342;
GPS_zVar=0.6312;
Q=eye(12)*0.00001;%This is equation error's variance
Q(3,3)=0.0001; Q(6,6)=0.00001;
Q(7,7)=0.0001;   Q(8,8)=Q(7,7);Q(9,9)=Q(7,7);
Q(10,10)=0.000001; Q(11,11)=Q(10,10);Q(12,12)=Q(10,10);

%% INITIATION
Xk=zeros(12,1)+0.001;%initial condition for state
uk=[0;0;0;0];
Pk=eye(12)*1;  %initial guess for process covarr
 %number taken for process variance. For now this number is not exactly correct as the model used is first principles model only. Later this matrix will be found through System Identification
T0=40816043;  %initial time in micro seconds. check the data file/log
N=15000;  %17378-max no of samples available; %no of samples we want to filter N*Ts= total time filtered
X_filteredData=zeros(12,N);
X_filteredData(:,1)=Xk; %X_filteredData is used to store the results
KalmanGainData=zeros(N,1);
tic

%% EKF LOOP
disp('ekf loop started')
index_loop = 0;
for index_loop = 2:N
    disp(index_loop)
    [R,C,Y_meas,uk_new]=fetchMeasurements(T0, index_loop, Xk); %past measurement is needed for integrating accelerometer measurement in the current formulation of ekf
         if(isempty(uk_new)==false)%check whether the input signal changed and update uk
             uk=uk_new;
         end        
    [X_pred,P_pred]=predictFunc(Xk,uk,Pk,Q);
    if(isempty(Y_meas)==false)%should update in this case when measurement is available. how can update happen without measurements.
        KalmanGain=getK(P_pred,R,C);
        [X_corr,P_corr]=correctFunc(X_pred,P_pred,KalmanGain,C,Y_meas);
        Xk=X_corr;
        Pk=P_corr;
        %disp('sensor data coming')
        %disp(KalmanGain)
    else
        Xk=X_pred;
        Pk=P_pred;
    end
     
    X_filteredData(:,index_loop)=Xk;%Storing the filtered data
end
disp('CPU time elapsed')
disp(toc)
disp('duration calculated')
disp(N*Ts)
plot((1:N)*Ts,X_filteredData(10,:)*180/pi);
xlabel('time in s')
title('roll angle in deg');
figure
plot((1:N)*Ts,X_filteredData(11,:)*180/pi) ;
xlabel('time in s')
title('pitch angle in deg');
figure
plot((1:N)*Ts,X_filteredData(12,:)*180/pi) ;
xlabel('time in s')
title('yaw angle in deg');
figure
plot((1:N)*Ts,-X_filteredData(3,:))
xlabel('time in s')%Altitude in normal human +ve coordinates
title('Height in meters')
figure
plot((1:N)*Ts,X_filteredData(2,:))
xlabel('time in s')%Altitude in normal human +ve coordinates
title('y in meters')
figure
plot((1:N)*Ts,X_filteredData(1,:))
xlabel('time in s')%Altitude in normal human +ve coordinates
title('x in meters')

display('roll mean')
display(mean(X_filteredData(10,:)*180/pi));%roll angles' mean