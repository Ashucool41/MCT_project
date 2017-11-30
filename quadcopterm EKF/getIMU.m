function [C,Y,Rvector] = getIMU(data,Cib,v_old)
global  IMU_gyroVar IMU_accVar Ts theta_accVar roll_accVar IMU_gyroBias IMU_accBias
    wx = pi/180*(data(3)-IMU_gyroBias(1));
    wy = pi/180*(data(4)-IMU_gyroBias(2));
    wz = pi/180*(data(5)-IMU_gyroBias(3));
    Acc = [data(6);data(7);data(8)];
    %v = v_old+Ts*(Cib*Acc+[0;0;9.81]);
    % In Raw accelerometer readings. g is not compensated!!!. This line is
    % giving a lot of problems. Firstly Cib estimate depends on the kalman
    % filter which inturn depends on this thing. Might become unstable.
    %Since we know that angles are going to be small and this integration
    %is not soo vital. I am going to replace Cib by Identity matrix. That
    %should be an approximate. 
    %9.5172 is used instead of 1g because that is the norm of IMU_accBias
    %at static test. so accelerometer seems to be a bit out of
    %calibration(about 3%)
    v=v_old+Ts*(Acc+[0;0;9.5172]);%however there is an issue somewhere else also
    %Using Accelerometer data to estimate the yaw, pitch and roll angles
    mod_acc=norm(Acc);
    theta_acc=asin(Acc(1)/mod_acc);
    roll_acc=atan(Acc(2)/Acc(3));    
    
    Y = [v;wx;wy;wz;roll_acc;theta_acc];
    C = zeros(8,12);
    C(1,4) = 1;
    C(2,5) = 1;
    C(3,6) = 1;
    C(4,7) = 1;
    C(5,8) = 1;
    C(6,9) = 1;
    C(7,10) =1;
    C(8,11) =1;
    velVar=Ts^2*IMU_accVar;
    gyroRadvar=(pi/180)^2*IMU_gyroVar;
    Rvector=[velVar;velVar;velVar;gyroRadvar;gyroRadvar;gyroRadvar;roll_accVar;theta_accVar];
end
