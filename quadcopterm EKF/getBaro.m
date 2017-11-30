function [C,Y,r] = getBaro(data)
global sortedData IMU_gyroVar IMU_accVar sonarVar sonarBias baroVar baroBias

Y = -(data(3)-baroBias);%Z is taken in usual aerospace convention opposite to common sense
%Z is taken downwards. So if height is 10m. Z=-10m hahaaha
C = zeros(1,12);
C(1,3) = 1;
r= baroVar;

end
