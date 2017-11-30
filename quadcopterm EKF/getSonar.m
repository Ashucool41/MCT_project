function [C,Y,r] = getSonar(data)
global    sonarVar sonarBias  
Y = -(data(3)-sonarBias);%Z is taken in usual aerospace convention opposite to common sense
%Z is taken downwards. So if height is 10m. Z=-10m hahaaha
C = zeros(1,12);
C(1,3) = 1;
r=sonarVar;
end
