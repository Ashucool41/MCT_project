function [C,Y] = getGPS(data)

[x,y,z] = geodetic2ned(data(8),data(9),data(10),home(0),home(1),home(2),spheroid);
vx = data(11)*cos(pi/90*data(12));
vy = data(11)*sin(pi/90*data(12));
vz = data(13);
Y = [x;y;z;vx;vy;vz];
C = zeros(6,12);
C(1,1) = 1;
C(2,2) = 1;
C(3,3) = 1;
C(4,4) = 1;
C(5,5) = 1;
C(6,6) = 1;
end
