function [C,Y,R_vector] = getGPS(data)

global home GPS_xyVar GPS_zVar

[x,y,z] = geodetic2ned(data(8),data(9),data(10),home(1),home(2),home(3),oblateSpheroid,'degrees');
vx = data(11)*cos(pi/90*data(12));
vy = data(11)*sin(pi/90*data(12));
vz = data(13);
Y = [x;y;-z;vx;vy;vz]; %z is taken negative upwards as standard aerospace convention
C = zeros(6,12);
C(1,1) = 1;
C(2,2) = 1;
C(3,3) = 1;
C(4,4) = 1;
C(5,5) = 1;
C(6,6) = 1;
R_vector = [GPS_xyVar; GPS_xyVar; GPS_zVar; GPS_xyVar; GPS_xyVar; GPS_zVar];
end