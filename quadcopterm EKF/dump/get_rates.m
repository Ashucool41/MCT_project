%continous state space model parameters
%var x,y,z,vx,vy,vz, wx,wy,wz, r,p,y,Cth,L,Kptau,Kpt;
A = sparse([1,2,3,10,10,10,11,11,12,12],[4,5,6,7,8,9,8,9,8,9],[1,1,1,1,tan(p)*sin(r),tan(p)*cos(r),cos(r),-sin(r),tan(r),cos(r)*sec(p)],12,12);
Cbi = [ cos(p)*cos(y),                        cos(p)*sin(y),                     -sin(p);
       -cos(r)*sin(y)+sin(r)*sin(p)*cos(y),   cos(r)*cos(y)+sin(r)*sin(p)*sin(y), sin(r)*cos(p);
        sin(r)*sin(y)+cos(r)*sin(p)*cos(y),  -sin(r)*cos(y)+cos(r)*sin(p)*sin(y), cos(r)*cos(p);];
B_vel_part = Cth/mass*inv(Cbi)* [0      ,0          ,0    ,     0;
                               0      ,0          ,0    ,     0;
                               1      ,1          ,1    ,     1;];
Km = L/sqrt(2)*Cth;

B_w_part = Km*[-1,1,1,-1;
             1,-1,1,-1;
             Kptau/Kpt,Kptau/Kpt,-Kptau/Kpt,-Kptau/Kpt;];
B = [zeros(3,4);B_vel_part;B_w_part;zeros(3,4];
