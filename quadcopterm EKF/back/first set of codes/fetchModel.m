%continous state space model parameters
syms x y z vx vy vz  wx wy wz phi theta psi tau
X = [x; y; z; vx; vy; vz;  wx; wy; wz; phi; theta; psi];
Cth =1; 
L =1;
Kptau=2;
Kpt=3;
Ts = .01;
mass = 1;
A_model = sym('b',[12,12])*0;
A_model(1,4) = 1;
A_model(2,5) = 1;
A_model(3,6) = 1;
A_model(10,7) = 1;
A_model(10,8) = tan(theta)*sin(phi);
A_model(10,9) = tan(theta)*cos(phi);
A_model(11,8) = cos(phi);
A_model(11,9) = -sin(phi);
A_model(11,9) = tan(phi);
A_model(11,9) = cos(phi)*sec(theta);

Cbi = [ cos(theta)*cos(psi),                        cos(theta)*sin(psi),                     -sin(theta);
       -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi),   cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta);
        sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),  -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta);];
B_vel_part = Cth/mass*transpose(Cbi)* [0      ,0          ,0    ,     0;
                               0      ,0          ,0    ,     0;
                               1      ,1          ,1    ,     1;];
Km = L/sqrt(2)*Cth;

B_w_part = Km*[-1,1,1,-1;
             1,-1,1,-1;
             Kptau/Kpt,Kptau/Kpt,-Kptau/Kpt,-Kptau/Kpt;];
B = [zeros(3,4);B_vel_part;B_w_part;zeros(3,4)];

Ad = expm(A_model*Ts);
Bd = int(expm(A_model*tau),tau,0,Ts)*B;
Ap = jacobian(A_model*X,X);
