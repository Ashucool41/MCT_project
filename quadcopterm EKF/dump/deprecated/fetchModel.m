%continous state space model parameters INCORRECT A MATRIX!!
syms x y z vx vy vz  wx wy wz phi theta psi 
X_syms = [x; y; z; vx; vy; vz;  wx; wy; wz; phi; theta; psi];
%% MODEL PARAMETERS
Cth =1; 
L =1;
Kptau=2;
Kpt=3;
Ts = .01;
mass = 1;
%% A_continuous and B_continous
A_model = sym('b',[12,12])*0;
A_model(1,4) = 1;
A_model(2,5) = 1;
A_model(3,6) = 1;
A_model(10,7) = 1;
A_model(10,8) = tan(theta)*sin(phi);
A_model(10,9) = tan(theta)*cos(phi);
% A_model(11,8) = cos(phi);INCORRECT
% A_model(11,9) = -sin(phi);
% A_model(11,9) = tan(phi);
% A_model(11,9) = cos(phi)*sec(theta);
%Rotation matrix(in case needed)
Cbi = [ cos(theta)*cos(psi),                        cos(theta)*sin(psi),                     -sin(theta);
       -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi),   cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta);
        sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),  -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta);];
Cbi_approx=[1, psi, -theta;-psi,1,phi;theta,-phi,1];
    B_vel_part = Cth/mass*transpose(Cbi_approx)* [0      ,0          ,0    ,     0;
                               0      ,0          ,0    ,     0;
                               1      ,1          ,1    ,     1;];
Km = L/sqrt(2)*Cth;

B_w_part = Km*[-1,1,1,-1;
             1,-1,1,-1;
             Kptau/Kpt,Kptau/Kpt,-Kptau/Kpt,-Kptau/Kpt;];
B_model = [zeros(3,4);B_vel_part;B_w_part;zeros(3,4)];

%% Linearisation and discretisation(Symbolic!!!)

%Ad = expm(A_model*Ts); %discretised. this should be changed slightly
%Ap = jacobian(A_model*X,X); % jacobian. This matrix is supposed to be used every where for discretised A matrix

A_linearised=jacobian(A_model*X_syms,X_syms);
Ad=expm(A_linearised*Ts); %linearised discrete symbolic model. PLug in the states when needed
syms tau
temp1=expm(A_linearised*tau);%takes around 5 minutes
Bd = int(temp1,tau,0,Ts)*B_model; %discretised model. Takes for ever

%NOTE: This is first principles model