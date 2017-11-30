syms A B Xk Xk_prev Uk Uk_prev

syms x y z vx vy vz  wx wy wz phi theta psi 
X_syms = [x; y; z; vx; vy; vz;  wx; wy; wz; phi; theta; psi];
%% MODEL PARAMETERS
Cth =9.8;%Newton/unit throttle 
L =0.45/2; %(half length of quad diagonal)
torqByThrust= 0.0162;

mass = 1.5;
Ixx=0.0142;
Iyy=Ixx;
Izz=0.0284;
%% A_continuous and B_continous
A_model = sym('b',[12,12])*0;
A_model(1,4) = 1;
A_model(2,5) = 1;
A_model(3,6) = 1;
A_model(10,7) = 1;
A_model(10,8) = 0;%(theta)*(phi);
A_model(10,9) = (theta);
A_model(11,8) = 1;
A_model(11,9) = -(phi);
A_model(12,8) = (phi);
A_model(12,9) = 1;

%Rotation matrix(in case needed)
Cbi = [ cos(theta)*cos(psi),                        cos(theta)*sin(psi),                     -sin(theta);
       -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi),   cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta);
        sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),  -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta);];
Cbi_approx=[1, psi, -theta;-psi,1,phi;theta,-phi,1];
    B_vel_part = Cth/mass*transpose(Cbi_approx)* [0      ,0          ,0    ,     0;
                               0      ,0          ,0    ,     0;
                               1      ,1          ,1    ,     1;];
Km = L/sqrt(2)*Cth;

B_w_part = Km*[-1/Ixx,1/Ixx,1/Ixx,-1/Ixx;
             1/Iyy,-1/Iyy,1/Iyy,-1/Iyy;
             torqByThrust/Izz,torqByThrust/Izz,-torqByThrust/Izz,-torqByThrust/Izz;];
B_model = [zeros(3,4);B_vel_part;B_w_part;zeros(3,4)];

Xdot_model = A_model*X_model+B_model*U;

%% Linearisation and discretisation(Symbolic!!!)

%Ad = expm(A_model*Ts); %discretised. this should be changed slightly
%Ap = jacobian(A_model*X,X); % jacobian. This matrix is supposed to be used every where for discretised A matrix

A_linearised=jacobian(A_model*X_syms,X_syms);
Ad=expm(A_linearised*Ts); %linearised discrete symbolic model(takes 30secs). PLug in the states when needed
syms tau
temp1=expm(A_linearised*tau);%takes around 5 minutes using accurate rotation matrix
Bd = int(temp1,tau,0,Ts)*B_model; %discretised model. Takes for ever
Ad_function=matlabFunction(Ad);
Bd_function=matlabFunction(Bd);
Cbi_function=matlabFunction(Cbi);
%NOTE: This is first principles model
%%
Xk = A*Xk_prev + B*Uk_prev; % linearised control model (approximated)
control_function = matlabFunction(Xk);
model_function = matlabFunction(X_model);
