
%%Prediction expression can use accurate model itself
%Ts comes from the main script. Without a value for timestep this code wont
%run
global X_syms Ts Cbi Cbi_approx Ad Bd
global x y z vx vy vz  wx wy wz phi theta psi 
global Ad_function Bd_function Cbi_function BiasD_function

syms x y z vx vy vz  wx wy wz phi theta psi 
syms u1 u2 u3 u4
X_syms = [x; y; z; vx; vy; vz;  wx; wy; wz; phi; theta; psi];
U_syms = [u1;u2;u3;u4];
%% MODEL PARAMETERS
Cth =0.75*9.8;%Newton/unit throttleInput 750gm thrust C2830 motor+10X4.5 
%inch propeller(http://www.rcbazaar.com/product.aspx?productid=1905)
%Note that all inputs(to esc--pulse width(uS)) mentioned here are scaled 
%between 0.0-1.0 for conveniance sake 
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
                               -1      ,-1          ,-1    ,     -1;];
Km = L/sqrt(2)*Cth;

B_w_part = Km*[-1/Ixx,1/Ixx,1/Ixx,-1/Ixx;
             1/Iyy,-1/Iyy,1/Iyy,-1/Iyy;
             torqByThrust/Izz,torqByThrust/Izz,-torqByThrust/Izz,-torqByThrust/Izz;];
B_model = [zeros(3,4);B_vel_part;B_w_part;zeros(3,4)];
Bias_model = [0 0 0 0 0 9.81 0 0 0 0 0 0]';

%% Linearisation and discretisation(Symbolic!!!) OLD ONE . Has A Linearisation problem
% 
% 
% A_linearised=jacobian(A_model*X_syms,X_syms);
% Ad=expm(A_linearised*Ts); %linearised discrete symbolic model(takes 30secs). PLug in the states when needed
% syms tau
% temp1=expm(A_linearised*tau);%takes around 5 minutes using accurate rotation matrix
% Bd = int(temp1,tau,0,Ts)*B_model; %discretised model. Takes for ever
% Ad_function=matlabFunction(Ad);
% Bd_function=matlabFunction(Bd);
% Cbi_function=matlabFunction(Cbi);
% %NOTE: This is first principles model

%% PROPER LINEARISATION about any operating point
A_linearised=jacobian(A_model*X_syms+B_model*U_syms,X_syms);
B_linearised=B_model;%But X0 and U0 points about linearisation should be substituted:)
Bias_linearised=Bias_model+A_model*X_syms-A_linearised*X_syms;

%% Discretisation
%Note that we are discretising about some operating point. So the following
%symbolic expressions are functions for the current operating point.
syms tau %for integration a time variable is required
Ad=expm(A_linearised*Ts);
temp1=expm(A_linearised*tau);
Bd=int(temp1,tau,0,Ts)*B_linearised;
Bias_discretised=int(temp1,tau,0,Ts)*Bias_linearised;

%% Convert Symbolic model to Functions
 Ad_function=matlabFunction(Ad);
 Bd_function=matlabFunction(Bd);
 BiasD_function=matlabFunction(Bias_discretised);
 Cbi_function=matlabFunction(Cbi);


