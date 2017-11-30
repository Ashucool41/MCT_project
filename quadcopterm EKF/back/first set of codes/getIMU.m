function [C,Y] = getIMU(data,Cib,v_old)
    wx = pi/90*data(3);
    wy = pi/90*data(4);
    wz = pi/90*data(5);
    Acc = [data(6);data(7);data(8)];
    v = v_old+Ts*Cib*Acc;
    Y = [v;wx;wy;wz];
    C = zeros(6,12);
    C(1,4) = 1;
    C(2,5) = 1;
    C(3,6) = 1;
    C(4,7) = 1;
    C(5,8) = 1;
    C(6,9) = 1;
end
