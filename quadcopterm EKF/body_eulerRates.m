%scratch code to take matrix inverse

syms y p r

C=[-sin(p),0, 1; cos(p)*sin(r) cos(r) 0; cos(p)*cos(r) -sin(r) 0]
%C*[y_dot;p_dot;r_dot]=[wx;wy;wz]

C_inv=inv(C)
