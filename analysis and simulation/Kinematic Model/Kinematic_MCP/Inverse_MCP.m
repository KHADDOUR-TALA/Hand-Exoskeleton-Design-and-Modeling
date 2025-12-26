clc
clear
%%direct KInematic without  misaligning displacemen
syms H V theta1  theta2 L5 L6 q5 q1 q2 q3 q4 L1 L2 L3 L4
q2=theta1
L=L3*c( q2 )*s( theta1 )
M=-L3*cos(q2)*cos(theta2);
N=L4*cos(q2)-(L1-V+L2*cos(q2))*sin(theta2)-H*cos(q2)*cos(theta2)
q3=2*atan((M+sqrt(M.^2-N.^2+L.^2))\(L+N));