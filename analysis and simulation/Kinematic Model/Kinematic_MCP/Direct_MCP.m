clc
clear
%%direct KInematic without  misaligning displacemen
syms H V theta1  theta2 L5 L6 q5 q1 q2 q3 q4 L1 L2 L3 L4
theta1=q2
A=(H-L3*sin(q_3))*cos(q_2);
B=L1-V+(L2+L3*cos(q3))*cos(q2);
C=L4*cos(q2);
theta2=2*atan((A+sqrt(A.^2-B.^2+C.^2))\(B+C));