function [alpha,theta] = forward(q2,q3)
%UNTITLED20 Summary of this function goes here
%   Detailed explanation goes here
L1=12;L2=20;L3=45;L4=4;L5=11;L6=36;H=20;V=47;dirac1=0;dirac2=0;dirac3=0;

alpha=q2;
D=L1-V+dirac3+L2*cos(q2)+L3*cos(q3)*cos(q2);
theta=2*atan((D-sqrt(D.^2+((H-dirac2-L3*sin(q3)).^2-(L4+L5).^2)*cos(q2).^2))/((H-dirac2-L3*sin(q3)+L4+L5)));    
A=(H-dirac2-L3*sin(q3))*cos(q2);
C=L1-V-dirac3+(L2+L3*cos(q3))*cos(q2);
B=(L4+L5)*cos(q2);

%%theta=2*atan((A+sqrt(A.^2-B.^2+C.^2))/(B+C));
theta1=2*atan((A+(A^2-B^2+C^2)^(1/2))/(B+C));
end

