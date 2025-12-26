clc
clear
%%direct KInematic with  misaligning displacemen
syms H V dirac1 dirac2 dirac3 alpha theta L5 L6 q5 q1 q2 q3 q4 L1 L2 L3 L4
theta=q3+q4-pi/2;
alpha=q2;
dirac1=q1+((L4+L5)*cos(q3+q4)+L2+L3*cos(q3)-(L6+q5)*sin(q3+q4))*sin(q2);
dirac2=H-(L6+q5)*cos(q3+q4)-(L4+L5)*sin(q3+q4)-L3*sin(q3);
dirac3=L1-V+(L2+(L4+L5)*cos(q3+q4)+L2+L3*cos(q3)-(L6+q5)*sin(q3+q4))*cos(q2);
%%%%%direct
alpha=q2;
D=L1-V+dirac3+L2*cos(q2)+L3*cos(q3)*cos(q2);
theta=2*atan((D-sqrt(D.^2+((H-dirac2-L3*sin(q3)).^2-(L4+L5).^2)*cos(q2).^2))\((H-dirac2-L3*sin(q3)+L4+L5)));
q1=(L1-V-dirac3)*tan(q2)-dirac1;
A=(H-dirac2-L3*sin(q3))*cos(q2);
B=L1-V-dirac3+(L2+L3*cos(q3))*cos(q2);
C=(L4+L5)*cos(q2);
theta1=2*atan((A+sqrt(A.^2-B.^2+C.^2))\(B+C));
%%%%inverse
alpha=pi/3;
L1=10.5;L2=20.5;L3=45;L4=8.6;L5=13.2;L6=36;H=22;V=39;dirac1=0;dirac2=0;dirac3=0;theta=pi/2;
q2=alpha
P=L3*cos( q2 )*sin( alpha );
M=-L3*cos(q2)*cos(theta);
N=(L4+L5)*cos(q2)-(L1-V-dirac3+L2*cos(q2))*sin(theta)-(H-dirac2)*cos(q2)*cos(theta);
q3=2*atan((M+sqrt(M.^2-N.^2+P.^2))\(P+N))*180/pi