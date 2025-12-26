function [q2,q3] =inverse_T(L,alpha,theta)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here
%L1=12;
L1=L(1);
L2=L(2);
L3=L(3);
%L3=45;
L4=L(4);L5=1;L6=36;H=20;V=47;dirac1=0;dirac2=0;dirac3=0; 
q2=alpha;
P=L3*cos( q2 )*sin( theta );
M=-L3*cos(q2)*cos(theta);
N=(L4+L5)*cos(q2)-(L1-V-dirac3+L2*cos(q2))*sin(theta)-(H-dirac2)*cos(q2)*cos(theta);
q3=2*atan((M+sqrt(M.^2-N.^2+P.^2))/(P+N));

end

