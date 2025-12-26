function J = jacobian2T()
 syms alpha1 theta
%L1=12;
L2=20;
%L3=45;
L4=4;L5=11;L6=36;H=20;V=47;dirac1=0;dirac2=0;dirac3=0; 
q2=alpha1;
P=L3*cos( q2 )*sin( theta );
M=-L3*cos(q2)*cos(theta);
N=(L4+L5)*cos(q2)-(L1-V-dirac3+L2*cos(q2))*sin(theta)-(H-dirac2)*cos(q2)*cos(theta);
q3=2*atan((M+sqrt(M.^2-N.^2+P.^2))/(P+N));
 
    
    % State vector
    X = [alpha1; theta];
    % Control vector
    phi = [q2;q3];
    
    % Compute the Jacobian
    J =inv( jacobian(phi,X));
    
    % Substitute numerical values
    %J = double(subs(J, {q2, q3}, {q_2, q_3}));
end
