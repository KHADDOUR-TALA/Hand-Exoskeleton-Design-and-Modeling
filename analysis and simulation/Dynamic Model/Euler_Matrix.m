function [ Orient ] = Euler_Matrix( Psi,Theta,Phi )

Orient(1,1)=cos(Psi)*cos(Phi)-sin(Psi)*cos(Theta)*sin(Phi);
Orient(1,2)=-cos(Psi)*sin(Phi)-sin(Psi)*cos(Theta)*cos(Phi);
Orient(1,3)=sin(Psi)*sin(Theta);

Orient(2,1)=sin(Psi)*cos(Phi)+cos(Psi)*cos(Theta)*sin(Phi);
Orient(2,2)=-sin(Psi)*sin(Phi)+cos(Psi)*cos(Theta)*cos(Phi);
Orient(2,3)=-cos(Psi)*sin(Theta);

Orient(3,1)=sin(Theta)*sin(Phi);
Orient(3,2)=sin(Theta)*cos(Phi);
Orient(3,3)=cos(Theta);
end

