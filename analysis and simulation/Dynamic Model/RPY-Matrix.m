function [ Orient ] = Roll_Pitch_Yaw_Matrix( Alpha,Beta,Gamma )

Orient(1,1)=cos(Alpha)*cos(Beta);
Orient(1,2)=-sin(Alpha)*cos(Gamma)+cos(Alpha)*sin(Beta)*sin(Gamma);
Orient(1,3)=sin(Alpha)*sin(Gamma)+cos(Alpha)*sin(Beta)*cos(Gamma);

Orient(2,1)=sin(Alpha)*cos(Beta);
Orient(2,2)=cos(Alpha)*cos(Gamma)+sin(Alpha)*sin(Beta)*sin(Gamma);
Orient(2,3)=-cos(Alpha)*sin(Gamma)+sin(Alpha)*sin(Beta)*cos(Gamma);

Orient(3,1)=-sin(Beta);
Orient(3,2)=cos(Beta)*sin(Gamma);
Orient(3,3)=cos(Beta)*cos(Gamma);

end

