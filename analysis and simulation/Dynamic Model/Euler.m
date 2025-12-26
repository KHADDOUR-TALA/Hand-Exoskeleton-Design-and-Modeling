function [ psi,theta,phi ] = Euler( Orient_Matrix )
theta=acos(Orient_Matrix(3,3));
psi=atan2(Orient_Matrix(1,3),-Orient_Matrix(2,3));
phi=atan2(Orient_Matrix(3,1),Orient_Matrix(3,2));
end