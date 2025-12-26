function [ alpha,beta,gamma ] = Roll_Pitch_Yaw( Orient_Matrix )
% beta=atan2(-Orient_Matrix(3,1),sqrt(Orient_Matrix(1,1)*Orient_Matrix(1,1)+Orient_Matrix(2,1)*Orient_Matrix(2,1)));
beta=asin(-Orient_Matrix(3,1));
alpha=atan2(Orient_Matrix(2,1),Orient_Matrix(1,1));
gamma=atan2(Orient_Matrix(3,2),Orient_Matrix(3,3));
end