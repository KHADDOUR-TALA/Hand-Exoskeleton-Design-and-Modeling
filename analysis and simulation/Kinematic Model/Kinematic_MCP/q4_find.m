function [q4] =q4_find(q2,q3)
%UNTITLED26 Summary of this function goes here
%   Detailed explanation goes here
[alpha,theta] = forward(q2,q3);
q4=theta-q3+pi/2;

end

