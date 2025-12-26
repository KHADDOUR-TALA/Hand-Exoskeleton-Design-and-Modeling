function [ theta1,x,theta2,q1] = forward_kinematics_MCP(q2,q3)
L1=12;
L2=20;
L3=45;
L4=15;
V=47;
H=20;
theta1=q2;
M=(H-L3*sin(q3))*cos(q2);
N=L4*cos(q2);
P=L1-V+(L2+L3*cos(q3))*cos(q2);
theta2=2*atan((M+(M^2-N^2+P^2)^(1/2))/(N+P));
x=(H-L4*sin(theta2+pi/2)-L3*sin(q3))/cos(theta2+pi/2);
q1=(L1-V)*tan(q2);
end

