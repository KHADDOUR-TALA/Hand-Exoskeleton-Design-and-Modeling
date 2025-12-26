syms q1 q2 q3 q4 dq1 dq2 dq3 dq4 real
syms L1 L2 L3 L4 real

% Define the DH Parameters
DH = [0, 0, 0, q1;
      L1, pi/2, 0, q2;
      L2, -pi/2, 0, q3;
      L3, 0, 0, q4;
      L4, pi/2, 0, pi/2];


% Transformation Matrices
T01 = DH2T(DH(1,:));
T12 = DH2T(DH(2,:));
T23 = DH2T(DH(3,:));
T34 = DH2T(DH(4,:));
T45 = DH2T(DH(5,:));

% Full Transformation Matrices
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;

% Extract position of the end-effector
p0 = T05(1:3,4);

% Compute the Jacobian matrix (linear part for simplicity)
Jv1 = jacobian(p0, q1);
Jv2 = jacobian(p0, q2);
Jv3 = jacobian(p0, q3);
Jv4 = jacobian(p0, q4);

% Combine Jacobians
Jv = [Jv1, Jv2, Jv3, Jv4];

% Joint Velocities
dq = [dq1; dq2; dq3; dq4];

% Velocity Model
velocity = Jv * dq;

%disp('Jacobian Matrix:');
%disp(Jv);
%disp('Velocity Model:');
%disp(velocity);
% Function to calculate transformation matrix from DH parameters
function T = DH2T(dh)
    theta = dh(4);
    d = dh(1);
    a = dh(3);
    alpha = dh(2);
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end
