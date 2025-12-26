clc
clear
syms q1 q2 q3 q4 real;
syms dq1 dq2 dq3 dq4  real;
syms ddq1 ddq2 ddq3 ddq4 real;

L1=12;
L2=20;
L3=45;
L4=15;
g=9.8;
m1=100;
m2=70;
m3=170;
m4=100;

I1 = diag([Ixx1, Iyy1, Izz1]);
I2 = diag([Ixx2, Iyy2, Izz2]);
I3 = diag([Ixx3, Iyy3, Izz3]);

q = [q1 q2 q3 q4]';
dq = [dq1 dq2 dq3 dq4]';
ddq = [ddq1 ddq2 ddq3 ddq4]';

T01 = DHmatrix(0, 0, q1, 0 );
T12 = DHmatrix(L1, pi/2, 0, q2);
T23 = DHmatrix(L2, -pi/2, 0, q3);
T34 = DHmatrix(L3, 0, 0, q4);
T45 = DHmatrix(L4, pi/2, 0, 0);

T02 = simplify(T01 * T12);
T03 = simplify(T02 * T23);
T04 = simplify(T03 * T34);
T05 = simplify(T04 * T45);

p1 = T01(1:3, 4) + T02(1:3, 1:3) * [g1x; g1y; g1z];
p2 = T02(1:3, 4) + T03(1:3, 1:3) * [g2x; g2y; g2z];
p3 = T03(1:3, 4) + T04(1:3, 1:3) * [g3x; g3y; g3z];

% p1 = T01(1:3, 4) + T02(1:3, 1:3) * [Lpp/2; 0; 0];
% p2 = T02(1:3, 4) + T03(1:3, 1:3) * [Lmp/2; 0; 0];
% p3 = T03(1:3, 4) + T04(1:3, 1:3) * [Ldp/2; 0; 0];
v1 = jacobian(p1, theta) * dtheta;
v2 = jacobian(p2, theta) * dtheta;
v3 = jacobian(p3, theta) * dtheta;

omega1 = [0; 0; dth1];
omega2 = omega1+R01 * [0; 0; dth2];  % Local z-axis of joint 2 in frame 1
omega3 = omega2+R01 * R12 * [0; 0; dth3];  % Local z-axis of joint 3 in frame 2
omega4 = omega3+R01 * R12 * R23 * [0; 0; dth4];  % Local z-axis of joint 4 in frame 3


T1 = (1/2) * m1 * dot(v1, v1);
T2 = (1/2) * m2 * dot(v2, v2) + (1/2) * I2 * dtheta_MCP_flex^2;
T3 = (1/2) * m3 * dot(v3, v3) + (1/2) * I3 * dtheta_DIP^2;
T = T1 + T2 + T3;

% Potential energy (V) of each segment
V1 = m1 * g * r1(2);
V2 = m2 * g * r2(2);
V3 = m3 * g * r3(2);

V = V1 + V2 + V3;

L = T - V;

EOM = sym(zeros(4,1));
for i = 1:4
    dLdtheta_dot = diff(L, dtheta(i));
    ddt_dLdtheta_dot = diff(dLdtheta_dot, theta(i)) * dtheta(i) + diff(dLdtheta_dot, dtheta(i)) * (i);
    dLdtheta = diff(L, theta(i));
    EOM(i) = simplify(ddt_dLdtheta_dot - dLdtheta);
end

disp('The equations of motion are:');
disp(EOM);

