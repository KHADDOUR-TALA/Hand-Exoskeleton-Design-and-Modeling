
syms th1 theta_MCP_flex theta_PIP theta_DIP real;
syms dth1 dtheta_MCP_flex dtheta_PIP dtheta_DIP real;
syms ddtheta_MCP_abd ddtheta_MCP_flex ddtheta_PIP ddtheta_DIP real;
syms Lmp Lpp Ldp real;
syms g1x g1y g1z g2x g2y g2z g3x g3y g3z real;
syms m1 m2 m3 real;
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 real;
syms c1 c2 c3 c4 k1 k2 k3 k4 real;
syms g real;

% Generalized coordinates and their derivatives
theta = [th1; theta_MCP_flex; theta_PIP; theta_DIP];
dtheta = [dth1; dtheta_MCP_flex; dtheta_PIP; dtheta_DIP];
ddtheta = [ddtheta_MCP_abd; ddtheta_MCP_flex; ddtheta_PIP; ddtheta_DIP];

DH_table=[ 0 pi/2 0 theta_MCP_abd;
              Lpp 0 0 theta_MCP_flex;
              Lmp 0 0 theta_PIP;
              Ldp  0 0 theta_DIP;
];
k=1;
T01=(DHmatrix( DH_table(k,1),DH_table(k,2),DH_table(k,3),DH_table(k,4) ));k=k+1;
T12=(DHmatrix( DH_table(k,1),DH_table(k,2),DH_table(k,3),DH_table(k,4) ));k=k+1;
T23=(DHmatrix( DH_table(k,1),DH_table(k,2),DH_table(k,3),DH_table(k,4) ));k=k+1;
T34=(DHmatrix( DH_table(k,1),DH_table(k,2),DH_table(k,3),DH_table(k,4) ));k=k+1;

T02=simplify(T01*T12);
T03=simplify(T01*T12*T23);
T04 = simplify(T03 * T34);

% Assume centers of mass are at the midpoints of each segment
p1 = T01(1:3, 4) + T02(1:3, 1:3) * [g1x; g1y; g1z];
p2 = T02(1:3, 4) + T03(1:3, 1:3) * [g2x; g2y; g2z];
p3 = T03(1:3, 4) + T04(1:3, 1:3) * [g3x; g3y; g3z];

% p1 = T01(1:3, 4) + T02(1:3, 1:3) * [Lpp/2; 0; 0];
% p2 = T02(1:3, 4) + T03(1:3, 1:3) * [Lmp/2; 0; 0];
% p3 = T03(1:3, 4) + T04(1:3, 1:3) * [Ldp/2; 0; 0];
v1 = jacobian(p1, theta) * dtheta;
v2 = jacobian(p2, theta) * dtheta;
v3 = jacobian(p3, theta) * dtheta;

R01 = T01(1:3, 1:3);
R12 = T12(1:3, 1:3);
R23 = T23(1:3, 1:3);
R34 = T34(1:3, 1:3);

omega1 = [0; 0; dth1];
omega2 = omega1+R01 * [0; 0; dtheta_MCP_flex];  % Local z-axis of joint 2 in frame 1
omega3 = omega2+R01 * R12 * [0; 0; dtheta_PIP];  % Local z-axis of joint 3 in frame 2
omega4 = omega3+R01 * R12 * R23 * [0; 0; dtheta_DIP];  % Local z-axis of joint 4 in frame 3

I1 = diag([Ixx1, Iyy1, Izz1]);
I2 = diag([Ixx2, Iyy2, Izz2]);
I3 = diag([Ixx3, Iyy3, Izz3]);

T1 = (1/2) * m1 * dot(v1, v1) + (1/2) * omega1.' * I1 * omega1;
T2 = (1/2) * m2 * dot(v2, v2) + (1/2) * omega2.' * I2 * omega2;
T3 = (1/2) * m3 * dot(v3, v3) + (1/2) * omega3.' * I3 * omega3;

T = T1 + T2 + T3;

V1 = m1 * g * p1(3);
V2 = m2 * g * p2(3);
V3 = m3 * g * p3(3);

V = V1 + V2 + V3;

D = (1/2) * c1 * dtheta_MCP_abd^2 + (1/2) * c2 * dtheta_MCP_flex^2 + (1/2) * c3 * dtheta_PIP^2 + (1/2) * c4 * dtheta_DIP^2;
K = (1/2) * k1 * theta_MCP_abd^2 + (1/2) * k2 * theta_MCP_flex^2 + (1/2) * k3 * theta_PIP^2+(1/2)*k4*theta_DIP^2;

L = T - V  - K;

dL_dq = jacobian(L, theta).';
dL_ddq = jacobian(L, dtheta).';

d_dt_dL_ddq = jacobian(dL_ddq, [theta; dtheta]) * [dtheta; ddtheta];

EOM = simplify(d_dt_dL_ddq - dL_dq);

% [M, C, G] = equationsOfMotion(theta, dtheta, ddtheta, EOM);
% 
% disp('Mass Matrix (M):');
% disp(M);
% 
% disp('Coriolis and Centrifugal Matrix (C):');
% disp(C);
% 
% disp('Gravity Vector (G):');
% disp(G);
