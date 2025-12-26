% Define symbolic variables
syms q1 q2 q3 q4 real
syms L1 L2 L3 L4 real
syms m1 m2 m3 m4 real  % Masses of the links
syms I1 I2 I3 I4 real  % Inertia of the links (assumed scalar for simplicity)

% Define DH parameters: [a, alpha, d, theta]
DH_table = [
    0, 0, (L1 - V ) * tan(q2), 0;
    L1, pi/2, 0, q2;
    L2, -pi/2, 0, q3;
    L3, 0, 0, q4;
    L4, pi/2, 0, pi/2
];

% Number of joints
n = size(DH_table, 1);

% Initialize transformation matrix
T = eye(4);
T_matrices = cell(1, n);

% Compute transformation matrices using DH parameters
for i = 1:n
    a = DH_table(i, 1);
    alpha = DH_table(i, 2);
    d = DH_table(i, 3);
    theta = DH_table(i, 4);
    
    T_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
           sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
           0, sin(alpha), cos(alpha), d;
           0, 0, 0, 1];
    
    T = T * T_i;
    T_matrices{i} = T;
end

% End-effector position
end_effector_pos = T(1:3, 4);

% Compute linear velocity Jacobian (Jv) with respect to active joints (q2 and q3)
Jv_full = jacobian(end_effector_pos, [q1, q2, q3, q4]);
Jv = Jv_full(:, 2:3);

% Compute angular velocity Jacobian (Jw) with respect to active joints (q2 and q3)
Jw_full = sym(zeros(3, n));
z = [0; 0; 1]; % initial z-axis

for i = 1:n
    if i == 1
        R_prev = eye(3);
    else
        R_prev = T_matrices{i-1}(1:3, 1:3);
    end
    Jw_full(:, i) = R_prev * z;
end

Jw = Jw_full(:, 2:3);

% Display Jacobians
disp('Linear Velocity Jacobian Jv:');
disp(Jv);
disp('Angular Velocity Jacobian Jw:');
disp(Jw);

% Define the inertia matrix for each link (assuming diagonal for simplicity)
I1_mat = diag([I1, I1, I1]);
I2_mat = diag([I2, I2, I2]);
I3_mat = diag([I3, I3, I3]);
I4_mat = diag([I4, I4, I4]);

% Define the mass matrix for each link
M1 = m1 * eye(3);
M2 = m2 * eye(3);
M3 = m3 * eye(3);
M4 = m4 * eye(3);

% Define the Jacobian for each link center of mass (assuming center at mid-point for simplicity)
Jv1_com = Jv / 2;
Jv2_com = Jv / 2;
Jv3_com = Jv / 2;
Jv4_com = Jv / 2;

% Inertia matrix computation
M = Jv1_com.' * M1 * Jv1_com + Jw.' * I1_mat * Jw + ...
    Jv2_com.' * M2 * Jv2_com + Jw.' * I2_mat * Jw + ...
    Jv3_com.' * M3 * Jv3_com + Jw.' * I3_mat * Jw + ...
    Jv4_com.' * M4 * Jv4_com + Jw.' * I4_mat * Jw;

% Reduce the inertia matrix to only include active joints (q2 and q3)
M_reduced = M(2:3, 2:3);

% Display Reduced Inertia Matrix
disp('Reduced Inertia Matrix M (for q2 and q3):');
disp(M_reduced);
