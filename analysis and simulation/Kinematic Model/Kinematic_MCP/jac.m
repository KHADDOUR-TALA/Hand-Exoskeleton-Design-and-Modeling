% Define symbolic variables
syms q1 q2 q3 q4 q5 real
syms L1 L2 L3 L4 L5 L6 H V real
syms m1 m2 m3 m4 real
syms I1 I2 I3 I4 real

% DH Parameters: [a, alpha, d, theta]
DH_table = [
    0, 0, q1, 0;
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
z = sym([0; 0; 1]); % Initial z-axis
p = sym([0; 0; 0]); % Initial position

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

% Extract positions and z-axes for each link
positions = cell(1, n);
z_axes = cell(1, n);
for i = 1:n
    T = T_matrices{i};
    positions{i} = T(1:3, 4);
    if i == 1
        z_axes{i} = z;
    else
        z_axes{i} = T_matrices{i-1}(1:3, 3);
    end
end

% Compute Jv and Jw for each link
Jv = cell(1, n);
Jw = cell(1, n);

for i = 1:n
    Jv{i} = sym(zeros(3, 2)); % 2 active joints
    Jw{i} = sym(zeros(3, 2)); % 2 active joints
    
    for j = 2:3 % Considering q2 and q3 as active joints
        if j == 1
            Jv{i}(:, j-1) = cross(z, positions{i});
        else
            Jv{i}(:, j-1) = cross(z_axes{j}, (positions{i} - positions{j}));
        end
        Jw{i}(:, j-1) = z_axes{j};
    end
end

% Display Jacobians for each link
for i = 1:n
    disp(['Linear Velocity Jacobian Jv for link ', num2str(i), ':']);
    disp(Jv{i});
    disp(['Angular Velocity Jacobian Jw for link ', num2str(i), ':']);
    disp(Jw{i});
end

% Inertia matrix calculation
masses = [m1, m2, m3, m4];
inertias = [I1, I2, I3, I4]; % Diagonal elements of inertia tensors
M = sym(zeros(2, 2)); % 2x2 Inertia matrix for 2 active joints

for i = 1:n
    if i~=4
      
   
    Jv_i = Jv{i}; % Linear velocity Jacobian for link i
    Jw_i = Jw{i}; % Angular velocity Jacobian for link i
    i
    % Kinetic energy due to linear velocity
    if i==5 
        i=4;
    end
    
    M = M + masses(i) * (Jv_i.' * Jv_i);
    
    % Kinetic energy due to angular velocity
    M = M + Jw_i.' * inertias(i) * Jw_i;
    end
end

% Display the inertia matrix
disp('Inertia matrix:');
disp(M);
