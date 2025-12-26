clc
clear
L1 = 12;
L2 = 20;
L3 = 45;
L4 = 15;
V = 47;
H = 20;
q2_max = 20;
q3_max = 60;

% Weighting matrix
W = diag([1, 1]);

theta1_range = linspace(-10, 10, 100);
theta2_range = linspace(-20, 90, 100);
[Theta1, Theta2] = meshgrid(theta1_range, theta2_range);

W_v = zeros(size(Theta1));

for i = 1:length(theta1_range)
    for j = 1:length(theta2_range)
        theta1 = Theta1(j, i);
        theta2 = Theta2(j, i);
        theta1=theta1*pi/180;
        theta2=theta2*pi/180;

        % Inverse Kinematics calculations
        q2 = theta1;
        M = (H - L3 * sin(theta2)) * cos(theta1);
        N = L4 * cos(theta1);
        P = L1 - V + (L2 + L3 * cos(theta2)) * cos(theta1);
        arg_q3 = (P - N * tan(theta2)) / (L2 + L3 * cos(theta2));

        % Clamp argument to the range [-1, 1] to avoid complex values
        arg_q3 = min(max(arg_q3, -1), 1);
        q3 = acos(arg_q3);

        syms q2_sym q3_sym real
        theta1_sym = q2_sym;
        M_sym = (H - L3 * sin(q3_sym)) * cos(q2_sym);
        N_sym = L4 * cos(q2_sym);
        P_sym = L1 - V + (L2 + L3 * cos(q3_sym)) * cos(q2_sym);
        theta2_sym = 2 * atan((M_sym + sqrt(M_sym^2 - N_sym^2 + P_sym^2)) / (N_sym + P_sym));
        x_sym = (H - L4 * sin(theta2_sym + pi/2) - L3 * sin(q3_sym)) / cos(theta2_sym + pi/2);
        q1_sym = (L1 - V) * tan(q2_sym);

        X_sym = [theta1_sym; x_sym; theta2_sym; q1_sym];
        phi_sym = [q2_sym; q3_sym];
        J_sym = jacobian(X_sym, phi_sym);
        
        J = double(subs(J_sym, {q2_sym, q3_sym}, {q2, q3}));

        J_W_inv = J /W;
        local_manipulability = sqrt(abs(det(J_W_inv * J_W_inv')));

        % Store the local manipulability measure
        W_v(j, i) = local_manipulability;
    end
end

global_manipulability = sum(W_v(:)) * (theta1_range(2) - theta1_range(1)) * (theta2_range(2) - theta2_range(1));
fprintf('Global Manipulability Measure: %f\n', global_manipulability);

figure;
surf(Theta1, Theta2, W_v);
xlabel('\theta_1');
ylabel('\theta_2');
zlabel('Local Manipulability Measure');
title('Local Kinematic Manipulability Measure');
colorbar;
