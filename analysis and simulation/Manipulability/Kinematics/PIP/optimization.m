clc
clear
% Define initial guesses for link lengths
L0 = [25, 23]; 
q6_max = 60 * pi / 180;

theta_range = linspace(-30 * pi / 180, 90 * pi / 180, 5);
q5_range = linspace(5.5,30,5);
[Theta, Q5] = meshgrid(theta_range, q5_range);
global W_v; % Declare W_v as global
W_v = zeros(size(Theta));
W = diag([1 / q6_max]);
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

objective = @(L) -computeGlobalManipulability(L, Theta, Q5, W);

constraints = [];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [10, 15]; % Lower bounds for L7, L8
ub = [25, 30]; % Upper bounds for L7, L8

[L_opt, fval] = fmincon(objective, L0, A, b, Aeq, beq, lb, ub, constraints, options);

disp(['Optimized Link Lengths: L7 = ', num2str(L_opt(1)), ', L8 = ', num2str(L_opt(2))]);

figure;
surf(Q5, Theta, W_v);
xlabel('q5');
ylabel('\theta_3');
zlabel('Kinematic Manipulability Measure W_v');
title('Kinematic Manipulability Measure');
colorbar;




